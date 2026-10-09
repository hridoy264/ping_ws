#!/usr/bin/env python3
"""Unloaded native joint test. Not a 100-ball feeding or launch validation."""
import argparse,json,os,queue,subprocess,threading,time,uuid
from ament_index_python.packages import get_package_prefix
from pathlib import Path
import xml.etree.ElementTree as ET
import physical_feed_trial as feed
from test_physical_monitor import stop
PACKAGE=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir',type=Path,default=PACKAGE.parents[1]/'head_meter_runtime')
OUT=parser.parse_args().output_dir.resolve();OUT.mkdir(parents=True,exist_ok=True)
feed.JOINTS=feed.JOINTS+('head_meter_roller_joint','head_meter_motor_joint','head_meter_idler_joint','head_meter_idler_slide')
root=ET.parse(PACKAGE/'worlds/physical_100.sdf').getroot();world=root.find('world');world.set('name','meter_joint_check')
for m in list(world.findall('model')):
 if m.get('name','').startswith('inventory_ball_'):world.remove(m)
ET.ElementTree(root).write(OUT/'unloaded.sdf')
env=dict(os.environ,IGN_PARTITION='meter-'+uuid.uuid4().hex)
env['IGN_GAZEBO_RESOURCE_PATH']=str(PACKAGE/'models')+':'+env.get('IGN_GAZEBO_RESOURCE_PATH','')
env['IGN_GAZEBO_SYSTEM_PLUGIN_PATH']=str(Path(get_package_prefix('pingpong_launcher_sim'))/'lib')+':'+env.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH','')
q=queue.Queue();server=sub=None;report={'scope':'Unloaded full mechanism; no balls in this diagnostic; canonical world still contains100.','pass':False,'samples':[]}
def command(topic,value):
 subprocess.run(['ign','topic','-t',topic,'-m','ignition.msgs.Double','-p','data: '+str(value)],env=env,check=True,timeout=8,stdout=subprocess.DEVNULL)
try:
 log=(OUT/'gazebo.log').open('w')
 sub=subprocess.Popen(['ign','topic','-t','/pingpong/joint_states','-e'],env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1,start_new_session=True)
 threading.Thread(target=feed.read_joints,args=(sub.stdout,q,'pingpong_r10'),daemon=True).start()
 server=subprocess.Popen(['ign','gazebo','-s','-r',str(OUT/'unloaded.sdf')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 deadline=time.monotonic()+70;phase=0;start=0;checkpoints=[]
 while time.monotonic()<deadline:
  if server.poll() is not None:raise RuntimeError('Gazebo exited; inspect log')
  try:kind,p=q.get(timeout=.3)
  except queue.Empty:continue
  if kind=='error':raise RuntimeError(p)
  if kind!='joint':continue
  now=p['sim_time'];js=p['joints'];report['samples'].append(p)
  assert -.00001<=js['head_meter_idler_slide']['position']<=.00201
  v=js['head_meter_roller_joint']['velocity']
  if phase==0:
   command('/pingpong/head_meter_velocity_cmd',2);command('/pingpong/yaw_cmd',.15);command('/pingpong/pitch_cmd',.1);phase=1
  elif phase==1 and abs(v-2)<.05:
   checkpoints.append(p);start=now;phase=2
  elif phase==2 and now-start>.8:
   checkpoints.append(p);command('/pingpong/head_meter_velocity_cmd',-2);phase=3
  elif phase==3 and abs(v+2)<.05:
   reverse_start=p;start=now;phase=4
  elif phase==4 and now-start>.8:
   checkpoints.append(p);command('/pingpong/head_meter_velocity_cmd',0);phase=5
  elif phase==5 and abs(v)<.05:
   checkpoints.append(p);break
 else:raise RuntimeError('Joint test timed out')
 assert len(checkpoints)==4,'Missing command phase'
 a,b,c,d=checkpoints
 for name in ['head_meter_roller_joint','head_meter_motor_joint']:
  assert b['joints'][name]['position']-a['joints'][name]['position']>1,name+' did not advance'
  assert c['joints'][name]['position']-reverse_start['joints'][name]['position']<-1,name+' did not reverse'
  assert abs(d['joints'][name]['velocity'])<.05,name+' did not stop'
 assert abs(d['joints']['yaw_joint']['position']-.15)<.02
 assert abs(d['joints']['pitch_joint']['position']-.1)<.02
 report['pass']=True;report['phases']=checkpoints;report['reverse_acknowledged']=reverse_start;report['feeding_pass']=False
except Exception as error:report['error']=str(error)
finally:
 stop(server);stop(sub)
 report['sample_count']=len(report['samples']);report['last']=report['samples'][-1] if report['samples'] else None
 (OUT/'samples.jsonl').write_text(''.join(json.dumps(p)+'\n' for p in report.pop('samples')))
 (OUT/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
raise SystemExit(0 if report['pass'] else 1)
