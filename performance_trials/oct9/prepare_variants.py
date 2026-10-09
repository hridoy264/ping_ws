"""Clone immutable Oct8 physical input for supported engine diagnostics only."""
from pathlib import Path
import copy, hashlib, json, xml.etree.ElementTree as ET
out=Path(__file__).resolve().parent
src=out.parent/'oct8'
manifest={'purpose':'Installed-engine diagnostics only; no canonical geometry changes or acceptance claim', 'source_world_sha256': hashlib.sha256((src/'full_100.sdf').read_bytes()).hexdigest(), 'snapshot_resource_path':'../oct8/models','worlds':[]}
variants=[('full_100_ode',100,'ode',None,None),('full_100_dart',100,'dart',None,None),('full_100_ode_pgs',100,'ode','pgs',None),('full_1_native_bullet',1,None,None,'ignition-physics-bullet-plugin'),('full_100_native_bullet',100,None,None,'ignition-physics-bullet-plugin')]
for name,count,detector,solver,engine in variants:
 root=ET.parse(src/f'full_{count}.sdf').getroot(); world=root.find('world');world.set('name',name)
 if detector or solver:
  dart=ET.SubElement(world.find('physics'),'dart')
  if detector: ET.SubElement(dart,'collision_detector').text=detector
  if solver: ET.SubElement(ET.SubElement(dart,'solver'),'solver_type').text=solver
 if engine:
  physics=next(p for p in world.findall('plugin') if p.get('name')=='ignition::gazebo::systems::Physics')
  ET.SubElement(physics,'engine',{'filename':engine})
 target=out/(name+'.sdf'); ET.indent(root); ET.ElementTree(root).write(target,encoding='utf-8',xml_declaration=True)
 manifest['worlds'].append({'name':name,'file':target.name,'count':count,'scope':'full','time_step_s':.001,'dart_collision_detector':detector or 'default','dart_constraint_solver':solver or 'default','physics_engine':engine or 'DART','robot_collisions':377,'sha256':hashlib.sha256(target.read_bytes()).hexdigest()})
(out/'benchmark_worlds_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
print([c['name'] for c in manifest['worlds']])
