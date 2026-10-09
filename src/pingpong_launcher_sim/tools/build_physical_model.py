#!/usr/bin/env python3
"""Generate the R10 physical development model and exactly 100 persistent balls.

All values are metres. Convex collision pieces preserve the feeder pockets and
the hollow feed route. This is reproducible geometry, not feeding qualification.
"""
import argparse
import copy
import itertools
import json
import math
from pathlib import Path
import shutil
import struct
import sys
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from pingpong_launcher_sim.inventory import AABB, append_to_world, pack_balls

MODEL = PACKAGE / 'models/pingpong_r10'
FEEDER = (-.130, -.0588, -.070)
BIN = (-.192, -.0588)
YAW = (.040, -.1208, 0)
PITCH = (.040, -.060, .185)
DECK_HEIGHT = .910
GRAPHITE = '.12 .14 .17 1'
HOPPER_COLOR = '.24 .27 .31 1'
ORANGE = '.90 .33 .08 1'
CYAN = '.03 .55 .64 1'
MESH_RECORDS = []


def node(parent, tag, text=None, **attributes):
    result = ET.SubElement(parent, tag, attributes)
    if text is not None:
        result.text = str(text) if isinstance(text, (str, int, float)) else ' '.join(f'{v:.12g}' for v in text)
    return result


def add(a, b): return tuple(x + y for x, y in zip(a, b))
def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def mul(a, scale): return tuple(x * scale for x in a)
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def cross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def norm(a): return math.sqrt(dot(a, a))
def unit(a): return mul(a, 1 / norm(a))


def material(parent, color):
    item = node(parent, 'material')
    node(item, 'ambient', color); node(item, 'diffuse', color)


def surface(collision):
    s = node(collision, 'surface')
    ode = node(node(s, 'friction'), 'ode')
    node(ode, 'mu', .3); node(ode, 'mu2', .3)
    bounce = node(s, 'bounce')
    node(bounce, 'restitution_coefficient', .1); node(bounce, 'threshold', .1)


def box(link, name, low, high, color=GRAPHITE, collision=True, visual=True):
    size, center = sub(high, low), mul(add(high, low), .5)
    assert min(size) > 0, name
    for tag in (['collision'] if collision else []) + (['visual'] if visual else []):
        item = node(link, tag, name=name if tag == 'collision' else name+'_visual')
        node(item, 'pose', (*center, 0, 0, 0))
        node(node(node(item, 'geometry'), 'box'), 'size', size)
        surface(item) if tag == 'collision' else material(item, color)


def oriented_box(link,name,center,size,axes,color=GRAPHITE,visual=False):
    # Axes are orthonormal local X,Y,Z basis vectors, stored as matrix columns.
    matrix=tuple(tuple(axes[j][i] for j in range(3)) for i in range(3))
    pitch=math.asin(max(-1,min(1,-matrix[2][0])))
    if abs(math.cos(pitch))>1e-9:
        roll=math.atan2(matrix[2][1],matrix[2][2]);yaw=math.atan2(matrix[1][0],matrix[0][0])
    else:roll=0;yaw=math.atan2(-matrix[0][1],matrix[1][1])
    for tag in ('collision','visual') if visual else ('collision',):
        item=node(link,tag,name=name if tag=='collision' else name+'_visual')
        node(item,'pose',(*center,roll,pitch,yaw))
        node(node(node(item,'geometry'),'box'),'size',size)
        surface(item) if tag=='collision' else material(item,color)


def cylinder(link,name,center,radius,length,color=GRAPHITE):
    for tag in ('collision','visual'):
        item=node(link,tag,name=name if tag=='collision' else name+'_visual')
        node(item,'pose',(*center,0,0,0))
        shape=node(node(item,'geometry'),'cylinder');node(shape,'radius',radius);node(shape,'length',length)
        surface(item) if tag=='collision' else material(item,color)


def ring_boxes(link,label,start,end,inner,thickness,sectors=8,omit=None):
    axis=unit(sub(end,start));center=mul(add(start,end),.5);length=norm(sub(end,start))
    reference=min(((1,0,0),(0,1,0),(0,0,1)),key=lambda a:abs(dot(a,axis)))
    u=unit(cross(axis,reference));v=cross(axis,u)
    for k in range(sectors):
        angle=math.tau*k/sectors
        radial=add(mul(u,math.cos(angle)),mul(v,math.sin(angle)))
        tangent=cross(radial,axis)
        p=add(center,mul(radial,inner+thickness/2))
        if omit and omit(p):continue
        oriented_box(link,f'{label}_{k:02d}',p,
                     (length,2*(inner+thickness)*math.tan(math.pi/sectors),thickness),
                     (axis,tangent,radial))


def area(poly):
    return sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(poly, poly[1:]+poly[:1])) / 2


def circle(radius, center=(0, 0), segments=64):
    return [(center[0]+radius*math.cos(math.tau*i/segments),
             center[1]+radius*math.sin(math.tau*i/segments)) for i in range(segments)]


def rectangle(x1, y1, x2, y2): return [(x1,y1),(x2,y1),(x2,y2),(x1,y2)]


def clip(poly, a, b, inside=True):
    """Clip a convex polygon against an oriented infinite half-plane."""
    if not poly: return []
    sign = 1 if inside else -1
    def side(p): return sign*((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0]))
    result = []
    previous, old = poly[-1], side(poly[-1])
    for point in poly:
        new = side(point)
        if (new >= -1e-14) != (old >= -1e-14):
            fraction = old / (old - new)
            result.append(tuple(x + fraction*(y-x) for x,y in zip(previous, point)))
        if new >= -1e-14: result.append(point)
        previous, old = point, new
    unique = []
    for point in result:
        if not unique or math.dist(point, unique[-1]) > 1e-10: unique.append(point)
    if len(unique) > 1 and math.dist(unique[0], unique[-1]) < 1e-10: unique.pop()
    return unique if len(unique) >= 3 and area(unique) > 1e-11 else []


def subtract_convex(poly, cutter):
    """Disjoint convex partition of poly minus a convex CCW cutter."""
    remaining, pieces = poly, []
    for a,b in zip(cutter, cutter[1:]+cutter[:1]):
        outside = clip(remaining, a, b, False)
        if outside: pieces.append(outside)
        remaining = clip(remaining, a, b, True)
        if not remaining: break
    return pieces


def subtract_all(polygons, cutters):
    for cutter in cutters:
        polygons = [piece for polygon in polygons for piece in subtract_convex(polygon, cutter)]
    return polygons


def turn(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def hull2(points):
    points=sorted(set(points))
    lower=[];upper=[]
    for point in points:
        while len(lower)>=2 and turn(lower[-2],lower[-1],point)<=1e-13:lower.pop()
        lower.append(point)
    for point in reversed(points):
        while len(upper)>=2 and turn(upper[-2],upper[-1],point)<=1e-13:upper.pop()
        upper.append(point)
    return lower[:-1]+upper[:-1]


def merge_convex(polygons):
    """Merge shared-edge cells only when their union is already convex."""
    for _ in range(50):
        edges={}
        for i,polygon in enumerate(polygons):
            for a,b in zip(polygon,polygon[1:]+polygon[:1]):
                key=tuple(sorted((tuple(round(v,10) for v in a),tuple(round(v,10) for v in b))))
                edges.setdefault(key,[]).append(i)
        used=set();merged=[]
        for neighbors in edges.values():
            if len(neighbors)!=2:continue
            i,j=neighbors
            if i==j or i in used or j in used:continue
            union=hull2(polygons[i]+polygons[j])
            if abs(area(union)-area(polygons[i])-area(polygons[j]))<1e-11:
                merged.append(union);used.update((i,j))
        if not used:break
        polygons=merged+[p for i,p in enumerate(polygons) if i not in used]
    return polygons


def rotor_outline():
    """Disk minus six open circle notches, as one simple CCW outer boundary."""
    outer,orbit,pocket=.079,.062,.026
    delta=math.acos((outer*outer+orbit*orbit-pocket*pocket)/(2*outer*orbit))
    gamma=math.atan2(outer*math.sin(delta),outer*math.cos(delta)-orbit)
    points=[]
    for i in range(6):
        alpha=i*math.pi/3;beta=(i+1)*math.pi/3
        start,end=alpha+delta,beta-delta
        steps=math.ceil((end-start)/(math.tau/72))
        for j in range(steps):
            a=start+(end-start)*j/steps;points.append((outer*math.cos(a),outer*math.sin(a)))
        center=(orbit*math.cos(beta),orbit*math.sin(beta))
        start,end=beta-gamma,beta+gamma-math.tau
        # <=0.32 mm chord error on the R26 pocket keeps contacts manageable.
        steps=math.ceil((start-end)/(math.tau/20))
        for j in range(steps):
            a=start+(end-start)*j/steps
            points.append((center[0]+pocket*math.cos(a),center[1]+pocket*math.sin(a)))
    return points


def triangulate(poly):
    """Ear clipping for the simple rotor boundary; holes are cut afterward."""
    remaining=list(poly);triangles=[]
    while len(remaining)>3:
        for i,b in enumerate(remaining):
            a=remaining[i-1];c=remaining[(i+1)%len(remaining)]
            if turn(a,b,c)<=1e-13:continue
            if any(turn(a,b,p)>=-1e-13 and turn(b,c,p)>=-1e-13 and turn(c,a,p)>=-1e-13
                   for j,p in enumerate(remaining) if j not in (i,(i-1)%len(remaining),(i+1)%len(remaining))):
                continue
            triangles.append([a,b,c]);remaining.pop(i);break
        else:raise ValueError('Could not triangulate simple rotor boundary')
    triangles.append(remaining)
    return triangles


def prism_triangles(poly, z1, z2, offset=(0, 0, 0)):
    bottom = [add((*p,z1),offset) for p in poly]
    top = [add((*p,z2),offset) for p in poly]
    triangles = []
    for i in range(1, len(poly)-1):
        triangles.extend([(bottom[0],bottom[i+1],bottom[i]), (top[0],top[i],top[i+1])])
    for i in range(len(poly)):
        j = (i+1)%len(poly)
        triangles.extend([(bottom[i],bottom[j],top[j]), (bottom[i],top[j],top[i])])
    return triangles


def convex_hull_triangles(points):
    """Small (6/8 point) convex cells, exported as watertight outward facets."""
    faces = {}
    for i,j,k in itertools.combinations(range(len(points)), 3):
        n = cross(sub(points[j],points[i]),sub(points[k],points[i]))
        if norm(n) < 1e-12: continue
        n = unit(n); d = dot(n,points[i]); values = [dot(n,p)-d for p in points]
        if max(values) <= 1e-9: pass
        elif min(values) >= -1e-9:
            n = mul(n,-1); d = -d; values = [-v for v in values]
        else: continue
        indices = frozenset(index for index,value in enumerate(values) if abs(value) <= 1e-9)
        if indices in faces: continue
        center = tuple(sum(points[index][axis] for index in indices)/len(indices) for axis in range(3))
        u = unit(sub(points[next(iter(indices))],center)); v = cross(n,u)
        projected = {(dot(sub(points[index],center),u), dot(sub(points[index],center),v)): index
                     for index in indices}
        ordered = [projected[p] for p in hull2(projected)]
        faces[indices] = [(points[ordered[0]],points[ordered[q]],points[ordered[q+1]])
                          for q in range(1,len(ordered)-1)]
    return [triangle for face in faces.values() for triangle in face]


def mesh(link, name, triangles, color, *, visual=True,collision=True):
    path = MODEL / 'meshes' / (name + '.stl')
    volume = sum(dot(a,cross(b,c))/6 for a,b,c in triangles)
    if volume <= 1e-13: raise ValueError(f'Nonpositive collision prism {name}: {volume}')
    payload = bytearray(b'R10 convex collision cell; metres'.ljust(80,b'\0'))
    payload.extend(struct.pack('<I',len(triangles)))
    for a,b,c in triangles:
        n = unit(cross(sub(b,a),sub(c,a)))
        payload.extend(struct.pack('<12fH',*n,*a,*b,*c,0))
    path.write_bytes(payload)
    for tag in (['collision'] if collision else [])+(['visual'] if visual else []):
        item = node(link,tag,name=name if tag == 'collision' else name+'_visual')
        shape = node(node(item,'geometry'),'mesh')
        node(shape,'uri','model://pingpong_r10/meshes/'+path.name)
        node(shape,'scale','1 1 1')
        surface(item) if tag == 'collision' else material(item,color)
    MESH_RECORDS.append({'file':path.name,'link':link.get('name'),'volume_m3':volume,
                         'triangles':len(triangles),'convex_cell':collision,'collision':collision})


def extruded_partition(link, label, polygons, z1,z2,offset,color):
    for i,polygon in enumerate(merge_convex(polygons)):
        mesh(link,f'{label}_{i:03d}',prism_triangles(polygon,z1,z2,offset),color)


def tube(link,label,centres,tangents,origin=(0,0,0),inner=.023,outer=.026,sectors=16):
    """Hollow piecewise-convex annular cells; no hull spans the complete bore."""
    rings=[]
    # One fixed binormal avoids a 90-degree frame flip at an elbow endpoint.
    reference=min(((1,0,0),(0,1,0),(0,0,1)),
                  key=lambda axis:sum(abs(dot(axis,tangent)) for tangent in tangents))
    for center,tangent in zip(centres,tangents):
        u=unit(cross(tangent,reference));v=cross(tangent,u)
        rings.append([[sub(add(center,mul(add(mul(u,math.cos(math.tau*k/sectors)),
                                               mul(v,math.sin(math.tau*k/sectors))),radius)),origin)
                       for k in range(sectors)] for radius in (inner,outer)])
    visual_triangles=[]
    for segment in range(len(rings)-1):
        for k in range(sectors):
            j=(k+1)%sectors
            points=[rings[s][r][a] for s in (segment,segment+1) for r in (0,1) for a in (k,j)]
            visual_triangles.extend(convex_hull_triangles(points))
        a,b=sub(centres[segment],origin),sub(centres[segment+1],origin)
        direction=unit(sub(b,a))
        # 0.5 mm overlap at each chord end closes panel seams. The octagonal
        # collision bore is circumscribed around the nominal inner radius.
        ring_boxes(link,f'{label}_wall_{segment:02d}',sub(a,mul(direction,.0005)),
                   add(b,mul(direction,.0005)),inner,outer-inner,sectors=8)
    mesh(link,label+'_visual_mesh',visual_triangles,GRAPHITE,collision=False)


def link(model,name,position,mass,size,com=(0,0,0)):
    result=node(model,'link',name=name)
    node(result,'pose',(*position,0,0,0));node(result,'self_collide','false')
    inertia=node(result,'inertial');node(inertia,'pose',(*com,0,0,0));node(inertia,'mass',mass)
    tensor=node(inertia,'inertia')
    x,y,z=size
    for axis,value in zip(('ixx','iyy','izz'),(mass*(y*y+z*z)/12,mass*(x*x+z*z)/12,mass*(x*x+y*y)/12)):
        node(tensor,axis,value)
    for axis in ('ixy','ixz','iyz'):node(tensor,axis,0)
    return result


def joint(model,name,parent,child,axis=None,limits=None,velocity=1,effort=1):
    result=node(model,'joint',name=name,type='revolute' if axis else 'fixed')
    node(result,'parent',parent);node(result,'child',child);node(result,'pose','0 0 0 0 0 0',relative_to=child)
    if axis:
        a=node(result,'axis');node(a,'xyz',axis);lim=node(a,'limit')
        node(lim,'lower',limits[0] if limits else -1e16);node(lim,'upper',limits[1] if limits else 1e16)
        node(lim,'velocity',velocity);node(lim,'effort',effort)


def controller(model,name,topic,position=False,speed=1):
    kind='joint-position-controller-system' if position else 'joint-controller-system'
    cls='JointPositionController' if position else 'JointController'
    p=node(model,'plugin',filename='ignition-gazebo-'+kind,name='ignition::gazebo::systems::'+cls)
    node(p,'joint_name',name);node(p,'topic',topic)
    if position:
        node(p,'use_velocity_commands','true');node(p,'initial_position',0)
        node(p,'cmd_max',speed);node(p,'cmd_min',-speed)
    else:
        node(p,'use_force_commands','false');node(p,'initial_velocity',0)


def import_head_visuals_and_collisions(model,head):
    old=ET.parse(PACKAGE/'models/pingpong_launcher/model.sdf').getroot().find('model')
    original=old.find("link[@name='head_link']")
    shift=(.132,.060,0)
    for item in original:
        if item.tag not in ('visual','collision'):continue
        if item.tag=='collision' and not item.get('name','').startswith('motor_'):continue
        item=copy.deepcopy(item)
        pose=item.find('pose')
        if pose is None:pose=node(item,'pose','0 0 0 0 0 0')
        values=[float(v) for v in pose.text.split()]
        pose.text=' '.join(str(v) for v in (*add(values[:3],shift),*values[3:]))
        for uri in item.findall('.//mesh/uri'):
            source=PACKAGE/'models'/uri.text.removeprefix('model://')
            destination=MODEL/'meshes'/('head_'+source.name)
            shutil.copyfile(source,destination)
            uri.text='model://pingpong_r10/meshes/'+destination.name
        head.append(item)
    # Coarse primitive enclosure retains all central openings. Detailed R5
    # meshes remain visual only; contact rubber remains an explicit limitation.
    cy=.060
    ring_boxes(head,'rear_shell',(.100,cy,0),(.163,cy,0),.107,.003)
    ring_boxes(head,'rear_plate',(.098,cy,0),(.104,cy,0),.026,.078)
    ring_boxes(head,'front_collar',(.165,cy,0),(.185,cy,0),.107,.003)
    ring_boxes(head,'head_inlet',(.060,cy,0),(.132,cy,0),.0215,.003)
    for k in range(8):
        a=math.tau*k/8;radial=(0,math.cos(a),math.sin(a))
        p=add((.185,cy,0),mul(radial,.107));q=add((.220,cy,0),mul(radial,.045))
        slope=unit(sub(q,p));normal=unit(sub(radial,mul(slope,dot(radial,slope))))
        width=cross(slope,normal);center=add(mul(add(p,q),.5),mul(normal,.0015))
        oriented_box(head,f'cover_slope_{k}',center,
                     (2*.110*math.tan(math.pi/8),norm(sub(q,p))+.001,.003),(width,slope,normal))
    for i in range(1,4):
        wheel=copy.deepcopy(old.find(f"link[@name='wheel_{i}_link']"))
        pose=wheel.find('pose');values=[float(v) for v in pose.text.split()]
        pose.text=' '.join(str(v) for v in (*add(values[:3],shift),*values[3:]))
        pose.set('relative_to','head_link')
        for uri in wheel.findall('.//mesh/uri'):
            source=PACKAGE/'models'/uri.text.removeprefix('model://')
            destination=MODEL/'meshes'/('head_'+source.name)
            shutil.copyfile(source,destination);uri.text='model://pingpong_r10/meshes/'+destination.name
        model.append(wheel)
        model.append(copy.deepcopy(old.find(f"joint[@name='wheel_{i}_joint']")))
        controller(model,f'wheel_{i}_joint',f'/pingpong/wheel_{i}_cmd')


def generate(seed=42):
    (MODEL/'meshes').mkdir(parents=True,exist_ok=True)
    MESH_RECORDS.clear()
    root=ET.Element('sdf',version='1.8');model=node(root,'model',name='pingpong_r10',canonical_link='base_link')
    node(model,'static','false');node(model,'self_collide','false')
    base=link(model,'base_link',(0,0,0),5.0,(.60,.35,.145),(-.030,0,-.070))
    yaw=link(model,'yaw_link',YAW,.35,(.10,.12,.20),(0,.035,.14))
    head=link(model,'head_link',PITCH,1.10,(.24,.24,.24),(.14,.06,0))
    rotor=link(model,'feeder_link',FEEDER,.09,(.158,.158,.0233),(0,0,.01235))
    joint(model,'world_fixed','world','base_link')
    # Match the native fixed-first Fusion command convention. Positive pitch
    # raises the muzzle (+Z); positive yaw turns the forward axis toward -Y.
    joint(model,'yaw_joint','base_link','yaw_link',(0,0,-1),(-math.radians(25),math.radians(25)),.6,2)
    joint(model,'pitch_joint','yaw_link','head_link',(0,-1,0),(-math.radians(20),math.radians(30)),.45,1.8)
    joint(model,'feeder_joint','base_link','feeder_link',(0,0,1),velocity=2,effort=.5)
    box(base,'case_floor',(-.330,-.175,-.145),(.270,.175,-.141))
    box(base,'case_left',(-.330,-.175,-.141),(.270,-.172,-.004))
    box(base,'case_right',(-.330,.172,-.141),(.270,.175,-.004))
    box(base,'case_rear',(-.330,-.172,-.141),(-.327,.172,-.004))
    box(base,'case_front',(.267,-.172,-.141),(.270,.172,-.004))
    box(base,'mains_divider',(.070,-.168,-.141),(.073,.168,-.007))
    deck=subtract_all([rectangle(-.330,-.175,.270,.175)],
                      [rectangle(BIN[0]-.0255,BIN[1]-.0255,BIN[0]+.0255,BIN[1]+.0255),
                       rectangle(YAW[0]-.043,YAW[1]-.043,YAW[0]+.043,YAW[1]+.043)])
    for i,p in enumerate(deck):
        box(base,f'deck_{i}',(min(v[0] for v in p),min(v[1] for v in p),-.004),
            (max(v[0] for v in p),max(v[1] for v in p),0))
    box(base,'psu_envelope',(.080,-.140,-.126),(.195,.075,-.076),'.55 .57 .60 1',collision=False)
    box(base,'pcb_envelope',(-.309,.065,-.125),(-.159,.145,-.090),'.05 .33 .20 1',collision=False)
    for i,(x,y) in enumerate(((-.297,-.141),(-.297,.141),(.237,-.141),(.237,.141))):
        box(base,f'foot_{i}',(x-.014,y-.014,-.151),(x+.014,y+.014,-.145),'.05 .05 .05 1')
    # Exact R6 planar constructive geometry, with screw recesses omitted.
    disk=circle(.091,segments=72);channel=rectangle(0,-.094,.100,-.030)
    footprint=[disk]+subtract_convex(channel,disk)
    cylinder(base,'feeder_floor',add(FEEDER,(0,0,-.003)),.091,.006)
    box(base,'channel_floor',add(FEEDER,(0,-.094,-.006)),add(FEEDER,(.100,-.030,0)))
    ring_boxes(base,'pan_wall',FEEDER,add(FEEDER,(0,0,.043)),.0845,.0065,sectors=24,
               omit=lambda p:p[0]>FEEDER[0]+.005 and FEEDER[1]-.090<p[1]<FEEDER[1]-.033)
    box(base,'channel_lower_wall',add(FEEDER,(0,-.094,0)),add(FEEDER,(.100,-.084,.043)))
    box(base,'channel_upper_wall',add(FEEDER,(.078,-.040,0)),add(FEEDER,(.100,-.030,.043)))
    roof=subtract_all(footprint,[circle(.022/math.cos(math.pi/16),(-.062,0),16)])
    extruded_partition(base,'feeder_roof',roof,.043,.048,FEEDER,GRAPHITE)
    rotor_pieces=merge_convex(triangulate(rotor_outline()))
    # The tiny shaft bore is omitted from collision geometry: balls cannot
    # reach this enclosed hub, and the revolute joint represents its shaft.
    extruded_partition(rotor,'rotor',rotor_pieces,.0007,.024,(0,0,0),ORANGE)
    box(base,'stripper',add(FEEDER,(.005,-.040,.025)),add(FEEDER,(.070,-.030,.043)),ORANGE)
    extruded_partition(base,'stripper_nose',[circle(.005,(.005,-.035),24)],.025,.043,FEEDER,ORANGE)
    # Hopper straight upper walls and circular neck.
    x,y=BIN
    box(base,'hopper_front',(x+.080,y-.108,.208),(x+.083,y+.108,.348),HOPPER_COLOR)
    box(base,'hopper_back',(x-.083,y-.108,.208),(x-.080,y+.108,.348),HOPPER_COLOR)
    box(base,'hopper_left',(x-.080,y-.108,.208),(x+.080,y-.105,.348),HOPPER_COLOR)
    box(base,'hopper_right',(x-.080,y+.105,.208),(x+.080,y+.108,.348),HOPPER_COLOR)
    tube(base,'loading_neck',[(x,y,-.022),(x,y,.066)],[(0,0,1)]*2,inner=.022,outer=.025,sectors=32)
    lower=[];upper=[]
    funnel_visual=[]
    for i in range(32):
        a=math.tau*i/32;ca,sa=math.cos(a),math.sin(a)
        reach=min(.080/abs(ca) if abs(ca)>1e-10 else math.inf,
                  .105/abs(sa) if abs(sa)>1e-10 else math.inf)
        lower.append((x+.022*ca,y+.022*sa,.065))
        upper.append((x+reach*ca,y+reach*sa,.208))
    for i in range(32):
        j=(i+1)%32
        for q,(a,b,c) in enumerate(((lower[i],lower[j],upper[j]),(lower[i],upper[j],upper[i]))):
            normal=unit(cross(sub(b,a),sub(c,a)))
            center=mul(add(add(a,b),c),1/3)
            if dot(normal,(center[0]-x,center[1]-y,0))<0:normal=mul(normal,-1)
            points=[a,b,c]+[add(p,mul(normal,.003)) for p in (a,b,c)]
            funnel_visual.extend(convex_hull_triangles(points))
    mesh(base,'funnel_visual_mesh',funnel_visual,HOPPER_COLOR,collision=False)
    for axis,halfwidth,other in ((0,.080,.105),(1,.105,.080)):
        for sign in (-1,1):
            bottom=[x,y,.065];top=[x,y,.208]
            bottom[axis]+=sign*.022;top[axis]+=sign*halfwidth
            slope=unit(sub(top,bottom));outward=[0.,0.,0.];outward[axis]=sign
            normal=unit(sub(outward,mul(slope,dot(outward,slope))))
            width=cross(slope,normal)
            center=add(mul(add(bottom,top),.5),mul(normal,.0015))
            oriented_box(base,f'funnel_slope_{axis}_{sign}',center,
                         (2*other+.006,norm(sub(top,bottom))+.001,.003),(width,slope,normal))
    # Constant-length route through two coaxial rotary interfaces.
    tube(base,'pan_outlet',[(-.030,-.1208,-.048),(-.020,-.1208,-.048)],[(1,0,0)]*2)
    angles=[math.pi*i/12 for i in range(7)]
    tube(base,'lower_elbow',[(-.020+.060*math.sin(a),-.1208,.012-.060*math.cos(a)) for a in angles],
         [(math.cos(a),0,math.sin(a)) for a in angles])
    tube(base,'vertical_riser',[(.040,-.1208,.012),(.040,-.1208,.1242)],[(0,0,1)]*2)
    tube(yaw,'yaw_elbow',[(.040,-.0608-.060*math.cos(a),.125+.060*math.sin(a)) for a in angles],
         [(0,math.sin(a),math.cos(a)) for a in angles],YAW)
    tube(head,'pitch_elbow',[(.100-.060*math.cos(a),-.060+.060*math.sin(a),.185) for a in angles],
         [(math.sin(a),math.cos(a),0) for a in angles],PITCH)
    import_head_visuals_and_collisions(model,head)
    from physical_head_meter import add_head_meter
    meter_report=add_head_meter(sys.modules[__name__],model,base,head)
    controller(model,'yaw_joint','/pingpong/yaw_cmd',True,.6)
    controller(model,'pitch_joint','/pingpong/pitch_cmd',True,.45)
    controller(model,'feeder_joint','/pingpong/physical_feeder_velocity_cmd')
    state=node(model,'plugin',filename='ignition-gazebo-joint-state-publisher-system',
               name='ignition::gazebo::systems::JointStatePublisher')
    node(state,'topic','/pingpong/joint_states')
    for name in ('yaw_joint','pitch_joint','feeder_joint','wheel_1_joint','wheel_2_joint','wheel_3_joint','head_meter_roller_joint','head_meter_motor_joint','head_meter_idler_joint','head_meter_idler_slide'):
        node(state,'joint_name',name)
    monitor=node(model,'plugin',filename='libpingpong_physical_ball_monitor.so',name='pingpong::PhysicalBallMonitor')
    for key,value in {'head_link':'head_link','muzzle_pose':'.220 .060 0 0 0 0','aperture_radius':.045,
                      'ball_name_prefix':'inventory_ball_','ball_link':'ball_link',
                      'expected_ball_count':100,'publish_rate':5}.items():node(monitor,key,value)
    ET.indent(root,space='  ');ET.ElementTree(root).write(MODEL/'model.sdf',encoding='utf-8',xml_declaration=True)
    config=ET.Element('model');node(config,'name','Ping Bot R10 physical development model')
    node(config,'version','0.1');node(config,'sdf','model.sdf',version='1.8')
    node(config,'description','Physical 100-ball development model; collision approximations and uncalibrated wheel contact.')
    ET.indent(config,space='  ');ET.ElementTree(config).write(MODEL/'model.config',encoding='utf-8',xml_declaration=True)
    world_root=ET.parse(PACKAGE/'worlds/training.sdf').getroot();world=world_root.find('world')
    world.set('name','physical_100')
    for name in ('table','net'):
        pose=world.find(f"model[@name='{name}']/pose")
        values=[float(value) for value in pose.text.split()];values[0]+=.100
        pose.text=' '.join(str(value) for value in values)
    for include in list(world.findall('include')):world.remove(include)
    old_bench=world.find("model[@name='launcher_bench']");world.remove(old_bench)
    bench=node(world,'model',name='launcher_bench');node(bench,'static','true')
    bench_link=node(bench,'link',name='bench')
    box(bench_link,'bench',(-.390,-.220,0),(.285,.220,.759),'.32 .25 .19 1')
    include=node(world,'include');node(include,'uri','model://pingpong_r10');node(include,'pose',(0,0,DECK_HEIGHT,0,0,0))
    bounds=AABB((x-.080,y-.105,DECK_HEIGHT+.208),(x+.080,y+.105,DECK_HEIGHT+.508))
    packing=pack_balls(bounds,count=100,seed=seed,jitter=.0003)
    world_root=append_to_world(world_root,packing)
    ET.indent(world_root,space='  ')
    world_path=PACKAGE/'worlds/physical_100.sdf'
    ET.ElementTree(world_root).write(world_path,encoding='utf-8',xml_declaration=True)
    routes={
      'pan_outlet':{'centres':[[-.030,-.1208,-.048],[-.020,-.1208,-.048]],'inner_radius_m':.023,'frame':'base'},
      'lower_bend':{'centres':[[-.020+.060*math.sin(a),-.1208,.012-.060*math.cos(a)] for a in angles],
                    'inner_radius_m':.023,'frame':'base'},
      'riser':{'centres':[[.040,-.1208,.012],[.040,-.1208,.1242]],'inner_radius_m':.023,'frame':'base'},
      'yaw_elbow':{'centres':[[.040,-.0608-.060*math.cos(a),.125+.060*math.sin(a)] for a in angles],
                   'inner_radius_m':.023,'frame':'yaw'},
      'head_elbow':{'centres':[[.100-.060*math.cos(a),-.060+.060*math.sin(a),.185] for a in angles],
                    'inner_radius_m':.023,'frame':'head'},
      'head':{'centres':[[.100,0,.185],[.260,0,.185]],'inner_radius_m':.0215,'frame':'head'},
    }
    for route in routes.values():route['coordinate_frame']='neutral_model'
    report={'revision':'R10 physical development','seed':seed,'ball_count':100,'ball_loading':'inside and above upper opening; un-settled',
            'robot_deck_world_z':DECK_HEIGHT,'feeder_origin_m':FEEDER,'bin_center_m':BIN,
            'table_and_net_center_world_x_m':1.670,'bench_table_gap_m':.015,
            'yaw_origin_m':YAW,'pitch_origin_m':PITCH,'muzzle_model_m':(.260,0,.185),
            'aim_command_convention':{'yaw_axis':[0,0,-1],'pitch_axis':[0,-1,0],
                'yaw_limits_deg':[-25,25],'pitch_limits_deg':[-20,30],
                'positive_pitch':'muzzle rises','positive_yaw':'forward axis turns toward -Y',
                'scope':'Matches native CAD commands; nine native CAD poses checked, including integrated meter and front cover.'},
            'feed_route_model_m':routes,'head_meter_and_drain':meter_report,
            'hopper_upper_inner_bounds_m':[[x-.080,y-.105,.208],[x+.080,y+.105,.348]],
            'initial_ball_bounds_world_m':[bounds.minimum,bounds.maximum],
            'convex_collision_mesh_count':sum(m['collision'] for m in MESH_RECORDS),
            'total_collision_count':len(model.findall('.//collision')),'meshes':MESH_RECORDS,
            'runtime_validated':False,'feeding_validated':False,'cad_visual_match_finalized':False,
            'limitations':['Estimated masses and ideal velocity-driven actuator dynamics.',
              'Feeder pockets use convex prisms; pipe bores use circumscribed box panels.',
              'Hopper contact funnel is a four-plane approximation of the CAD round-to-rectangle loft.',
              'Deck collision holes are conservative rectangular openings; cosmetic fillets and shaft bore omitted.',
              'Retained 39 mm rigid wheel gap does not model rubber deformation.',
              'Self-collision disabled; CAD checks must establish assembly clearance.',
              'No launch impulse, ball replacement or expiry plugin.']}
    (MODEL/'generation_manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    # Remove only obsolete cells from this generator, never unrelated CAD files.
    used={Path(uri.text).name for uri in model.findall('.//mesh/uri')}
    for stale in (MODEL/'meshes').glob('*.stl'):
        if stale.name not in used and stale.read_bytes()[:40].startswith(b'R10 convex collision cell;'):
            stale.unlink()
    print(json.dumps({key:report[key] for key in ('ball_count','convex_collision_mesh_count','runtime_validated','feeding_validated')},indent=2))
    print(world_path)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--seed',type=int,default=42)
    generate(parser.parse_args().seed)
