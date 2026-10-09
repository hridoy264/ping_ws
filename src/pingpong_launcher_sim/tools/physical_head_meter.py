"""CAD-derived R10 meter visuals and explicit, documented contact approximations."""
import copy
import itertools
import json
import math


def add_head_meter(g, model, base, head):
    origins = {'head_link': g.PITCH, 'base_link': (0, 0, 0),
               'head_meter_roller_link': (.192, .025, .186),
               'head_meter_idler_carriage_link': (.192, -.025, .197),
               'head_meter_idler_link': (.192, -.025, .186),
               'head_meter_motor_link': (.166, .025, .2085),
               'drain_cover_link': (0, 0, 0), 'drain_service_panel_link': (0, 0, 0)}
    links = {'base_link': base, 'head_link': head}
    for name, mass, size in [
        ('head_meter_roller_link', .009, (.012, .012, .032)),
        ('head_meter_idler_carriage_link', .008, (.016, .010, .010)),
        ('head_meter_idler_link', .008, (.012, .012, .024)),
        ('head_meter_motor_link', .003, (.014, .014, .010)),
        ('drain_cover_link', .040, (.100, .040, .076)),
        ('drain_service_panel_link', .060, (.143, .005, .095))]:
        links[name] = g.link(model, name, origins[name], mass, size)
    g.joint(model, 'head_meter_roller_joint', 'head_link', 'head_meter_roller_link', (0, 0, 1), velocity=30, effort=.08)
    g.joint(model, 'head_meter_motor_joint', 'head_link', 'head_meter_motor_link', (0, 0, 1), velocity=30, effort=.08)
    g.joint(model, 'head_meter_idler_joint', 'head_meter_idler_carriage_link', 'head_meter_idler_link', (0, 0, 1), velocity=60, effort=.08)
    g.joint(model, 'head_meter_idler_slide', 'head_link', 'head_meter_idler_carriage_link', (0, -1, 0), (0, .002), .05, 2)
    slide = model.find("joint[@name='head_meter_idler_slide']")
    slide.set('type', 'prismatic')
    dynamics = g.node(slide.find('axis'), 'dynamics')
    # The hardware spring is unselected. Zero stiffness is explicit; never
    # manufacture a calibrated force from the nominal spring CAD envelope.
    g.node(dynamics, 'spring_reference', 0); g.node(dynamics, 'spring_stiffness', 0)
    g.node(dynamics, 'damping', .02)
    for name in ('drain_cover', 'drain_service_panel'):
        g.joint(model, name+'_installed_joint', 'base_link', name+'_link')
    for name in ('head_meter_roller_joint', 'head_meter_motor_joint'):
        g.controller(model, name, '/pingpong/head_meter_velocity_cmd')
    for name in ('head_meter_roller_link', 'head_meter_idler_link'):
        g.cylinder(links[name], name+'_tread', (0, 0, 0), .006, .006, '.08 .08 .09 1')
        # Exact CAD tire visual is supplied below; retain only primitive contact.
        for visual in list(links[name].findall('visual')): links[name].remove(visual)
    g.box(links['head_meter_idler_carriage_link'], 'idler_carriage', (-.008, -.005, -.003), (.008, .003, .003), visual=False)
    # Ball-accessible rails, stop, bearings and motor body; small holes, threads,
    # belt teeth and spring helix are visual-only, as recorded in the manifest.
    for label, lo, hi in [
        ('meter_right_rail', (.151,.029,.194), (.240,.043,.200)),
        ('meter_left_rail', (.151,-.043,.194), (.240,-.0322,.200)),
        ('meter_idler_stop', (.182,-.022,.194), (.202,-.019,.204)),
        ('meter_motor_body', (.156,.015,.215), (.176,.035,.245)),
        ('meter_motor_mount', (.156,.015,.212), (.176,.043,.215))]:
        g.box(head, label, g.sub(lo,g.PITCH), g.sub(hi,g.PITCH), visual=False)
    # Remove the old unbroken side wall and provide the CAD service opening.
    for tag in ('visual', 'collision'):
        for element in list(base.findall(tag)):
            if element.get('name') in ('case_left','case_left_visual'):
                base.remove(element)
    for k, lo, hi in [
        ('rear',(-.330,-.175,-.141),(-.040,-.172,-.004)),
        ('front',(.079,-.175,-.141),(.270,-.172,-.004)),
        ('bottom',(-.040,-.175,-.141),(.079,-.172,-.092)),
        ('top',(-.040,-.175,-.005),(.079,-.172,-.004))]:
        g.box(base,'service_wall_'+k,lo,hi)
    g.box(links['drain_service_panel_link'],'service_panel',(-.052,-.1783,-.099),(.091,-.1753,-.004),visual=False)
    split_drain_contacts(g, base, links['drain_cover_link'])
    # External mating flanges lie below/outside the 46mm bore.
    for target, y1, y2 in [(base,-.1208,-.1168),(links['drain_cover_link'],-.1248,-.1208)]:
        for label,lo,hi in [('bottom',(-.030,y1,-.082),(.069,y2,-.071)),('side',(.061,y1,-.082),(.069,y2,-.008))]:
            g.box(target,'drain_flange_'+label,lo,hi,visual=False)
    manifest = json.loads((g.MODEL/'meshes/cad_meter/manifest.json').read_text())
    if any(rec['part'].startswith('A02') for rec in manifest['records']):
        old=head.find("visual[@name='A02_Head_guide_and_50mm_hose_spigot']")
        if old is not None:head.remove(old)
    for record in manifest['records']:
        target = links[record['link']]
        visual = g.node(target,'visual',name=record['file'].removesuffix('.stl'))
        g.node(visual,'pose',(*g.mul(origins[record['link']],-1),0,0,0))
        shape = g.node(g.node(visual,'geometry'),'mesh')
        g.node(shape,'uri','model://pingpong_r10/meshes/cad_meter/'+record['file'])
        g.node(shape,'scale',(.001,.001,.001)); g.material(visual,g.GRAPHITE)
    return {'cad_source':manifest['source_archive'],'cad_visual_bodies':len(manifest['records']),
            'roller_centres_model_m':[[.192,.025,.186],[.192,-.025,.186]],
            'tire_radius_m':.006,'tire_face_z_m':[.183,.189],'unloaded_gap_m':.038,
            'idler_slide_axis':[0,-1,0],'idler_travel_m':[0,.002],
            'spring_stiffness_N_per_m':0,'spring_status':'Unselected; passive free slide with provisional damping, not qualified spring physics.',
            'motor_centre_model_m':[.166,.025,.2085],
            'drive':'Equal velocity commands at 1:1 ratio; no belt force transmission or brake simulation.',
            'command_topic':'/pingpong/head_meter_velocity_cmd','positive_velocity':'Driven tire contact moves ball toward +X.',
            'drain':'Separate installed fixed links for cover and service panel; no automatic opening or emptying.',
            'collision_scope':'Primitive tires/rails/motor, plane-split hollow pipe panels and flange boxes. Fasteners, belt and spring visual only.',
            'ball_sensors':'CAD optical modules retained as visuals; functional beam sensing pending.',
            'contact_status':'Rigid 12mm tires, friction inherited from model; compliance and spring force not calibrated.'}


def split_drain_contacts(g, base, cover):
    """Partition existing hollow panels at CAD split planes without filling bore."""
    def clip_faces(faces,axis,value,less):
        result=[];cap=[]
        def inside(p):return (p[axis]-value)*(1 if less else -1)<=1e-12
        for face in faces:
            out=[]
            for a,b in zip(face,face[1:]+face[:1]):
                ia,ib=inside(a),inside(b)
                if ia:out.append(a)
                if ia!=ib:
                    t=(value-a[axis])/(b[axis]-a[axis]);p=g.add(a,g.mul(g.sub(b,a),t));out.append(p);cap.append(p)
            if len(out)>=3:result.append(out)
        cap=list(dict.fromkeys(tuple(round(v,12) for v in p) for p in cap))
        if len(cap)>=3:
            axes=[i for i in range(3) if i!=axis];c=[sum(p[i] for p in cap)/len(cap) for i in axes]
            cap.sort(key=lambda p:math.atan2(p[axes[1]]-c[1],p[axes[0]]-c[0]));result.append(cap)
        return result
    for item in list(base.findall('collision')):
        if not item.get('name','').startswith(('pan_outlet_wall','lower_elbow_wall','vertical_riser_wall')):continue
        values=list(map(float,item.findtext('pose').split()));center=values[:3];r,p,y=values[3:]
        cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
        matrix=((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),(sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr))
        half=[v/2 for v in map(float,item.findtext('geometry/box/size').split())]
        points={s:tuple(center[i]+sum(matrix[i][j]*s[j]*half[j] for j in range(3)) for i in range(3)) for s in itertools.product((-1,1),repeat=3)}
        faces=[]
        for axis in range(3):
            other=[j for j in range(3) if j!=axis]
            for sign in (-1,1):
                face=[]
                for a,b in [(-1,-1),(1,-1),(1,1),(-1,1)]:
                    s=[0,0,0];s[axis]=sign;s[other[0]]=a;s[other[1]]=b;face.append(points[tuple(s)])
                faces.append(face)
        minus=clip_faces(faces,1,-.1208,True)
        cells=[(base,clip_faces(faces,1,-.1208,False)),(base,clip_faces(minus,2,-.008,False)),(cover,clip_faces(minus,2,-.008,True))]
        base.remove(item)
        for i,(target,fs) in enumerate(cells):
            vertices=list(dict.fromkeys(tuple(round(v,12) for v in p) for f in fs for p in f))
            if len(vertices)<4:continue
            triangles=g.convex_hull_triangles(vertices)
            volume=sum(g.dot(a,g.cross(b,c))/6 for a,b,c in triangles)
            if volume>1e-13:g.mesh(target,item.get('name')+'_split_'+str(i),triangles,g.GRAPHITE,visual=False)
    for item in list(base.findall('visual')):
        if item.get('name','').startswith(('pan_outlet_visual_mesh','lower_elbow_visual_mesh','vertical_riser_visual_mesh')):base.remove(item)
