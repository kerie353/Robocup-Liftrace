"""Offline competition configuration and physical SITL scene generation."""
import copy
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET
import yaml
from uav_mission import competition_config


def build_scene(root, out, settings, seed):
    source = ET.parse(root/'vision_ws/src/uav_vision_eval/models/r2026_horizontal_field/field.world').getroot().find('world')
    sdf = ET.Element('sdf', version='1.6')
    world = ET.SubElement(sdf, 'world', name='default')
    for el in source:
        if el.tag not in ('model','include','state','gui'): world.append(copy.deepcopy(el))
    def include(uri, name, x, y, yaw=0.):
        el=ET.SubElement(world,'include')
        ET.SubElement(el,'uri').text='model://'+uri
        ET.SubElement(el,'name').text=name
        ET.SubElement(el,'pose').text='{} {} 0 0 0 {}'.format(x,y,yaw)
    include('sun','sun',0,0)
    include('ground_plane','ground_plane',0,0)
    include('landing_h','landing_h',0,0)
    include('landing_h','landing_h_clone',*settings['landing_xy'])
    obstacles=[]
    def box(name, x, y, sx, sy):
        if sx<=1e-6 or sy<=1e-6: return
        model=ET.SubElement(world,'model',name=name)
        ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text='{} {} .75 0 0 0'.format(x,y)
        link=ET.SubElement(model,'link',name='link')
        for kind in ('collision','visual'):
            el=ET.SubElement(link,kind,name=kind)
            ET.SubElement(ET.SubElement(ET.SubElement(el,'geometry'),'box'),'size').text='{} {} 1.5'.format(sx,sy)
            if kind=='visual':
                material=ET.SubElement(el,'material')
                ET.SubElement(material,'ambient').text='.72 .76 .78 1'
                ET.SubElement(material,'diffuse').text='.72 .76 .78 1'
        obstacles.append(dict(name=name,xy=[x,y],size=[sx,sy,1.5]))
    xmin,xmax,ymin,ymax=settings['flight_bounds']
    box('field_west',xmin-.1,(ymin+ymax)/2,.2,ymax-ymin+.4)
    box('field_east',xmax+.1,(ymin+ymax)/2,.2,ymax-ymin+.4)
    box('field_south',(xmin+xmax)/2,ymin-.1,xmax-xmin,.2)
    box('field_north',(xmin+xmax)/2,ymax+.1,xmax-xmin,.2)
    trees=[]
    # Rotate the canonical field into +X-forward/+Y-left mission coordinates.
    search=settings['search_center_bounds']
    for tree in source.find("model[@name='toudi2']").findall('model'):
        el=copy.deepcopy(tree)
        p=list(map(float,el.findtext('pose').split()))
        x,y=p[1],-p[0]
        if not search[0]+.6<=x<=search[1]-.6 or not search[2]+.6<=y<=search[3]-.6: continue
        el.find('pose').text='{} {} {} {} {} {}'.format(x,y,*p[2:5],p[5]-math.pi/2)
        world.append(el)
        trees.append([x,y])
    context=copy.deepcopy(source.find("model[@name='indoor_context']"))
    if context is not None:
        ET.SubElement(context,'pose').text='0 0 0 0 0 {}'.format(-math.pi/2)
        world.append(context)
    route=[[p['x'],p['y']] for p in settings['corridor_waypoints']]
    schedule=settings.get('corridor_speed_schedule') or {}
    axis=schedule.get('axis',1)
    other=1-axis
    bounds=[(xmin,xmax),(ymin,ymax)]
    # Gate locations are a disclosed scene assumption derived from measured
    # route/wall planes, never a source of navigation or recognition truth.
    side=1 if route[-1][other] >= (search[2*other]+search[2*other+1])/2 else -1
    edge=search[2*other+(1 if side>0 else 0)]
    divider=(edge+route[-1][other])/2
    width=.8
    def segment(name,plane,fixed_axis,lo,hi):
        center=[0.,0.];size=[0.,0.]
        center[fixed_axis]=plane;center[1-fixed_axis]=(lo+hi)/2
        size[fixed_axis]=.2;size[1-fixed_axis]=hi-lo
        box(name,*center,*size)
    def crossing(plane,a):
        for p,q in zip(route,route[1:]):
            if abs(q[a]-p[a])>1e-9 and min(p[a],q[a])<=plane<=max(p[a],q[a]):
                t=(plane-p[a])/(q[a]-p[a])
                return p[1-a]+t*(q[1-a]-p[1-a])
        return None
    entry=crossing(divider,other)
    if entry is None: entry=route[0][axis]
    if not bounds[axis][0]+width/2<=entry<=bounds[axis][1]-width/2:
        raise ValueError('Entrance opening does not fit the scene bounds')
    segment('search_divider_a',divider,other,bounds[axis][0],entry-width/2)
    segment('search_divider_b',divider,other,entry+width/2,bounds[axis][1])
    gates=[]
    corridor_lo,corridor_hi=sorted([divider,bounds[other][1 if side>0 else 0]])
    for i,plane in enumerate(schedule.get('wall_coordinates',[])):
        center=crossing(plane,axis)
        if center is None: raise ValueError('A corridor wall plane is not crossed by the configured route')
        if not corridor_lo+width/2<=center<=corridor_hi-width/2:
            raise ValueError('A corridor opening does not fit the scene')
        segment('gate_{}_a'.format(i),plane,axis,corridor_lo,center-width/2)
        segment('gate_{}_b'.format(i),plane,axis,center+width/2,corridor_hi)
        xy=[0.,0.];xy[axis]=plane;xy[other]=center
        gates.append(dict(xy=xy,width=width))
    rng=random.Random(seed)
    target_bounds=settings['target_bounds']
    targets=[]
    for kind,model in [('bridge','qiaoliang'),('panzer','zhuangjiache'),('pillbox','dibao'),('tent','zhangpeng'),('red_cross','red_cross')]:
        for attempt in range(2000):
            x=rng.uniform(target_bounds[0]+.55,target_bounds[1]-.55)
            y=rng.uniform(target_bounds[2]+.55,target_bounds[3]-.55)
            if math.hypot(x,y)<1.0: continue
            if any(math.hypot(x-tx,y-ty)<1.15 for tx,ty in trees): continue
            if any(math.hypot(x-t['x'],y-t['y'])<1.1 for t in targets): continue
            break
        else: raise ValueError('Scene cannot fit five separated targets inside target_bounds')
        yaw=rng.uniform(-math.pi,math.pi)
        include(model,'target_'+kind,x,y,yaw)
        targets.append(dict(class_name=kind,x=x,y=y,yaw=yaw))
    ET.ElementTree(sdf).write(out/'scene.world',encoding='utf-8',xml_declaration=True)
    scene=dict(scope='physical_scene_and_evaluation_only',seed=seed,bounds=settings['flight_bounds'],
        targets=targets,trees=trees,gates=gates,obstacles=obstacles,landing_xy=settings['landing_xy'],
        assumptions=['trees and indoor context inherited from canonical world',
                     'divider and 0.80m gate openings inferred from route/wall planes'])
    (out/'scene.json').write_text(json.dumps(scene,indent=2))
    return scene


def prepare(args, root, out):
    out.mkdir(parents=True,exist_ok=True)
    source=Path(args.site_config) if args.site_config else root/'deployment/competition/field_10x10_default.yaml'
    requested=yaml.safe_load(source.read_text())
    settings=competition_config.apply_overrides(requested,args.motion_optimization,args.obstacle_columns,resume=args.resume_survey)
    if args.flight_config:
        from uav_mission.flight_settings import apply_patch
        settings=apply_patch(settings,yaml.safe_load(Path(args.flight_config).read_text()),'competition')
    competition_config.validate(settings,flight=False)
    handoff=settings.get('landing_handoff_mode','POSCTL')
    alignment=settings.get('landing_alignment_mode','non_vh')
    settings=copy.deepcopy(settings)
    settings['site_confirmed']=True  # Generated simulator copy only, not field approval.
    if handoff=='POSCTL': settings['landing_handoff_mode']='AUTO.LAND'
    if settings['landing_handoff_mode']=='AUTO.LAND': settings['landing_alignment_mode']='non_vh'
    settings.update(image_topic='/downward_camera/image_raw',camera_info_topic='/downward_camera/camera_info',
                    compressed_image_topic='/downward_camera/image_raw/compressed')
    rig=yaml.safe_load((root/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml').read_text())
    # The real generator validates its real-only contract. Switch the generated
    # simulator service wiring afterwards; never weaken the hardware validator.
    ref=competition_config.generate(root,out,settings,(0.,0.,0.),rig)
    overrides=yaml.safe_load((out/'overrides.yaml').read_text())
    overrides.update({'/target_detector/device':'cpu','/board_trials/actuator_mode':'mock',
        '/board_trials/mock_only':True,'/board_trials/ground_z':ref['ground_z'],
        '/board_trials/fc_on_ground_z':0.})
    (out/'overrides.yaml').write_text(yaml.safe_dump(overrides,sort_keys=False))
    settings['actuator_mode']='mock'
    settings['metadata_topic']='/competition/run_metadata'
    (out/'settings.yaml').write_text(yaml.safe_dump(settings,sort_keys=False))
    ref['settings']=settings
    (out/'ground_reference.json').write_text(json.dumps(ref,indent=2))
    scene=build_scene(root,out,settings,args.scene_seed)
    summary=dict(status='CONFIG_VALID',simulation=True,trial='competition',
        site_config=str(source),site_confirmed=requested.get('site_confirmed',False),generation_ready=True,
        actuator_mode='mock',requested_actuator_mode=requested.get('actuator_mode'),
        requested_handoff=handoff,effective_handoff=settings['landing_handoff_mode'],
        requested_alignment=alignment,effective_alignment=settings['landing_alignment_mode'],
        settings=settings,scene_seed=args.scene_seed,physical_scene=str(out/'scene.world'),
        target_count=len(scene['targets']),tree_count=len(scene['trees']),gate_count=len(scene['gates']),
        scene_assumptions=scene['assumptions'],**ref['effective_config'])
    (out.parent/'run_metadata.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2))
    print('CONFIG_VALID; no ROS nodes started',flush=True)
    print(json.dumps(summary,ensure_ascii=False),flush=True)
    return ref
