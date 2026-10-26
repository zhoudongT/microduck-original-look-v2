"""Official-outline Microduck revision with full-size UBT packaging candidates.
Appearance studies derived from Pollen Robotics meshes: CC BY-NC-SA 4.0.
This is NOT a completed mechanical adapter or a validated walking robot.
"""
from pathlib import Path
import sys, json, csv, shutil
R=Path(__file__).resolve().parent
import numpy as np
import trimesh

SCALE=2.0
OFF=R/'source/official'
BASE=R/'source/cartridge-v1'
official=json.loads((OFF/'reference-stand-scene.json').read_text(encoding='utf-8'))
frames=json.loads((OFF/'motor-frame-report.json').read_text(encoding='utf-8'))
v1=json.loads((BASE/'scene.json').read_text(encoding='utf-8'))
v1j=json.loads((BASE/'joints.json').read_text(encoding='utf-8'))
slim=json.loads((R/'slim-cartridge-report.json').read_text(encoding='utf-8'))

EXTERIOR={'top_head_shell','bottom_head_shell','face_part','noenoeil','lens',
          'jaw','jaw_soft','soft_mouth_top','left_shell','right_shell',
          'upper_leg_left','upper_leg_right','foot_left','foot_right',
          'sole_left','sole_right','ankle_left','ankle_right'}
NONPRINT={'lens'}
ELECTRONICS={'np_f970','speaker','banana_pcb_locker','elec_rpi_robot_hat_pcb',
             'pcb__raspberry_pi_zero_2_w','m12_lens_holder'}
COLOR={k:'#e5e2d9' for k in ['top_head_shell','left_shell','right_shell','upper_leg_left','upper_leg_right']}
COLOR.update({k:'#ee6e21' for k in ['jaw','bottom_head_shell','foot_left','foot_right','ankle_left','ankle_right']})
COLOR.update({k:'#f2b42b' for k in ['noenoeil','sole_left','sole_right']})
COLOR.update(face_part='#777a77',lens='#161b1e',jaw_soft='#c4b3a0',soft_mouth_top='#c4b3a0')
ROT=np.eye(4);ROT[:3,:3]=[[0,1,0],[-1,0,0],[0,0,1]] # reference +X front -> viewer -Y
all_bounds=[]
for i in official['instances']:
    if i['geometry'] not in EXTERIOR:continue
    raw=official['geometries'][i['geometry']]
    verts=np.array(raw['v']).reshape(-1,3)
    world=trimesh.transform_points(verts,np.array(i['matrix']).reshape((4,4),order='F'))
    all_bounds.append(world)
FLOOR=float(np.vstack(all_bounds)[:,2].min()*SCALE)
ROT[2,3]=-FLOOR

GEOM={};INST=[];FULL={};STATUS={};ASSEMBLY=[]
def matrix(values):return np.array(values,dtype=float).reshape((4,4),order='F')
def load_mesh(name):
    mesh=trimesh.load_mesh(OFF/'assets'/f'{name}.stl',process=True)
    mesh.apply_scale(1000*SCALE)
    return mesh
def register(name,mesh,status,face_target=1700):
    FULL[name]=mesh.copy();STATUS[name]=status
    shown=mesh.simplify_quadric_decimation(face_count=face_target) if len(mesh.faces)>face_target else mesh.copy()
    GEOM[name]={'v':np.round(shown.vertices,3).reshape(-1).tolist(),'i':shown.faces.reshape(-1).tolist()}
def add(name,geom,M,color,category,group,explode,status):
    INST.append(dict(name=name,geometry=geom,matrix=np.round(M.reshape(-1,order='F'),8).tolist(),
                     color=color,category=category,group=group,explode=np.round(explode,3).tolist(),status=status))
    world=FULL[geom].copy();world.apply_transform(M)
    ASSEMBLY.append((name,geom,category,world))

def explode_official(name,group):
    if name=='top_head_shell':return [0,12,125]
    if name in ['face_part','noenoeil','lens']:return [0,-(95 if name=='lens' else 55),85]
    if name=='bottom_head_shell':return [0,0,45]
    if name in ['jaw','jaw_soft','soft_mouth_top']:return [0,-70,20]
    if name in ['left_shell','right_shell']:return [75 if name=='left_shell' else -75,0,15]
    side=1 if group=='left-leg' else -1
    if 'sole' in name:return [side*35,0,-75]
    if 'foot' in name:return [side*35,0,-45]
    if 'upper_leg' in name:return [side*75,0,0]
    if 'ankle' in name:return [side*35,0,-20]
    return [0,0,20]

for item in official['instances']:
    name=item['geometry']
    if name=='xl330' or name in ELECTRONICS or name.startswith('seeed_bearing'):continue
    key='OFF_'+name
    if key not in FULL:
        mesh=load_mesh(name)
        exterior=name in EXTERIOR
        status='Official outer geometry uniformly scaled; holes/walls not redesigned' if exterior else 'Original internal link, scaled reference only; attachment to UBT not designed'
        register(key,mesh,status,17000 if name=='top_head_shell' else 2300 if exterior else 900)
        if exterior and name not in NONPRINT:
            mesh.export(R/'cad/exterior-study-stl'/f'{name}_2x_REFERENCE.stl')
    M=matrix(item['matrix']);M[:3,3]*=SCALE;M=ROT@M
    group=item['group'];category='shell' if name in EXTERIOR else 'reference'
    add('外形_'+name if category=='shell' else '待改接口_'+item['name'],key,M,COLOR.get(name,'#7d8790'),category,group,
        explode_official(name,group),STATUS[key])

# Full-size new servo/cartridge pieces: unlike the exterior they are NOT scaled.
old_anchor=np.array(v1j['J_L_ANKLE']['matrix'])
old_inverse=np.linalg.inv(old_anchor)
template=[i for i in v1['instances'] if i['name'].startswith('J_L_ANKLE')]
for i in template:
    key=i['geometry']
    if key in FULL:continue
    if key in ['P01_cage_back','P02_cage_front']:
        # The slim-cartridge builder exports in the original v1 cartridge frame.
        mesh=trimesh.load_mesh(R/'cad/adapter-stl'/f'{key}.stl',process=True)
    elif key.startswith('P'):
        mesh=trimesh.load_mesh(R/'cad/adapter-stl'/f'{key}.stl',process=True)
    else:
        raw=v1['geometries'][key]
        mesh=trimesh.Trimesh(vertices=np.array(raw['v']).reshape(-1,3),faces=np.array(raw['i']).reshape(-1,3),process=True)
    register(key,mesh,'UBT candidate cartridge; shaft/horn dimensions still assumed',1700)

motor_map={m['instance']:m for m in frames['motors']}
clamp_positions=slim['clamp_holes_v1_mm']
new_motor_frames=[]
for item in official['instances']:
    if item['geometry']!='xl330':continue
    info=motor_map[item['name']]
    joint=info['matches'][0]['joint'] if info['matches'] else 'mouth'
    prefix='J_'+joint.upper().replace('LEFT_','L_').replace('RIGHT_','R_')
    M=matrix(item['matrix']);M[:3,3]*=SCALE
    C=matrix(info['v1_module_to_xl330_local_candidates']['-1'])
    B=ROT@M@C
    # Preserve the external mesh. Move only the packaging candidate inside it.
    # These offsets invalidate direct reuse of the original joint interfaces.
    offset=np.zeros(3)
    if joint.endswith('hip_yaw'):offset=np.array([0,1.4,0])
    if joint.endswith(('knee','hip_pitch')):offset=np.array([0,5.2,0])
    if joint.endswith('ankle'):offset=np.array([0,0,-6.5])
    if joint=='head_roll':offset=np.array([0,-1.0,0])
    if joint=='mouth':offset=np.array([1,-1,0])
    B[:3,3]+=B[:3,:3]@offset
    new_motor_frames.append(dict(joint=joint,source=item['name'],matrix=B.tolist(),assumed_drive_side='XL negative X',packaging_offset_v1_local_mm=offset.tolist(),original_joint_interface_preserved=False))
    g=item['group'];spread=np.array([65 if g=='left-leg' else -65 if g=='right-leg' else 0,0,50 if g=='head' else 0])
    for old in template:
        suffix=old['name'][len('J_L_ANKLE'):]
        L=old_inverse@matrix(old['matrix'])
        if suffix.startswith(('_clamp_M3_','_clamp_nut_')):
            idx=int(suffix.rsplit('_',1)[1]);pos=clamp_positions[idx]
            L[0,3]=pos[0];L[2,3]=pos[2]
        color='#282c30' if old['geometry'] in ['P01_cage_back','P02_cage_front','REF_UBT12HC_envelope'] else '#a2abb1' if old['category']=='hardware' else '#b2bac0'
        exp=spread+B[:3,:3]@(old_inverse[:3,:3]@np.array(old['explode']))*.7
        # Cage sides expand along their own bolt axis, useful for maintenance inspection.
        if suffix=='_cage_front':exp=spread+B[:3,1]*32
        if suffix=='_cage_back':exp=spread-B[:3,1]*32
        add(prefix+suffix,old['geometry'],B@L,color,old['category'],g,exp,'UBT layout candidate, physical fit and attachment not validated')

# Full-resolution external surface check: compare new cartridge against preserved outer parts.
# Inner links remain references. Report their collisions separately, never hide them by cutting shells.
check_rows=[];errors=[]
candidates=[(n,k,c,m) for n,k,c,m in ASSEMBLY if n.startswith('J_') and c in ['servo','structure']]
outer=[(n,k,c,m) for n,k,c,m in ASSEMBLY if c=='shell']
def overlaps(a,b):return bool(np.all(a.bounds[0]<b.bounds[1]-1e-5) and np.all(b.bounds[0]<a.bounds[1]-1e-5))
for n,k,c,m in candidates:
    for on,ok,oc,om in outer:
        if not overlaps(m,om):continue
        try:
            overlap=trimesh.boolean.intersection([m,om],engine='manifold',check_volume=False)
            v=abs(float(overlap.volume)) if overlap is not None and len(overlap.faces) else 0
            if v>.5:check_rows.append(dict(candidate=n,outer=on,intersection_mm3=round(v,3)))
        except Exception as exc:errors.append(dict(candidate=n,outer=on,error=str(exc)))

prefixes=['J_'+j['joint'].upper().replace('LEFT_','L_').replace('RIGHT_','R_') for j in new_motor_frames]
owners={n:next(p for p in prefixes if n.startswith(p+'_')) for n,k,c,m in candidates}
between=[]
for index,(n,k,c,m) in enumerate(candidates):
    for on,ok,oc,om in candidates[index+1:]:
        if owners[n]==owners[on] or not overlaps(m,om):continue
        try:
            overlap=trimesh.boolean.intersection([m,om],engine='manifold',check_volume=False)
            v=abs(float(overlap.volume)) if overlap is not None and len(overlap.faces) else 0
            if v>.5:between.append(dict(part_a=n,part_b=on,intersection_mm3=round(v,3)))
        except Exception as exc:errors.append(dict(part_a=n,part_b=on,error=str(exc)))

points=np.vstack([m.vertices for n,k,c,m in ASSEMBLY if c=='shell'])
meta=dict(title='Microduck original outline · UBT packaging v2',revision='v2 exterior and packaging study',
          units='mm',exterior_scale=SCALE,exterior_extents_mm=np.round(np.ptp(points,axis=0),3).tolist(),
          servo_count=15,reference_servo_count=0,price_ceiling_cny=50,price_verified=False,
          source=official['meta']['source'],source_commit=official['meta']['source_commit'],
          original_geometry_author='Pollen Robotics',mesh_license='CC BY-NC-SA 4.0',
          unverified=['all mounting interfaces','real UBT shaft/horn/plug','full motion','torque/mass','budget','walking'],
          external_collision_count=len(check_rows),intermodule_collision_count=len(between),collision_errors=len(errors))
(R/'scene.json').write_text(json.dumps(dict(geometries=GEOM,instances=INST,meta=meta),separators=(',',':')),encoding='utf-8')
(R/'motor-layout.json').write_text(json.dumps(new_motor_frames,indent=2),encoding='utf-8')
(R/'packaging-report.json').write_text(json.dumps(dict(meta=meta,scope='Static cartridge vs exterior and cartridge vs other cartridges; threshold 0.5 mm3. Original internal links, hardware, same-cartridge contacts, motion and rigid attachment not validated.',collisions=check_rows,intermodule_collisions=between,errors=errors,part_status=STATUS),indent=2),encoding='utf-8')
with (R/'parts-status.csv').open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['geometry','quantity','status'])
    for k in FULL:w.writerow([k,sum(i['geometry']==k for i in INST),STATUS[k]])
visual=trimesh.Scene()
for i in INST:
    raw=GEOM[i['geometry']]
    mesh=trimesh.Trimesh(vertices=np.array(raw['v']).reshape(-1,3),faces=np.array(raw['i']).reshape(-1,3),process=False)
    mesh.visual.face_colors=trimesh.visual.color.hex_to_rgba(i['color'])
    visual.add_geometry(mesh,node_name=i['name'],transform=matrix(i['matrix']))
(R/'cad/microduck-v2-layout.glb').write_bytes(visual.export(file_type='glb'))
print(json.dumps(dict(meta=meta,instances=len(INST),geometries=len(GEOM),collisions=check_rows[:12],errors=errors[:3]),ensure_ascii=False))
