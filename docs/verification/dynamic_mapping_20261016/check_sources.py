#!/usr/bin/env python3
"""Static ROS source checks only. Does not compile C++, launch ROS or touch hardware."""
import ast
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[3]
changed=subprocess.check_output(['git','diff','--name-only','-z'],cwd=ROOT,text=True).split('\0')
new=subprocess.check_output(['git','ls-files','--others','--exclude-standard','-z'],cwd=ROOT,text=True).split('\0')
manifest=Path(__file__).with_name('source_files.json')
recorded=json.loads(manifest.read_text()) if manifest.exists() else []
paths=sorted(set(name for name in changed+new+recorded if name))
if not manifest.exists():
    paths=sorted(set(paths+[manifest.relative_to(ROOT).as_posix()]))
    manifest.write_text(json.dumps(paths,ensure_ascii=False,indent=2)+'\n')
python=xml=cpp=0
for name in paths:
    p=ROOT/name
    if not p.is_file():continue
    if p.suffix=='.py':ast.parse(p.read_text(),filename=name);python+=1
    if p.suffix in ('.launch','.xml'):ET.parse(p);xml+=1
    if p.suffix in ('.cpp','.h'):
        # Lexical balance is deliberately NOT reported as a C++ syntax check.
        source=re.sub(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'', '',p.read_text())
        stack=[];opposites={')':'(',']':'[','}':'{'}
        for ch in source:
            if ch in '([{':stack.append(ch)
            elif ch in ')]}':
                assert stack and stack.pop()==opposites[ch],name
        assert not stack,name
        cpp+=1

src=ROOT/'patrol_uav_ws-patrol_planner/src'
entries=[ROOT/'deployment/board_trials_4x4/common/uav_board_trials/launch/mapping.launch',
         src/'uav_mission/launch/competition_mapping.launch']
for p in entries:
    tree=ET.parse(p).getroot()
    args={n.attrib['name']:n.attrib.get('default') for n in tree.findall('arg')}
    assert args['sensor_tf_frame']=='body'
    assert args['observation_enabled']=='false' # uncompiled candidate stays opt-in
    assert any(n.attrib.get('file','').endswith('/config/observation_map.yaml') for n in tree.findall('rosparam'))

env=src/'Fast-Planner/fast_planner/plan_env'
cmake=(env/'CMakeLists.txt').read_text()
for name in ('observation_worker.cpp','observation_integration.cpp'):
    assert 'src/'+name in cmake and (env/'src'/name).is_file()
assert 'test/test_observation_grid.cpp' in cmake
core=(env/'include/plan_env/observation_grid.h').read_text()
candidates=core.split('void candidates(',1)[1].split('bool observe(',1)[0]
assert '.hit=' not in candidates and '.free=' not in candidates
worker=(env/'src/observation_worker.cpp').read_text()
assert 'ros::Time(0)' not in worker and 'ros::Duration(.01)' in worker
assert 'max_input_points_' in worker and 'grid_.reset(epoch)' in worker
integration=(env/'src/observation_integration.cpp').read_text()
assert integration.index('manager.call(prepare)')<integration.index('freedom.call(reset)')
assert 'LANDED_STATE_ON_GROUND' in integration and '!observation_fc_.armed' in integration
assert 'observation_worker_->reset(observation_epoch_)' in integration
assert 'observation_resetting_ || !md_.has_odom_' in integration
assert 'pose_age>observation_worker_->maxAge()' in integration
shell=(src/'uav_mission/scripts/navigation_mission_manager.py').read_text()
assert 'observation_snapshot_stamp_mismatch' in shell
assert shell.index("rospy.set_param('/navigation_observation/rebuilding', False)")<shell.index('self._map_rebuild_pending = False',shell.index('def _on_observation'))

print(f'Python AST: {python} files; XML parse: {xml} files; C++ lexical balance: {cpp} files')
print('Static contracts: TF/body, opt-in defaults, entry wiring, build registration, candidate evidence separation, exact-time TF, reset guards/order/generation: PASS')
print('NOT performed: C++ compilation, C++ test execution, ROS launch expansion, simulation, board execution or performance measurements')
