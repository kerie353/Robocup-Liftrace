#!/usr/bin/env python3
"""WSL-only SITL adapter. Existing hardware entry points remain unchanged."""
import argparse
import json
import os
import signal
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / 'deployment/board_trials_4x4/common/uav_board_trials/scripts'
sys.path[:0] = [str(SCRIPTS), str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'),
                str(ROOT/'vision_ws/src/uav_high_view/src')]
from trial_config import generate, load_trial_settings, apply_site_profile
from trial_speed_profiles import apply_speed_profile


def prepare(args, out):
    if args.trial == 'competition':
        from competition import prepare as prepare_competition
        return prepare_competition(args, ROOT, out)
    # Reuse the existing simulator's physical 4x4 fixture, then apply the
    # workbench's independently generated parameters without editing originals.
    import prepare_simulation as fixture
    if not hasattr(ET, 'indent'): ET.indent = lambda *a, **k: None
    saved_argv, saved_generate = sys.argv, fixture.generate
    try:
        fixture.generate = lambda root, directory, settings, xyz, rig: generate(
            root, directory, dict(settings, landing_alignment_mode='non_vh'), xyz, rig)
        sys.argv = ['prepare_simulation.py', 'full_mission', str(out)]
        fixture.main()
    finally:
        sys.argv, fixture.generate = saved_argv, saved_generate
    settings = load_trial_settings(ROOT, '08_full_mission')
    if args.site_config:
        apply_site_profile(settings, yaml.safe_load(Path(args.site_config).read_text()))
    if args.speed_profile:
        apply_speed_profile(ROOT, settings, 'full_mission', args.speed_profile)
    if args.flight_config:
        from uav_mission.flight_settings import apply_patch
        settings = apply_patch(settings, yaml.safe_load(Path(args.flight_config).read_text()), '08_full_mission')
    if args.motion_optimized: settings['motion_optimization'] = dict(settings.get('motion_optimization', {}), enabled=True)
    if args.motion_optimization: settings['motion_optimization'] = dict(settings.get('motion_optimization', {}), enabled=args.motion_optimization=='on')
    if args.obstacle_columns: settings['obstacle_columns_enabled'] = args.obstacle_columns=='on'
    if args.survey_pattern: settings['survey_pattern'] = args.survey_pattern
    if args.resume_survey: settings['resume_survey_enabled'] = args.resume_survey == 'on'
    settings['camera_info_topic'] = '/downward_camera/camera_info'
    settings['compressed_image_topic'] = '/downward_camera/image_raw/compressed'
    settings['actuator_mode'] = 'mock'
    # POSCTL requires a physical pilot. Preserve OFFBOARD candidate testing;
    # replace only a requested POSCTL handoff with simulator AUTO.LAND.
    requested_handoff = settings.get('landing_handoff_mode', 'POSCTL')
    requested_alignment = settings.get('landing_alignment_mode', 'non_vh')
    if requested_handoff == 'POSCTL': settings['landing_handoff_mode'] = 'AUTO.LAND'
    if settings['landing_handoff_mode'] == 'AUTO.LAND': settings['landing_alignment_mode'] = 'non_vh'
    rig = yaml.safe_load((SCRIPTS.parent/'config/known_rig.yaml').read_text())
    ref = generate(ROOT, out, settings, (0., 0., 0.), rig)
    overrides = yaml.safe_load((out/'overrides.yaml').read_text())
    overrides['/target_detector/device'] = 'cpu'
    (out/'overrides.yaml').write_text(yaml.safe_dump(overrides, sort_keys=False))
    (out/'settings.yaml').write_text(yaml.safe_dump(settings, sort_keys=False))
    # Reposition the H marker to the requested navigation goal. The fixture's
    # walls/gates/targets stay fixed and are disclosed in metadata.
    scene = ET.parse(out/'scene.world')
    for inc in scene.getroot().find('world').findall('include'):
        if inc.findtext('name') == 'landing_h_clone':
            hx, hy = settings['landing_xy']
            inc.find('pose').text = '{} {} 0 0 0 0'.format(hx, hy)
    scene.write(out/'scene.world', encoding='utf-8', xml_declaration=True)
    summary = dict(status='CONFIG_VALID', source='generated_runtime', simulation=True,
        trial='full_mission', settings=settings, runtime_path=str(out/'runtime.yaml'),
        mode=settings['mode'], speed_profile=settings.get('speed_profile','limited'),
        motion_optimization=bool(settings.get('motion_optimization',{}).get('enabled',False)),
        resume_survey=bool(settings.get('resume_survey_enabled',False)),
        obstacle_columns=bool(settings.get('obstacle_columns_enabled',True)),
        following=yaml.safe_load((out/'runtime.yaml').read_text()).get('following_speed_profile'),
        requested_handoff=requested_handoff, effective_handoff=settings['landing_handoff_mode'],
        requested_alignment=requested_alignment, effective_alignment=settings['landing_alignment_mode'],
        fixture='fixed 4x4; editing waypoints does not rebuild walls/gates/targets',
        vision_model_ready=Path(os.environ['UAV_VISION_MODEL_PATH']).is_file())
    (out.parent/'run_metadata.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print('CONFIG_VALID; no ROS nodes started', flush=True)
    print(json.dumps(summary, ensure_ascii=False), flush=True)
    return ref


def build_launch(args, out):
    launch = ET.parse(SCRIPTS.parent/'launch/simulation.launch')
    if args.trial == 'competition':
        # Use the simulated sensors/mock wiring around the exact real manager.
        application=ET.parse(SCRIPTS.parent/'launch/application.launch')
        manager=application.getroot().find("group/group/node[@name='mission_manager']")
        if manager is None: raise RuntimeError('Simulation application manager layout changed')
        manager.set('pkg','uav_mission')
        manager.set('type',str(ROOT/'deployment/workbench_sim/competition_manager.py'))
        app_path=out/'competition_application.launch'
        application.write(app_path,encoding='utf-8',xml_declaration=True)
        gate=launch.getroot().find("node[@name='trial_sim_gate']")
        gate.set('type',str(ROOT/'deployment/workbench_sim/competition_gate.py'))
        ignored=launch.getroot().find("node[@name='gazebo_contact_monitor']/rosparam")
        patterns=yaml.safe_load(ignored.text)
        patterns.extend(['target_pillbox::link::collision','target_tent::link::collision'])
        ignored.text=yaml.safe_dump(patterns)
    # Preserve the canonical simulator and flight-control/mission code.
    for inc in launch.getroot().findall('include'):
        if inc.get('file') == '$(find uav_board_trials)/launch/application.launch':
            ET.SubElement(inc, 'arg', name='record_camera_video', value='false')
            if args.trial == 'competition': inc.set('file',str(app_path))
    for node in list(launch.getroot().findall('node')):
        if node.get('name') in ('spawn_overview_camera', 'overview_video_recorder'):
            launch.getroot().remove(node)
    path = out/'workbench.launch'
    launch.write(path, encoding='utf-8', xml_declaration=True)
    return path


def launch(args, out):
    def terminate(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    ref = prepare(args, out)
    path = build_launch(args, out)
    print('INITIALIZING '+json.dumps(dict(run_dir=str(out.parent), simulation=True)), flush=True)
    from trial_bag import TrialBag
    settings = yaml.safe_load((out/'settings.yaml').read_text())
    bag = TrialBag(out.parent, settings, dict(os.environ))
    bag.start()
    try:
        command = ['roslaunch', str(path), 'generated_dir:='+str(out),
                   'trial:=full_mission', 'mode:=high_view_full',
                   'model_path:='+os.environ['UAV_VISION_MODEL_PATH'],
                   'ground_z:='+str(ref['ground_z']), 'low_z:='+str(ref['low_z'])]
        return subprocess.call(command)
    except KeyboardInterrupt:
        return 130
    finally:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        bag.close()
        print('Trial application stopped', flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('trial', choices=['full_mission','competition'])
    p.add_argument('mode', choices=['preview','flight'])
    p.add_argument('--site-config')
    p.add_argument('--check-config', action='store_true')
    p.add_argument('--flight-config')
    p.add_argument('--speed-profile', choices=['limited','competition'])
    p.add_argument('--motion-optimized', action='store_true')
    p.add_argument('--motion-optimization', choices=['on','off'])
    p.add_argument('--obstacle-columns', choices=['on','off'])
    p.add_argument('--survey-pattern', choices=['rectangle','snake2','snake3'])
    p.add_argument('--resume-survey', choices=['on','off'])
    p.add_argument('--scene-seed',type=int,default=11)
    p.add_argument('--inside-wrapper', action='store_true')
    args = p.parse_args()
    if not 0<=args.scene_seed<=2147483647: p.error('scene seed must be 0..2147483647')
    if args.trial=='competition' and (args.speed_profile or args.survey_pattern or args.motion_optimized):
        p.error('competition uses its field YAML and on/off switches')
    if args.inside_wrapper:
        if not os.environ.get('SIM_RUN_DIR'): p.error('sim_run.sh required')
        return launch(args, Path(os.environ['SIM_RUN_DIR'])/'generated')
    if args.check_config or args.mode == 'preview':
        base = ROOT/'logs/flight_workbench'
        base.mkdir(parents=True, exist_ok=True)
        out = Path(tempfile.mkdtemp(prefix='sim_config_', dir=str(base)))/'generated'
        prepare(args, out)
        return 0
    if not Path(os.environ['UAV_VISION_MODEL_PATH']).is_file(): p.error('Missing .pt model: '+os.environ['UAV_VISION_MODEL_PATH'])
    env = dict(os.environ, SIM_RUN_AUTHORIZED='1')
    process = subprocess.Popen(['bash', str(ROOT/'top_level_scripts/sim_run.sh'),
        'board_competition_wsl' if args.trial=='competition' else 'board_full_mission_wsl', '/usr/bin/python3', str(Path(__file__).resolve()),
        *sys.argv[1:], '--inside-wrapper'], env=env, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, universal_newlines=True, bufsize=1)
    readers = []
    def forward_markers(directory):
        path = Path(directory)/'run.log'
        while process.poll() is None and not path.exists():
            time.sleep(.2)
        if not path.exists(): return
        ready, checked_at = False, 0.
        with path.open() as stream:
            while True:
                if not ready and time.monotonic()-checked_at>2:
                    checked_at=time.monotonic()
                    try:
                        progress=json.loads((Path(directory)/'generated/progress.json').read_text())
                        if progress.get('started'):
                            print('READY: '+directory, flush=True)
                            ready=True
                    except (OSError, ValueError): pass
                line = stream.readline()
                if not line:
                    if process.poll() is not None: return
                    time.sleep(.2)
                    continue
                if line.startswith(('CONFIG_VALID', '{"status": "CONFIG_VALID"',
                                    'INITIALIZING', 'BAG_CLOSED:', 'Trial application stopped')):
                    print(line.rstrip(), flush=True)
    try:
        for line in process.stdout:
            print(line.rstrip(), flush=True)
            if line.startswith('Run dir:') and not readers:
                directory = line.partition(':')[2].strip()
                print('INITIALIZING '+json.dumps(dict(run_dir=directory, simulation=True)), flush=True)
                reader = threading.Thread(target=forward_markers, args=(directory,), daemon=True)
                reader.start()
                readers.append(reader)
        code = process.wait()
    except KeyboardInterrupt:
        process.terminate()
        try: process.wait(timeout=50)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        code = 130
    for reader in readers: reader.join(timeout=2)
    return code


if __name__ == '__main__':
    sys.exit(main())
