"""Default selection and geometry-only promotion; no ROS or flight."""
import sys,json,subprocess,os,unittest
from pathlib import Path
import yaml
TOOL=Path(__file__).resolve().parents[1];ROOT=TOOL.parents[1]
sys.path[:0]=[str(TOOL),str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/src'),str(ROOT/'vision_ws/src/uav_high_view/src')]
from uav_mission.competition_config import validate
import wb_board

class Default10x10Tests(unittest.TestCase):
    def setUp(self):
        self.relative='deployment/competition/field_10x10_default.yaml'
        self.default=yaml.safe_load((ROOT/self.relative).read_text(encoding='utf-8'))
        self.template=yaml.safe_load((ROOT/'deployment/competition/field.example.yaml').read_text(encoding='utf-8'))
    def test_geometry_and_policy_authority(self):
        self.assertEqual(self.default['landing_xy'],[8.3,-3.5])
        self.assertEqual(self.default['corridor_waypoints'],[
            dict(x=6.9,y=3.4,agl=.9),dict(x=8.3,y=3.4,agl=.9),dict(x=8.3,y=1.9,agl=.9),
            dict(x=8.3,y=-.4,agl=.9),dict(x=8.3,y=-3.5,agl=.9)])
        geometry={'flight_bounds','search_center_bounds','target_bounds','coverage_bounds','staging_xy','survey_xy',
                  'corridor_waypoints','landing_xy','high_agl','landing_transit_agl','landing_capture_agl','map_size','corridor_speed_schedule'}
        for key in self.template.keys()-geometry:self.assertEqual(self.default[key],self.template[key],key)
        self.assertFalse(self.default['site_confirmed'])
        validate(self.default,flight=False)
        with self.assertRaises(ValueError):validate(self.default,flight=True)
    def test_workbench_defaults_and_offline_reference(self):
        cfg=wb_board.load_config();group=next(g for g in cfg['groups'] if g['channel']=='competition')
        self.assertEqual(group['site_config'],self.relative)
        self.assertIn(self.relative,wb_board.build_group_command(cfg,group,'preview')[0])
        self.assertEqual(yaml.safe_load((TOOL/'competition_templates'/self.relative).read_text(encoding='utf-8')),self.default)
    def test_cli_omission_selects_new_default_without_starting_ros(self):
        env=dict(os.environ,PYTHONUTF8='1',PYTHONPATH=os.pathsep.join(sys.path[:3]))
        command=[sys.executable,str(ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts/competition_supervisor.py'),
                 'preview','--root',str(ROOT),'--check-config']
        run=subprocess.run(command,cwd=TOOL,env=env,capture_output=True,text=True)
        self.assertEqual(run.returncode,0,run.stderr)
        result=json.loads(run.stdout)
        self.assertEqual(Path(result['site_config']),ROOT/self.relative)
        self.assertFalse(result['ros_started']);self.assertFalse(result['site_confirmed'])
        run=subprocess.run(command+['--site-config',str(ROOT/'deployment/competition/field.example.yaml')],env=env,capture_output=True,text=True)
        self.assertEqual(run.returncode,0,run.stderr)
        self.assertTrue(json.loads(run.stdout)['site_config'].endswith('field.example.yaml'))
if __name__=='__main__':unittest.main(verbosity=2)
