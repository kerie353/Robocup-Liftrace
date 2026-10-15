"""Offline integration checks; do not start ROS or Gazebo."""
import argparse
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import yaml
import session


class CompetitionPreparation(unittest.TestCase):
    def args(self):
        return argparse.Namespace(trial='competition',site_config=str(session.ROOT/'deployment/competition/field_10x10_default.yaml'),
            flight_config=None,motion_optimization=None,obstacle_columns=None,resume_survey=None,scene_seed=11)

    def test_real_config_preserved_and_only_simulator_wiring_changes(self):
        args=self.args()
        source=Path(args.site_config)
        before=source.read_bytes()
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'generated'
            ref=session.prepare(args,out)
            original=yaml.safe_load(before)
            control=yaml.safe_load((out/'control.yaml').read_text())
            runtime=yaml.safe_load((out/'runtime.yaml').read_text())
            overrides=yaml.safe_load((out/'overrides.yaml').read_text())
            summary=json.loads((out.parent/'run_metadata.json').read_text())
            self.assertEqual(source.read_bytes(),before)
            self.assertEqual(ref['settings']['cruise_speed'],original['cruise_speed'])
            self.assertEqual(control['drop_system']['release_setpoint_height'],ref['ground_z']+original['drop_agl'])
            self.assertEqual(runtime['mission']['landing_xy'],original['landing_xy'])
            self.assertEqual(runtime['high_view_probe']['config']['survey_xy'],original['survey_xy'])
            self.assertEqual(overrides['/board_trials/actuator_mode'],'mock')
            self.assertFalse(summary['site_confirmed'])
            self.assertEqual(summary['effective_handoff'],'AUTO.LAND')
            launch=session.build_launch(args,out)
            app=ET.parse(out/'competition_application.launch')
            manager=app.getroot().find("group/group/node[@name='mission_manager']")
            self.assertTrue(manager.get('type').endswith('competition_manager.py'))
            self.assertNotIn('navigation_competition_manager.py',launch.read_text())

    def test_scene_reproducible_and_truth_kept_out_of_runtime(self):
        args=self.args()
        with tempfile.TemporaryDirectory() as temp:
            a,b=Path(temp)/'a/generated',Path(temp)/'b/generated'
            session.prepare(args,a)
            session.prepare(args,b)
            scene=json.loads((a/'scene.json').read_text())
            self.assertEqual(scene['targets'],json.loads((b/'scene.json').read_text())['targets'])
            self.assertEqual(len(scene['targets']),5)
            self.assertEqual(len(scene['trees']),4)
            self.assertEqual(len(scene['gates']),2)
            self.assertEqual([g['xy'][0] for g in scene['gates']],[8.3,8.3])
            self.assertTrue(all(g['width']==.8 for g in scene['gates']))
            text=(a/'runtime.yaml').read_text()
            self.assertNotIn('scene.json',text)
            self.assertNotIn('target_bridge',text)
            world=ET.parse(a/'scene.world').getroot().find('world')
            h=next(el for el in world.findall('include') if el.findtext('name')=='landing_h_clone')
            self.assertTrue(h.findtext('pose').startswith('8.3 -3.5 '))


if __name__=='__main__': unittest.main()
