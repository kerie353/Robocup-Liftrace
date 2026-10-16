#!/usr/bin/env python3
"""Offline tests of ROS mission admission callbacks; no ROS processes/devices."""
import ast
import importlib.util
import math
from pathlib import Path
import threading
from types import SimpleNamespace as NS
import unittest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('admission',ROOT/'src/uav_mission/observation_admission.py')
admission=importlib.util.module_from_spec(spec); spec.loader.exec_module(admission)

def header(stamp=100.,seq=1,frame='camera_init'):
    return NS(stamp=NS(to_sec=lambda:stamp),seq=seq,frame_id=frame)


class ObservationAdmissionTest(unittest.TestCase):
    def setUp(self):
        # Execute the actual callback method ASTs without importing ROS or the
        # neural/flight runtime. This is not a parallel implementation.
        source=ast.parse((ROOT/'scripts/navigation_mission_manager.py').read_text())
        cls=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='NavigationMissionManager')
        names={'_on_prepare_map_rebuild','_on_observation','_readiness','_age_state'}
        body=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in names]
        node=ast.Module(body=[ast.ClassDef(name='Shell',bases=[],keywords=[],body=body,decorator_list=[])],type_ignores=[])
        writes=[]
        env=dict(math=math,ready_header=admission.ready_header,
                 rebuild_allowed=admission.rebuild_allowed,rebuild_completed=admission.rebuild_completed,
                 TriggerResponse=lambda **kw:NS(**kw),_phase_name=str,
                 rospy=NS(Time=NS(now=lambda:NS(to_sec=lambda:100.)),set_param=lambda *args:writes.append(args)))
        exec(compile(ast.fix_missing_locations(node),'<actual ROS admission methods>','exec'),env)
        self.shell=env['Shell'](); s=self.shell
        s._lock=threading.RLock(); s._runtime=None; s._observation_required=True
        s._observation_header=header(); s._map_rebuild_pending=False
        s._map_rebuild_after=0.; s._map_rebuild_generation=0
        s._mission_config=lambda:NS(mission_frame='camera_init')
        s._pose=NS(header=header(),pose=NS(position=NS(x=0.,y=0.,z=1.)))
        s._pose_max_age=.5; s._stamp_future_tolerance=.05; s._map_max_age=2.; s._require_map=True
        s._map=NS(header=header(),width=1,height=1,point_step=12,row_step=12,data=bytes(12))
        self.writes=writes

    def test_fresh_header(self):
        self.assertTrue(admission.ready_header(99.8,100.,'camera_init','camera_init'))
        for stamp in (0.,99.,100.01,float('nan'),float('inf')):
            self.assertFalse(admission.ready_header(stamp,100.,'camera_init','camera_init'))
        self.assertFalse(admission.ready_header(100.,100.,'map','camera_init'))

    def test_active_mission_and_unfinished_transactions_reject_rebuild(self):
        for phase in ('SEARCH','ALIGN','LAND','COMPLETE'):
            self.shell._runtime=NS(core=NS(phase=phase,active_action=object()))
            self.assertFalse(self.shell._on_prepare_map_rebuild(None).success)
        self.shell._runtime=NS(core=NS(phase='SEARCH',active_action=None))
        self.assertFalse(self.shell._on_prepare_map_rebuild(None).success)

    def test_idle_admission_latches_start_inhibition(self):
        self.assertTrue(self.shell._on_prepare_map_rebuild(None).success)
        self.assertTrue(self.shell._map_rebuild_pending)
        self.assertEqual((False,'map_rebuild_pending'),self.shell._readiness(100.))

    def test_repeated_prepare_does_not_move_generation_fence(self):
        self.shell._on_prepare_map_rebuild(None)
        self.shell._observation_header=header(100.,seq=2)
        self.shell._on_prepare_map_rebuild(None)
        self.assertEqual(1,self.shell._map_rebuild_generation)

    def test_old_or_zero_ready_cannot_release_inhibition(self):
        self.shell._on_prepare_map_rebuild(None)
        for message in (header(100.,1),header(0.,2),header(99.9,2),header(100.01,2),header(100.,2,'map')):
            self.shell._on_observation(message)
            self.assertTrue(self.shell._map_rebuild_pending)

    def test_only_fresh_new_generation_after_reset_releases_inhibition(self):
        self.shell._on_prepare_map_rebuild(None)
        self.shell._map_rebuild_after=99.8
        self.shell._on_observation(header(99.9,2))
        self.assertFalse(self.shell._map_rebuild_pending)
        self.assertEqual(('/navigation_observation/rebuilding',False),self.writes[-1])

    def test_delayed_previous_generation_is_ignored(self):
        self.shell._observation_header=header(100.,3)
        self.shell._on_observation(header(0.,2))
        self.assertEqual(3,self.shell._observation_header.seq)

    def test_invalidated_ready_rejects_start(self):
        self.shell._observation_header=header(0.,2)
        self.assertEqual((False,'observation_map_not_ready'),self.shell._readiness(100.))

    def test_new_ready_and_old_map_cannot_mix_epochs(self):
        self.shell._observation_header=header(100.,2)
        self.shell._map.header=header(99.9,1)
        self.assertEqual((False,'observation_snapshot_stamp_mismatch'),self.shell._readiness(100.))

    def test_empty_current_map_requires_fresh_observation_certificate(self):
        self.shell._map.width=0; self.shell._map.row_step=0; self.shell._map.data=b''
        self.assertEqual((True,'ready'),self.shell._readiness(100.))
        self.shell._observation_header=header(0.)
        self.assertFalse(self.shell._readiness(100.)[0])

    def test_legacy_map_contract_is_retained_when_disabled(self):
        self.shell._observation_required=False
        self.shell._map.width=0
        self.assertEqual((False,'map_empty'),self.shell._readiness(100.))
        self.assertFalse(self.shell._on_prepare_map_rebuild(None).success)

    def test_fresh_ready_does_not_make_an_old_cloud_fresh(self):
        self.shell._map.header=header(99.)
        self.assertFalse(self.shell._readiness(100.)[0])

    def test_ros_cloud_sequence_rewrite_does_not_break_certificate(self):
        self.shell._map.header=header(100.,999)
        self.assertEqual((True,'ready'),self.shell._readiness(100.))


if __name__=='__main__':unittest.main()
