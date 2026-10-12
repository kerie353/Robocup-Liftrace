"""Boundary checks and production arbiter/proxy/Bridge calls without ROS nodes."""
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace as N
sys.path.insert(0, str(Path(__file__).resolve().parent))
from uav_mission.drop_timeout import DropTimeout
from uav_mission.motion_optimization import MotionOptimization
from test_release_transaction_followups import (
    ARB, ROS, Stamp, Clock, Publisher, permission, context, proxy, bridge,
    COMPLETED, action_identity, candidate,
)


class TimeoutPolicy(unittest.TestCase):
    def setUp(self):
        self.p=DropTimeout();self.p.bind(('m',1),100.)
    def test_twenty_seconds_and_renewal(self):
        self.assertEqual(self.p.update(119.999),'')
        self.p.bind(('m',1),119.999)
        self.assertEqual(self.p.update(120.),'forced_drop_lock_timeout')
    def test_new_identity_and_reset_clear_old_timeout(self):
        self.assertEqual(self.p.update(120.),'forced_drop_lock_timeout')
        self.p.bind(('m',2),121.)
        self.assertEqual(self.p.update(140.999),'')
        self.assertEqual(self.p.update(141.),'forced_drop_lock_timeout')
        self.p.reset();self.assertEqual(self.p.update(500.),'')
    def test_clock_rollback_latches_until_new_identity(self):
        self.p.update(102.);self.assertEqual(self.p.update(101.),'')
        self.assertEqual(self.p.update(131.),'')
        self.p.bind(('m',2),131.);self.assertEqual(self.p.update(151.),'forced_drop_lock_timeout')
    def test_paused_clock_never_advances_timeout(self):
        for _ in range(100):self.assertEqual(self.p.update(110.),'')
        self.assertEqual(self.p.update(119.999),'')
        self.assertEqual(self.p.update(120.),'forced_drop_lock_timeout')
    def test_invalid_clock_and_parameters(self):
        for value in (0.,-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):DropTimeout(value)
        self.assertEqual(self.p.update(float('nan')),'')
        self.assertEqual(self.p.update(120.),'')


class TimeoutTransaction(unittest.TestCase):
    def setUp(self):
        Clock.value=100.;self.m=permission();self.c=c=context(self.m,True)
        c.schema_version=1;c.command=2;c.has_target=True;c.class_profile='r2026'
        c.header=N(stamp=Stamp(100.),frame_id='camera_init')
        c.target_pose=N(header=N(frame_id='camera_init'))
        c.deadline=Stamp(190.);c.target_observation_stamp=Stamp(99.9)
        self.a=a=ARB.__new__(ARB)
        a._drop_timeout_enabled=True;a._drop_timeout=DropTimeout()
        a._alignment_context=None;a._require_evidence_context=True;a._class_profile='r2026'
        a._next_slot=1;a._payload_slots=3;a._blocked_slots=set();a._revoked_actions=set()
        a._authorized_actions={};a._called_actions={};a._released_targets=set();a._completed_slots=set()
        a._commitment=None;a._evidence_context=None;a._lock=threading.RLock()
        a._required_control_state=2;a._align_mode='drop_circle';a._control_state=2
        a._control_state_stamp=Stamp(100.)
        a._control_state_timeout=a._pose_timeout=a._evidence_timeout=.5
        a._pose=N(header=N(stamp=Stamp(100.),frame_id='camera_init'),pose=N(position=N(x=0.,y=0.,z=1.2)))
        a._permission_epoch='test';a._permission_revision=0;a._permission_lifetime=.25
        a._permission_pub=Publisher();a._authorization_pub=Publisher();a._permission_state_pub=Publisher()
        a._current_evidence_source=lambda now:(False,'no_release_evidence',None)
        a._commitment_policy=N(evaluate=lambda **kwargs:(False,'no_release_commitment'))
        ARB._evaluate.__globals__['MODE_TARGET_CLASS']={'drop_circle':'circle','drop_cross':'red_cross'}
        ARB._on_alignment_context.__globals__['AlignmentTargetContext']=N(SCHEMA_VERSION=1,ALIGN=2)
        a._on_alignment_context(c)
    def advance(self,t,height=1.2):
        Clock.value=t;self.c.header.stamp=Stamp(t);self.a._control_state_stamp=Stamp(t)
        self.a._pose.header.stamp=Stamp(t);self.a._pose.pose.position.z=height
    def force(self):
        self.advance(120.);return self.a._evaluate(Stamp(120.))
    def publish(self):
        ROS.Duration=Stamp
        try:self.a._publish_permission(None)
        finally:del ROS.Duration
        return self.a._permission_pub.messages[-1]
    def test_grants_without_visual_or_release_altitude(self):
        allowed,reason,source=self.force();self.assertTrue(allowed)
        self.assertEqual(reason,'forced_drop_lock_timeout');self.assertEqual(source['decision_seq'],self.c.decision_seq)
    def test_low_height_hover_never_forces_before_twenty_seconds(self):
        for height in (.1,.43,.59,.60,1.4):
            self.setUp()
            for i in range(1,200):
                t=100+i/10;self.advance(t,height)
                self.assertFalse(self.a._evaluate(Stamp(t))[0])
                self.assertEqual(self.a._drop_timeout.reason,'')
            self.advance(120.,height)
            allowed,reason,_=self.a._evaluate(Stamp(120.))
            self.assertTrue(allowed);self.assertEqual(reason,'forced_drop_lock_timeout')
    def test_published_identity_ignores_old_visual_context(self):
        self.force();self.a._evidence_context=N(mission_id='wrong',decision_seq=900,attempt=1,
            semantic_target_first_seen=Stamp(1),semantic_target_id=999,semantic_target_class='tent')
        p=self.publish();self.assertTrue(p.permitted)
        self.assertEqual(action_identity(p),action_identity(self.m))
        self.assertIn(action_identity(p),self.a._authorized_actions)
    def test_revoke_stale_expired_and_frame_block(self):
        for field in ('revoke','stale','deadline','frame'):
            self.setUp();self.force()
            if field=='revoke':self.c.active=False;self.a._on_alignment_context(self.c)
            elif field=='stale':self.c.header.stamp=Stamp(110.)
            elif field=='deadline':self.c.deadline=Stamp(120.)
            else:self.a._pose.header.frame_id='wrong'
            self.assertIsNone(self.a._forced_source(Stamp(120.)))
    def test_stale_pose_control_used_slot_and_invalid_feedback_block(self):
        for field in ('pose','control','slot','nan','future','future_control'):
            self.setUp();self.advance(120.)
            if field=='pose':self.a._pose.header.stamp=Stamp(100.)
            elif field=='control':self.a._control_state=1
            elif field=='slot':self.a._blocked_slots.add(1)
            elif field=='future':self.a._pose.header.stamp=Stamp(120.1)
            elif field=='future_control':self.a._control_state_stamp=Stamp(120.1)
            else:self.a._pose.pose.position.z=float('nan')
            self.assertFalse(self.a._evaluate(Stamp(120.))[0])
    def test_renewal_preserves_start(self):
        self.advance(119.);self.a._on_alignment_context(self.c)
        self.assertEqual(self.a._drop_timeout.started,100.);self.assertTrue(self.force()[0])
    def test_raw_once_ack_enters_recovery_without_visual(self):
        self.force();p=self.publish();o=proxy();o._permission=p
        ok,fact,_=o._execute_request(1,p)
        self.assertTrue(ok);self.assertEqual(fact,COMPLETED);self.assertEqual(o._raw_client.calls,1)
        self.assertFalse(o._execute_request(1,p)[0]);self.assertEqual(o._raw_client.calls,1)
        self.a._on_result(o._result_pub.messages[-1]);self.assertEqual(self.a._next_slot,2)
        b=bridge();b._transaction.phase='ALIGN_COMMAND_SENT';b._transaction.strict_evidence_stamp_ns=0
        target=b._transaction.decision.target
        for msg in o._result_pub.messages:
            msg.mission_id=b._transaction.decision.mission_id;msg.decision_seq=b._transaction.decision.decision_seq
            msg.attempt=target.attempt;msg.payload_slot=target.payload_slot;msg.target_id=target.target_id
            msg.target_first_seen=Stamp(target.first_seen_ns/1e9);msg.target_class=target.class_name
            msg.align_mode=b._transaction.align_mode;b._on_release_result(msg)
        self.assertEqual(b._transaction.phase,'RECOVERY');self.assertEqual(b._transaction.strict_evidence_stamp_ns,0)
        b._update_recovery.__globals__['MotionOptimization']=MotionOptimization
        b._recovery_height=1.;b._control_state=1;b._align_mode='disabled'
        b._control_state_receipt_ns=b._align_mode_receipt_ns=120_010_000_000
        b._odom_rejection_reason=lambda sample,now: ''
        for t in (120.1,120.4,120.7):
            Clock.value=t;ns=round(t*1e9)
            b._update_recovery(N(stamp_ns=ns,x=0.,y=0.,z=1.4),ns)
        self.assertEqual(b._transaction.phase,'TERMINAL')
        self.assertFalse(b.errors);self.assertEqual(b.core.committed_slots,1)
        # Core resumes the mission only after the physical ACK and recovery.
        self.assertEqual(b.core.phase.value,'SEARCH')
        next_action=b.core.choose_confirmed(candidate(target_id=8,class_name='panzer',now=121.),121.,(0.,0.))
        self.assertIsNotNone(next_action);self.assertEqual(next_action.payload_slot,2)



class TimeoutConfiguration(unittest.TestCase):
    ROOT=Path(__file__).resolve().parents[4]
    def check_generated(self, gen, settings, rig):
        import tempfile
        import yaml
        for fc in ((0.,0.,0.),(.01,-.01,-.05)):
            with tempfile.TemporaryDirectory() as out:
                ref=gen(self.ROOT,out,settings,fc,rig)
                params=yaml.safe_load((Path(out)/'overrides.yaml').read_text())
                self.assertNotIn('/release_permission_arbiter/drop_timeout/ground_z',params)
                control=yaml.safe_load((Path(out)/'control.yaml').read_text())['drop_system']
                self.assertAlmostEqual(control['release_setpoint_height']-ref['ground_z'],settings['drop_agl'])
    def test_competition_ground_reference(self):
        import yaml
        from uav_mission.competition_config import generate
        settings=yaml.safe_load((self.ROOT/'deployment/competition/field.example.yaml').read_text())
        settings.update(site_confirmed=True,corridor_waypoints=[dict(x=6.7,y=4.,agl=1.4),
            dict(x=6.7,y=4.,agl=.9),dict(x=8.3,y=4.,agl=.9),dict(x=8.3,y=-4.,agl=.9)],landing_xy=[8.5,-4.2])
        rig=yaml.safe_load((self.ROOT/'patrol_uav_ws-patrol_planner/src/uav_mission/config/competition/known_rig.yaml').read_text())
        self.check_generated(generate,settings,rig)
    def test_nine_trial_ground_references(self):
        import yaml
        import importlib.util
        base=self.ROOT/'deployment/board_trials_4x4'
        scripts=base/'common/uav_board_trials/scripts'
        sys.path.insert(0,str(scripts))
        self.addCleanup(lambda:sys.path.remove(str(scripts)))
        spec=importlib.util.spec_from_file_location('timeout_trial_config',scripts/'trial_config.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        rig=yaml.safe_load((base/'common/uav_board_trials/config/known_rig.yaml').read_text())
        for p in sorted(base.glob('*/settings.yaml')):
            settings=yaml.safe_load(p.read_text())
            if settings.get('trial_kind') in ('corridor_landing','full_mission'):
                settings.update(corridor_waypoints=[dict(x=.6,y=0.,agl=1.),dict(x=1.5,y=.4,agl=1.)],landing_xy=[2.5,0.])
            self.check_generated(module.generate,settings,rig)

if __name__=='__main__':unittest.main()
