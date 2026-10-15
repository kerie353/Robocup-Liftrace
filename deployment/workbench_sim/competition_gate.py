#!/usr/bin/env python3
"""Reuse the SITL observer; video is optional in this bag-only profile."""
import json
import os
from pathlib import Path
import sys
import rospy
root=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(root/'deployment/board_trials_4x4/common/uav_board_trials/scripts'))
from trial_sim_gate import Gate
from trial_result import evaluate

class CompetitionGate(Gate):
    def save(self,reason):
        result=super().save(reason)
        result['checks'].pop('video_recorded',None)
        supervisor=dict(trial='full_mission',end_reason=reason,actuator_mode='mock')
        acceptance=evaluate(supervisor,self.latest,self.settings)
        result['status']='PASS' if acceptance['status']=='PASS' and all(result['checks'].values()) else 'INCOMPLETE'
        result['scope']='independent_competition_SITL'
        result['trial']='competition'
        for path in (self.out/'gate_status.json',Path(os.environ['SIM_RUN_DIR'])/'gate_status.json'):
            path.write_text(json.dumps(result,indent=2))
        return result

if __name__=='__main__':
    rospy.init_node('trial_sim_gate')
    CompetitionGate()
    rospy.spin()
