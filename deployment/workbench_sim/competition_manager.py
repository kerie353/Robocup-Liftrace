#!/usr/bin/env python3
"""SITL adapter for the real competition manager class and runtime strategy."""
import importlib.util
import os
import json
from pathlib import Path
import rospy
from std_msgs.msg import String

if __name__=='__main__':
    rospy.init_node('mission_manager')
    if (not rospy.get_param('/use_sim_time',False) or not os.environ.get('SIM_RUN_DIR')
            or rospy.get_param('/board_trials/actuator_mode','real')!='mock'):
        raise RuntimeError('Competition simulation requires SITL clock, wrapper and mock actuator')
    path=Path(__file__).resolve().parents[2]/'patrol_uav_ws-patrol_planner/src/uav_mission/scripts/navigation_competition_manager.py'
    spec=importlib.util.spec_from_file_location('workbench_competition_strategy',str(path))
    module=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.CompetitionManager(hardware=True)
    metadata=(Path(os.environ['SIM_RUN_DIR'])/'run_metadata.json').read_text()
    publishers=[rospy.Publisher(topic,String,queue_size=1,latch=True)
                for topic in ('/competition/run_metadata','/board_trials/run_metadata')]
    for publisher in publishers: publisher.publish(String(data=metadata))
    rospy.spin()
