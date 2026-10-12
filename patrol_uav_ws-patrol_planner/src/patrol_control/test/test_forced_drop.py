"""Compile production forced tick and RPC admission with transport doubles."""
from pathlib import Path
import subprocess
import tempfile
import unittest
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_async_servo import method
P=Path(__file__).resolve().parents[1]
PROGRAM=r'''
#include "patrol_control/drop_action.h"
#include <cassert>
#include <string>
#include <functional>
#define ROS_WARN(...) ((void)0)
#define ROS_INFO(...) ((void)0)
namespace Eigen {struct Vector3d {};}
namespace ros {
struct Time {double value=100;static Time now(){return {};}bool isZero() const{return value==0;}};
bool operator>(Time a,Time b){return a.value>b.value;}
bool operator<=(Time a,Time b){return a.value<=b.value;}
struct ServiceClient {template<class T>bool call(T&){return true;}};
}
struct Pose {struct {double x=1,y=2,z=1.4;} position;};
struct PoseStamped {Pose pose;};
namespace patrol_control {
struct ServoAction {
 struct Request {
  std::string permission_epoch,mission_id,target_class,align_mode;
  int permission_revision=1,request_id=0,payload_slot=1,decision_seq=1,attempt=1,target_id=7;
  ros::Time target_first_seen;
 } request;
 struct Response {} response;
};
struct Servo {struct {int req;} request;struct{bool res=true;} response;};
DropActionResult classifyServoAction(ServoAction::Request,bool,ServoAction::Response){return DropActionResult::kSuccess;}
struct AsyncServo {
 using Execute=std::function<DropActionResult(int)>;int calls=0;
 bool submit(int,int,double,Execute){++calls;return true;}
};
struct Context {
 bool active=true,has_target=true;
 std::string mission_id="mission",semantic_target_class="panzer",align_mode="drop_circle";
 int decision_seq=1,attempt=1,semantic_target_id=7,payload_slot=1;
 ros::Time semantic_target_first_seen,deadline{190};
};
struct Permission {std::string reason="forced_drop_lock_timeout",permission_epoch="epoch";int permission_revision=1;};
struct Profile {bool was_reset=false;void reset(){was_reset=true;}};
bool nearWallAlignReleaseAllowed(bool inside,PoseStamped){return inside;}
class LLController {
public:
 enum Mode {Aligning,Run_point,Land};enum Task {MAIN_MISSION,CROSS_MISSION};
 Mode Drone_mode=Aligning;Task current_task_type=MAIN_MISSION;
 bool external_mission_mode_=true,require_vision_release_permission_=true;
 bool permission_fresh=true,flight_ready=true,motion_fresh=true,near_wall_align_fence_=true;
 bool compensated_alignment_enabled_=true,drop_compensate_before_descent_=true;
 bool drop_descent_started_=false,geometry_ready=false;
 mutable int geometry_checks=0;
 bool forced_drop_active_=false,drop_complete=false,have_servo_alignment_context_=true;
 bool servo_action_attempted_=false,servo_action_pending_=false;
 int servo_action_slot_=0,servo_action_id_=1,servo_alignment_decision_seq_=1,servo_alignment_target_id_=7;
 std::string servo_alignment_target_class_="panzer";
 DropActionResult servo_action_result_=DropActionResult::kPending;
 Permission release_authorization_;Context servo_alignment_context_;
 PoseStamped forced_drop_hold_,uav_pose,patrol_cmd,mavros_point_cmd,last_mavros_point_cmd;
 Profile drop_descent_profile_;
 struct {bool data=false;} servo_complete;
 double align_height=1.4,adjust_target_position[3]={},servo_call_timeout_sec_=2;
 double external_cross_recovery_setpoint_height_=1.3,external_standard_recovery_setpoint_height_=1.2,external_recovery_height_=1.0;
 bool cross_drop_completed=false,cross_mission_completed=false,have_planner_cmd=true;
 bool external_waiting_for_motion_=false,external_planner_height_hold_active_=true;int detect_point_counter=0;
 AsyncServo async_servo_;ros::ServiceClient servo_action_client_,servo_client;
 bool hasFreshMissionReleasePermission() const{return permission_fresh;}
 bool externalLandingControlReady(ros::Time) const{return flight_ready;}
 bool freshMotion(Eigen::Vector3d*,Eigen::Vector3d*) const{return motion_fresh;}
 bool compensatedDropSettled(bool){++geometry_checks;return geometry_ready;}
 void resetDropState(){forced_drop_active_=false;}
 void cleanupAfterCrossDrop(){align_height=1.8;current_task_type=MAIN_MISSION;}
 bool forcedDropAuthorized() const;bool forcedDropControlReady() const;bool forcedDropTick();DropActionResult executeDropAction(int);
};
METHODS
}
using namespace patrol_control;
int main(int argc,char**argv){
 std::string name=argv[1];LLController c;
 if(name=="force"){
  assert(c.forcedDropTick());assert(c.async_servo_.calls==1);assert(c.geometry_checks==0);
  assert(c.drop_descent_profile_.was_reset);assert(c.patrol_cmd.pose.position.z==1.4);
  for(int i=0;i<100;i++)c.forcedDropTick();assert(c.async_servo_.calls==1);
 }else if(name=="normal"){
  c.release_authorization_.reason="permission_granted";assert(!c.forcedDropTick());
  assert(c.executeDropAction(1)==DropActionResult::kPending);assert(c.async_servo_.calls==0);
 }else if(name=="blocked"){
  for(int n=0;n<6;n++){
   LLController b;
   if(n==0)b.permission_fresh=false;
   if(n==1)b.servo_alignment_context_.deadline={99};
   if(n==2)b.servo_alignment_decision_seq_=2;
   if(n==3)b.external_mission_mode_=false;
   if(n==4)b.Drone_mode=LLController::Land;
   if(n==5)b.require_vision_release_permission_=false;
   assert(!b.forcedDropTick());assert(b.async_servo_.calls==0);
  }
 }else if(name=="fresh"){
  for(int n=0;n<4;n++){
   LLController b;
   if(n==0)b.motion_fresh=false;
   if(n==1)b.flight_ready=false;
   if(n==2)b.near_wall_align_fence_=false;
   if(n==3)b.servo_alignment_context_.active=false;
   b.forcedDropTick();assert(b.async_servo_.calls==0);
  }
 }else if(name=="ack"){
  for(bool cross:{false,true}){
   LLController b;b.current_task_type=cross?LLController::CROSS_MISSION:LLController::MAIN_MISSION;
   b.forcedDropTick();b.permission_fresh=false;b.have_servo_alignment_context_=false;
   b.drop_complete=true;b.servo_complete.data=true;b.uav_pose.pose.position.z=.5;
   b.forcedDropTick();assert(b.Drone_mode==LLController::Aligning);assert(b.detect_point_counter==0);
   b.uav_pose.pose.position.z=1.1;b.forcedDropTick();
   assert(b.Drone_mode==LLController::Run_point);assert(b.detect_point_counter==1);
   assert(b.external_waiting_for_motion_);assert(!b.have_planner_cmd);
   assert(b.align_height==(cross?1.3:1.2));assert(b.patrol_cmd.pose.position.x==1);
   assert(b.async_servo_.calls==1);assert(!b.forced_drop_active_);
  }
 }else if(name=="unknown"){
  c.forcedDropTick();c.servo_action_result_=DropActionResult::kServiceCallFailed;
  for(int n=0;n<100;n++)c.forcedDropTick();
  assert(c.async_servo_.calls==1);assert(c.Drone_mode==LLController::Aligning);
  assert(c.detect_point_counter==0);assert(!c.drop_complete);
 }else if(name=="inflight"){
  c.servo_action_attempted_=true;c.servo_action_slot_=1;c.forcedDropTick();
  assert(c.async_servo_.calls==0);assert(c.forced_drop_active_);
 }else if(name=="height_stall"){
  c.release_authorization_.reason="forced_drop_height_stall";
  assert(!c.forcedDropTick());
  assert(c.executeDropAction(1)==DropActionResult::kPending);
  assert(c.async_servo_.calls==0);
 }else return 2;
}
'''
class ForcedDrop(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  s=(P/'src/patrol_control.cpp').read_text()
  code='\n'.join(method(s,x) for x in ('bool LLController::forcedDropAuthorized(',
      'bool LLController::forcedDropControlReady(','bool LLController::forcedDropTick(','DropActionResult LLController::executeDropAction('))
  cls.folder=tempfile.TemporaryDirectory(prefix='forced-drop-');cls.addClassCleanup(cls.folder.cleanup)
  r=Path(cls.folder.name);cpp=r/'test.cpp';cpp.write_text(PROGRAM.replace('METHODS',code))
  cls.binary=r/'test'
  subprocess.run(['g++','-std=c++14','-O1','-fsanitize=undefined','-fno-sanitize-recover=all',
     '-I',str(P/'include'),str(cpp),'-o',str(cls.binary)],check=True,timeout=60)
 def run_case(self,name):subprocess.run([str(self.binary),name],check=True,timeout=5)
 def test_force(self):self.run_case('force')
 def test_normal(self):self.run_case('normal')
 def test_blocked(self):self.run_case('blocked')
 def test_fresh(self):self.run_case('fresh')
 def test_ack(self):self.run_case('ack')
 def test_unknown(self):self.run_case('unknown')
 def test_inflight(self):self.run_case('inflight')
 def test_removed_height_stall_cannot_bypass_geometry(self):self.run_case('height_stall')
if __name__=='__main__':unittest.main()
