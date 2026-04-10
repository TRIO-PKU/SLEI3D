#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <std_srvs/Trigger.h>
#include <msgs/plan_traj.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

#include <msgs/waypoint.h>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, CONTINUE_EXPLORE, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };
enum FAST_STATE { FAST_INIT, FAST_WAIT_TARGET, FAST_GEN_NEW_TRAJ, FAST_REPLAN_TRAJ, FAST_EXEC_TRAJ, FAST_REPLAN_NEW };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  /* fastpolanner */
  ros::Timer fast_timer_, fast_safety_timer_;
  void fastFSMCallback(const ros::TimerEvent& e);
  void fastCheckCollisionCallback(const ros::TimerEvent& e);

  vector<double> replan_time_;

  bool trigger_, have_target_;
  FAST_STATE fast_state_;
  bool stop_fastplanner_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  Eigen::Vector3d end_pt_, end_vel_;                              // target state
  int current_wp_;

  ros::Publisher fast_finish_pub_;
  ros::ServiceServer waypoint_server_;

  bool waypointCallback(msgs::waypointRequest& req, msgs::waypointResponse& res);
  void changeFastState(FAST_STATE new_state, string pos_call);
  bool callKinodynamicReplan();  

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_, ins_point_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, exp_meeting_start_sub_, exp_meeting_end_sub_, exp_pause_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, finish_pub, ins_pub, ins_vis_pub, plan_fail_pub_;
  ros::ServiceServer trigger_server_, plan_traj_server_;

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  bool triggerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  bool planTrajCallback(msgs::plan_traj::Request& req, msgs::plan_traj::Response& res);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();
  void clearVisMarker();
  void genInsPoint(const ros::TimerEvent& e);
  void feature_vis_cb(const geometry_msgs::PoseStamped& msg);

  bool in_meeting;
  void meetingStartCallback(const geometry_msgs::PoseStamped& msg);
  void meetingEndCallback(const std_msgs::Empty& msg);
  void pauseCallback(const std_msgs::Empty& msg);

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif