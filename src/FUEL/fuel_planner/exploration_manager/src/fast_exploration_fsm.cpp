
#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

using Eigen::Vector4d;
vector<Eigen::Vector3d> feature_vis;

namespace fast_planner {
void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  // trigger_sub_ =
  //     nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
  trigger_server_ = nh.advertiseService("start_exploration", &FastExplorationFSM::triggerCallback, this);
  plan_traj_server_ = nh.advertiseService("plan_traj", &FastExplorationFSM::planTrajCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
  finish_pub = nh.advertise<std_msgs::Empty>("/finish_exploration", 10);

  exp_meeting_start_sub_ = nh.subscribe("exp_meeting_start", 1, &FastExplorationFSM::meetingStartCallback, this);
  exp_meeting_end_sub_ = nh.subscribe("exp_meeting_end", 1, &FastExplorationFSM::meetingEndCallback, this);
  exp_pause_sub_ = nh.subscribe("exp_pause", 1, &FastExplorationFSM::pauseCallback, this);

  /* fastplanner*/
  current_wp_ = 0;
  have_target_ = false;
  fast_state_ = FAST_INIT;
  fast_timer_ = nh.createTimer(ros::Duration(0.04), &FastExplorationFSM::fastFSMCallback, this);
  // fast_safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::fastCheckCollisionCallback, this);
  waypoint_server_ = nh.advertiseService("waypoint", &FastExplorationFSM::waypointCallback, this);
  fast_finish_pub_ = nh.advertise<std_msgs::Empty>("finish_fastplanning", 10);

  plan_fail_pub_ = nh.advertise<std_msgs::Empty>("plan_fail", 10);
}

void FastExplorationFSM::pauseCallback(const std_msgs::Empty& msg)
{
  fd_->static_state_ = true;
  transitState(WAIT_TRIGGER, "FSM");
}

void FastExplorationFSM::meetingStartCallback(const geometry_msgs::PoseStamped& msg)
{
  expl_manager_->ed_->in_meeting = true;
  expl_manager_->ed_->meeting_pose = msg;
}

void FastExplorationFSM::meetingEndCallback(const std_msgs::Empty& msg)
{
  expl_manager_->ed_->in_meeting = false;
}

bool FastExplorationFSM::waypointCallback(msgs::waypointRequest& req, msgs::waypointResponse& res) {
  // if (req.path.poses[0].pose.position.z < -0.1) return false;
  ROS_WARN("Get trigger");
  trigger_ = true;
  stop_fastplanner_ = false;

  end_pt_ << req.path.poses[0].pose.position.x, req.path.poses[0].pose.position.y, req.path.poses[0].pose.position.z;

  visualization_->drawGoal(end_pt_, 1, Eigen::Vector4d(0, 1, 0, 1.0));
  end_vel_.setZero();
  have_target_ = true;

  if (fast_state_ == FAST_WAIT_TARGET)
    changeFastState(FAST_GEN_NEW_TRAJ, "TRIG");
  else if (fast_state_ == FAST_EXEC_TRAJ)
    changeFastState(FAST_REPLAN_TRAJ, "TRIG");
  res.success = true;
  return true;
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  if (fd_ == nullptr) {
    ROS_ERROR("fd_ is null");
    return;
  }

  if (planner_manager_ == nullptr) {
    ROS_ERROR("planner_manager_ is null");
    return;
  }
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      // ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case CONTINUE_EXPLORE: {
      if (fd_->odom_vel_.norm() > 0.1) {
        // wait until stop completely
        return;
      }
      transitState(PLAN_TRAJ, "FSM");
    }

    case FINISH: {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
      }

      // Inform traj_server the replanning
      replan_pub_.publish(std_msgs::Empty());
      int res;
      try{
        res = callExplorationPlanner();
      }catch(...){
        res = FAIL;
      }
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        transitState(FINISH, "FSM");
        finish_pub.publish(std_msgs::Empty());
        fd_->static_state_ = true;
        clearVisMarker();
      } else if (res == FAIL) {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN_THROTTLE(1, "plan fail"); //TODO 可能1
        fd_->static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
        bspline_pub_.publish(fd_->newest_traj_);
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (time_to_end < fp_->replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        // ROS_WARN("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(PLAN_TRAJ, "FSM");
        // ROS_WARN("Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      if (t_cur > fp_->replan_thresh3_ && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        // ROS_WARN("Replan: periodic call=======================================");
      }
      break;
    }
  }
}

void FastExplorationFSM::fastFSMCallback(const ros::TimerEvent& e) {
  if (stop_fastplanner_) {
    changeFastState(FAST_WAIT_TARGET, "FSM");
    return;
  }
  switch (fast_state_) {
    case FAST_INIT: {
      if (!fd_->have_odom_) {
        return;
      }
      if (!trigger_) {
        return;
      }
      changeFastState(FAST_WAIT_TARGET, "FSM");
      break;
    }

    case FAST_WAIT_TARGET: {
      if (!have_target_)
        return;
      else {
        changeFastState(FAST_GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case FAST_GEN_NEW_TRAJ: {
      start_pt_ = fd_->odom_pos_;
      start_vel_ = fd_->odom_vel_;
      start_acc_.setZero();

      Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      start_yaw_(0) = atan2(rot_x(1), rot_x(0));
      start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = callKinodynamicReplan();
      if (success) {
        changeFastState(FAST_EXEC_TRAJ, "FSM");
      } else {
        // have_target_ = false;
        // changeFastState(WAIT_TARGET, "FSM");
        plan_fail_pub_.publish(std_msgs::Empty());
        changeFastState(FAST_GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case FAST_EXEC_TRAJ: {
      /* determine if need to replan */
      LocalTrajData* info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2 || (end_pt_ - pos).norm() < fp_->replan_thresh1_) {
        have_target_ = false;
        changeFastState(FAST_WAIT_TARGET, "FSM");
        fast_finish_pub_.publish(std_msgs::Empty());
        return;
      } else if ((end_pt_ - pos).norm() < fp_->replan_thresh1_) {
        // cout << "near end" << endl;
        return;
      } else if ((info->start_pos_ - pos).norm() < fp_->replan_thresh3_) {
        // cout << "near start" << endl;
        return;
      } else {
        changeFastState(FAST_REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case FAST_REPLAN_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();

      start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
      start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
      start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

      start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
      start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

      std_msgs::Empty replan_msg;
      replan_pub_.publish(replan_msg);

      bool success = callKinodynamicReplan();
      if (success) {
        changeFastState(FAST_EXEC_TRAJ, "FSM");
      } else {
        changeFastState(FAST_GEN_NEW_TRAJ, "FSM");
      }
      break;
    }
  }
}

bool FastExplorationFSM::callKinodynamicReplan() {
  auto t1 = ros::Time::now();
  ros::Time time_r = ros::Time::now();
  bool plan_success =
      planner_manager_->kinodynamicReplan(start_pt_, start_vel_, start_acc_, end_pt_, end_vel_);

  auto t2 = ros::Time::now();
  if (plan_success) {
    replan_time_.push_back((t2 - t1).toSec());
    double mean1 = 0.0;
    for (auto t : replan_time_)
      mean1 += t;
    mean1 /= replan_time_.size();
    ROS_INFO("Replan number: %d, mean traj: %lf", replan_time_.size(), mean1);
  }

  if (plan_success) {
    planner_manager_->planYaw(start_yaw_);

    auto info = &planner_manager_->local_data_;
    info->start_time_ = time_r;

    /* publish traj */
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();

    bspline_pub_.publish(bspline);

    /* visulization */
    auto plan_data = &planner_manager_->plan_data_;

    visualization_->drawGeometricPath(plan_data->kino_path_, 0.05, Eigen::Vector4d(1, 0, 1, 1));
    visualization_->drawBspline(info->position_traj_, 0.08, Eigen::Vector4d(1.0, 1.0, 0.0, 1), true,
                                0.15, Eigen::Vector4d(1, 0, 0, 1));

    return true;
  } else {
    // cout << "generate new traj fail." << endl;
    return false;
  }
}

void FastExplorationFSM::fastCheckCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  if (have_target_) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(end_pt_, /* time to program start + */ info->duration_) :
        edt_env->evaluateCoarseEDT(end_pt_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;
      double new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {
            new_x = end_pt_(0) + r * cos(theta / 57.3);
            new_y = end_pt_(1) + r * sin(theta / 57.3);
            new_z = end_pt_(2) + nz;

            Eigen::Vector3d new_pt(new_x, new_y, new_z);
            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt,
                                           /* time to program start+ */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset end_pt_ */
              goal(0) = new_x;
              goal(1) = new_y;
              goal(2) = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        end_pt_ = goal;
        have_target_ = true;
        end_vel_.setZero();

        if (fast_state_ == FAST_EXEC_TRAJ) {
          changeFastState(FAST_REPLAN_TRAJ, "SAFETY");
        }

        visualization_->drawGoal(end_pt_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFastState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFastState(FAST_REPLAN_TRAJ, "FSM");

        std_msgs::Empty emt;
        replan_pub_.publish(emt);
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (fast_state_ == FAST_STATE::FAST_EXEC_TRAJ) {
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);

    if (!safe) {
      // cout << "current traj in collision." << endl;
      ROS_WARN("current traj in collision.");
      changeFastState(FAST_REPLAN_TRAJ, "SAFETY");
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  //Draw local refined viewpoints info
  visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
                            Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  1),
                            "refined_view", 0, 6);
  visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  0, 1),
                            "refine_pair", 0, 6);
  for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
    visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
                                visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
                                ed_ptr->frontiers_.size()),
                                "n_points", i, 6);
  for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
    visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // Draw trajectory
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (++delay < 5) return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
  }

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = ros::Time::now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (ros::Time::now() - t1).toSec();
  //   ROS_WARN("Average astar time: %lf", astar_time);
  // }
}

bool FastExplorationFSM::triggerCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  // if (msg->poses[0].pose.position.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) 
  {
    res.success = 0;
    return true;
  }
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(CONTINUE_EXPLORE, "triggerCallback");
  stop_fastplanner_ = true;
  res.success = 1;
  return true;
}

bool FastExplorationFSM::planTrajCallback(msgs::plan_traj::Request& req, msgs::plan_traj::Response& res) {
  LocalTrajData stash_local_data;
  stash_local_data.traj_id_ = planner_manager_->local_data_.traj_id_;
  stash_local_data.position_traj_ = planner_manager_->local_data_.position_traj_;
  stash_local_data.velocity_traj_ = planner_manager_->local_data_.velocity_traj_;
  stash_local_data.acceleration_traj_ = planner_manager_->local_data_.acceleration_traj_;
  stash_local_data.yaw_traj_ = planner_manager_->local_data_.yaw_traj_;
  stash_local_data.yawdot_traj_ = planner_manager_->local_data_.yawdot_traj_;
  stash_local_data.yawdotdot_traj_ = planner_manager_->local_data_.yawdotdot_traj_;
  stash_local_data.duration_ = planner_manager_->local_data_.duration_;
  stash_local_data.start_time_ = planner_manager_->local_data_.start_time_;
  stash_local_data.start_pos_ = planner_manager_->local_data_.start_pos_;

  ros::Time start_time = req.start_time;
  Vector3d start_pt(req.pos.x, req.pos.y, req.pos.z);
  Vector3d start_vel(req.vel.x, req.vel.y, req.vel.z);
  Vector3d start_acc(req.acc.x, req.acc.y, req.acc.z);
  Vector3d start_yaw(req.yaw.x, req.yaw.y, req.yaw.z);
  Vector3d next_pos(req.next_pos.x, req.next_pos.y, req.next_pos.z);
  double next_yaw = req.next_yaw;

  int flag = expl_manager_->planTraj(start_pt, start_vel, start_acc, start_yaw, next_pos, next_yaw);
  if (flag != SUCCEED) {
    res.success = false;
  } else {
    auto info = &planner_manager_->local_data_;
    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = start_time;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
    res.success = true;
    res.bspline = bspline;
    res.duration = info->duration_;
  }
  expl_manager_->planner_manager_->local_data_ = stash_local_data;

  return true;
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      // ROS_WARN("Replan: collision detected==================================");
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
       << endl;
}

void FastExplorationFSM::changeFastState(FAST_STATE new_state, string pos_call) {
  int pre_s = int(fast_state_);
  fast_state_ = new_state;
}
}  // namespace fast_planner
