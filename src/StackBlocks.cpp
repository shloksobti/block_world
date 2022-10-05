#include <StackBlocks.h>

StackBlocks::StackBlocks(ros::NodeHandle& nh) : nh_(nh) {
  mg_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      arm_planning_group_);
  initialize();
  robowflex::RNG::setSeed(4);

  spawn_service_ =
      nh_.advertiseService("/spawn_blocks", &StackBlocks::spawnCallback, this);

  reset_service_ =
      nh_.advertiseService("/reset", &StackBlocks::resetCallback, this);

  action_name_ = "/stack_blocks";
  as_ = std::make_shared<
      actionlib::SimpleActionServer<block_world::StackBlocksAction>>(
      nh_, action_name_,
      boost::bind(&StackBlocks::stackBlocksCallback, this, _1), false);
  as_->start();
}

bool StackBlocks::spawnCallback(block_world::BlankSrv::Request& req,
                                block_world::BlankSrv::Response& res) {
  spawnBlocks();
  res.msg = "done";
  return true;
}

bool StackBlocks::resetCallback(block_world::BlankSrv::Request& req,
                                block_world::BlankSrv::Response& res) {
  reset();
  res.msg = "done";
  return true;
}

void StackBlocks::stackBlocksCallback(
    const block_world::StackBlocksGoalConstPtr& goal) {
  if (!as_->isActive() || as_->isPreemptRequested()) return;

  std::string top = goal->top;
  std::string bottom = goal->bottom;
  bool success = false;
  if (stackBlocks(top, bottom, true)) success = true;
  if (success) {
    result_.success = true;
    feedback_.status = "success";
  } else {
    result_.success = false;
    feedback_.status = "aborted";
  }
  as_->publishFeedback(feedback_);
  as_->setSucceeded(result_);
}

void StackBlocks::initialize() {
  robot_ = std::make_shared<robowflex::ParamRobot>();
  scene_ = std::make_shared<robowflex::Scene>(robot_);
  move_group_.pullState(robot_);
  move_group_.pullScene(scene_);
  robot_->loadKinematics(arm_planning_group_);
  initializePlanner();
  homeArm();
  scene_->fromYAMLFile("package://block_world/yaml/blocks.yaml");
  move_group_.pushScene(scene_);
}

bool StackBlocks::homeArm() {
  auto current = robot_->getState();
  auto res = computePath(
      current, {1.57, -0.785398, 0, -2.35619, 0, 1.5708, 0.785398, 0.05, 0.05});
  if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR("Cannot compute path to home.");
    return false;
  }
  if (!executeTrajectory(*res.trajectory_)) return false;
  return true;
}

bool StackBlocks::stackBlocks(std::string top, std::string bottom,
                              bool publish_feedback) {
  if (!toggleGripper(true)) return false;
  if (!executePreGrasp(top)) return false;  // Pick top
  if (!executeGrasp()) return false;
  if (publish_feedback) {
    feedback_.status = "Grasped";
    as_->publishFeedback(feedback_);
  }
  if (!toggleGripper(false)) return false;
  attachObject(top, true);
  if (!executePrePlace(top, bottom)) return false;  // Place on bottom
  attachObject(top, false);                         // Detach Object
  if (!toggleGripper(true)) return false;
  if (!executePostPlace()) return false;
  if (publish_feedback) {
    feedback_.status = "Placed";
    as_->publishFeedback(feedback_);
  }
  if (!homeArm()) return false;
  return true;
}

void StackBlocks::spawnBlocks(int num) {
  std::vector<std::vector<double>> positions;
  for (int n = 0; n < num; n++) {
    double x, y, z;
    // Check if this is valid placement
    while (true) {
      x = robowflex::RNG::uniformReal(0.3, 0.6);
      y = robowflex::RNG::uniformReal(0.2, -0.2);
      z = 0.52;

      double min_dist = 1000;
      for (auto& position : positions) {
        double dist = sqrt(pow(position[0] - x, 2) + pow(position[1] - y, 2));
        min_dist = std::min(min_dist, dist);
      }
      if (min_dist > 0.2) break;
    }

    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d({x, y, z});

    std::string name = "Block" + std::to_string(n + 1);
    auto block = robowflex::Geometry::makeBox(0.07, 0.07, 0.1);
    scene_->updateCollisionObject(name, block, pose);
    positions.push_back({x, y});
  }
  move_group_.pushScene(scene_);
}

bool StackBlocks::executeGrasp() {
  auto grasp_pose = robot_->getLinkTF("panda_hand");
  grasp_pose.translate(Eigen::Vector3d({0, 0, 0.03}));

  auto query = robowflex::Robot::IKQuery(arm_planning_group_, {grasp_pose},
                                         {"panda_hand"});
  query.setScene(scene_);
  auto res = cartesian_planner_->plan(*robot_->getScratchState(), query);
  if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR("Unable to find valid path to grasp.");
    return false;
  }
  if (!executeTrajectory(*res.trajectory_)) return false;
  return true;
}

bool StackBlocks::executePostPlace() {
  auto pose = robot_->getLinkTF("panda_hand");
  pose.translate(Eigen::Vector3d({0, 0, -0.08}));
  auto query =
      robowflex::Robot::IKQuery(arm_planning_group_, {pose}, {"panda_hand"});
  query.setScene(scene_);
  auto res = cartesian_planner_->plan(*robot_->getScratchState(), query);
  if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR("Unable to find valid path to post place.");
    return false;
  }
  if (!executeTrajectory(*res.trajectory_)) return false;
  return true;
}

void StackBlocks::initializePlanner() {
  settings_.simplify_solutions = true;
  planner_ = std::make_shared<robowflex::OMPL::OMPLInterfacePlanner>(robot_);
  cartesian_planner_ =
      std::make_shared<robowflex::SimpleCartesianPlanner>(robot_);
  planner_->initialize("package://block_world/config/ompl_planning.yaml",
                       settings_);

  request_ = std::make_shared<robowflex::MotionRequestBuilder>(
      planner_, arm_planning_group_);
  request_->setAllowedPlanningTime(5);
  request_->setConfig("RRTConnect");
}

bool StackBlocks::toggleGripper(bool open) {
  double gripper_pos;
  if (open)
    gripper_pos = 0.058;
  else
    gripper_pos = 0.03;

  const std::map<std::string, double> angles = {
      {"panda_finger_joint1", gripper_pos},
      {"panda_finger_joint2", gripper_pos}};

  updateStateAndScene();
  auto trajectory = robot_trajectory::RobotTrajectory(robot_->getModel(),
                                                      gripper_planning_group_);
  trajectory.addSuffixWayPoint(*robot_->getScratchState(), 0);
  robot_->setState(angles);
  trajectory.addSuffixWayPoint(*robot_->getScratchState(), 1.0);
  return executeTrajectory(trajectory, false);
}

bool StackBlocks::executeTrajectory(
    robot_trajectory::RobotTrajectory& trajectory, bool parametrize) {
  if (parametrize) parametrizer_.computeTimeStamps(trajectory);
  if (!move_group_.executeTrajectory(trajectory)) {
    ROS_ERROR("Failed to Execute Gripper Trajectory...");
    return false;
  }
  updateStateAndScene();
  return true;
}

void StackBlocks::attachObject(std::string object, bool attach) {
  if (attach) {
    mg_->attachObject(object, "panda_hand",
                      {"panda_leftfinger", "panda_rightfinger", "panda_hand"});
  } else {
    mg_->detachObject(object);
  }
  sleep(0.4);
  updateStateAndScene();
}

bool StackBlocks::executePrePlace(std::string attached, std::string place_on) {
  updateStateAndScene();
  request_->clearGoals();

  auto object_pose = scene_->getObjectPose(place_on);
  auto rot = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  object_pose.linear() = rot.toRotationMatrix();
  auto attached_object = robot_->getScratchState()->getAttachedBody(attached);
  auto poses = attached_object->getShapePosesInLinkFrame();

  for (int n = 0; n < 4; n++) {
    Eigen::Isometry3d place_pose(object_pose);
    place_pose.rotate(Eigen::AngleAxisd(-n * M_PI / 2, Eigen::Vector3d::UnitX())
                          .toRotationMatrix());
    place_pose.translate(-1 * poses[0].translation());
    place_pose.translation()[2] += 0.1;
    request_->addGoalPose("panda_hand", "panda_link0", place_pose);
  }

  request_->useSceneStateAsStart(scene_);
  auto res = planner_->plan(scene_, request_->getRequest());
  if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR("Path to place cannot be computed.");
    return false;
  }
  if (!executeTrajectory(*res.trajectory_)) return false;
  return true;
}

bool StackBlocks::executePreGrasp(std::string object) {
  updateStateAndScene();
  request_->clearGoals();

  // Set up grasping goals
  auto object_pose = scene_->getObjectPose(object);
  auto rot = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
  object_pose.linear() = rot.toRotationMatrix();

  for (int n = 0; n < 4; n++) {
    Eigen::Isometry3d grasp_pose(object_pose);
    grasp_pose.rotate(Eigen::AngleAxisd(-n * M_PI / 2, Eigen::Vector3d::UnitX())
                          .toRotationMatrix());
    grasp_pose.translate(Eigen::Vector3d({-0.05, 0, -0.15}));
    request_->addGoalPose("panda_hand", "panda_link0", grasp_pose);
  }

  request_->useSceneStateAsStart(scene_);
  auto res = planner_->plan(scene_, request_->getRequest());
  if (res.error_code_.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
    ROS_ERROR("Path to pre grasp cannot be computed.");
    return false;
  }
  if (!executeTrajectory(*res.trajectory_)) return false;
  return true;
}

void StackBlocks::reset() {
  scene_->removeCollisionObject("Block1");
  scene_->removeCollisionObject("Block2");
  move_group_.pushScene(scene_);
  homeArm();
}

planning_interface::MotionPlanResponse StackBlocks::computePath(
    std::vector<double> start, std::vector<double> goal) {
  request_->setStartConfiguration(start);
  request_->setGoalConfiguration(goal);
  planner_->refreshContext(scene_, request_->getRequestConst(), true);

  auto res = planner_->plan(scene_, request_->getRequest());
  return res;
}

void StackBlocks::updateStateAndScene() {
  move_group_.pullScene(scene_);
  move_group_.pullState(robot_);
}

bool StackBlocks::setToRandomValidState(robowflex::RobotPtr& robot) {
  while (true) {
    robot->getScratchState()->setToRandomPositions();
    robot->getScratchState()->update(true);
    if (scene_->getScene()->isStateValid(*robot->getScratchState(), "", false))
      return true;
  }
  return false;
}
