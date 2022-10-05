#include "ros/ros.h"

// Robowflex / Moveit
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <robowflex_library/builder.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/planning.h>
#include <robowflex_library/random.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/util.h>
#include <robowflex_movegroup/services.h>
#include <robowflex_ompl/ompl_interface.h>

// OMPL
#include <ompl/base/spaces/RealVectorStateSpace.h>

// Actions
#include <actionlib/server/simple_action_server.h>
#include <block_world/BlankSrv.h>
#include <block_world/StackBlocksAction.h>

class StackBlocks {
 private:
  void initialize();
  void initializePlanner();

  bool toggleGripper(bool open);

  void spawnBlock(Eigen::Vector3d& pose);
  planning_interface::MotionPlanResponse computePath(std::vector<double> start,
                                                     std::vector<double> goal);

  void attachObject(std::string object, bool attach);

  bool executePreGrasp(std::string object);
  bool executeGrasp();

  bool executePrePlace(std::string attached, std::string place_on);
  void executePlace();
  bool executePostPlace();

  bool executeTrajectory(robot_trajectory::RobotTrajectory& trajectory,
                         bool parametrize = true);
  void updateStateAndScene();
  bool setToRandomValidState(robowflex::RobotPtr& robot);
  void reset();

  void stackBlocksCallback(const block_world::StackBlocksGoalConstPtr &goal);
  bool spawnCallback(block_world::BlankSrv::Request& req, block_world::BlankSrv::Response& res);
  bool resetCallback(block_world::BlankSrv::Request& req, block_world::BlankSrv::Response& res);

  ros::NodeHandle nh_;

  robowflex::movegroup::MoveGroupHelper move_group_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> mg_;

  std::shared_ptr<robowflex::Robot> robot_;
  std::shared_ptr<robowflex::Scene> scene_;
  std::string arm_planning_group_ = "panda_manipulator";
  std::string gripper_planning_group_ = "panda_hand";

  robowflex::OMPL::Settings settings_;
  std::shared_ptr<robowflex::OMPL::OMPLInterfacePlanner> planner_;
  std::shared_ptr<robowflex::SimpleCartesianPlanner> cartesian_planner_;
  std::shared_ptr<robowflex::MotionRequestBuilder> request_;
  trajectory_processing::IterativeParabolicTimeParameterization parametrizer_;

  std::string action_name_;
  std::shared_ptr<actionlib::SimpleActionServer<block_world::StackBlocksAction>> as_;
  block_world::StackBlocksFeedback feedback_;
  block_world::StackBlocksResult result_;
  
  ros::ServiceServer spawn_service_;
  ros::ServiceServer reset_service_;

 public:
  StackBlocks(ros::NodeHandle& nh);
  bool homeArm();
  bool stackBlocks(std::string top, std::string bottom, bool publish_feedback=false);
  void spawnBlocks(int num = 2);
};
