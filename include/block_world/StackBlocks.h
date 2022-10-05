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
  /**
   * @brief Initializes the Robot and Scene.
   */
  void initialize();
  /**
   * @brief Initializes the Planner (OMPL) and some settings.
   */
  void initializePlanner();

  /**
   * @brief Toggles the gripper open or close.
   *
   * @param open denote if open or close.
   *
   * @return success.
   */
  bool toggleGripper(bool open);

  /**
   * @brief Helper function to compute path between two joint configurations.
   *
   * @param start start configuration.
   * @param goal goal configuration.
   *
   * @return MotionPlanResponse object.
   */
  planning_interface::MotionPlanResponse computePath(std::vector<double> start,
                                                     std::vector<double> goal);

  /**
   * @brief Attach/Detach object to the end effector of the robot and removes it
   * from the environment.
   *
   * @param object object to attach
   * @param attach attach or detach
   */
  void attachObject(std::string object, bool attach);

  /**
   * @brief Executes motion to pregrasp of the object.
   *
   * @param object Object to grasp.
   *
   * @return success.
   */
  bool executePreGrasp(std::string object);

  /**
   * @brief Executes grasp of the object assuming already at pregrasp.
   *
   * @return success.
   */
  bool executeGrasp();

  /**
   * @brief Executes Placement of the object
   *
   * @param attached Name of attached object.
   * @param place_on Name of object to place on.
   *
   * @return  Success.
   */
  bool executePrePlace(std::string attached, std::string place_on);

  /**
   * @brief Places to object and gripper moves out of narrow passage.
   */
  bool executePostPlace();

  /**
   * @brief  Executes a given trajectory.
   *
   * @param trajectory Trajectory to execute.
   * @param parametrize If parametrization is needed.
   *
   * @return
   */
  bool executeTrajectory(robot_trajectory::RobotTrajectory& trajectory,
                         bool parametrize = true);
  /**
   * @brief Internal function to update robot and scene reprensentation.
   */
  void updateStateAndScene();

  /**
   * @brief Sets the robot to a valid random state.
   *
   * @param robot Robot object.
   *
   * @return Success.
   */
  bool setToRandomValidState(robowflex::RobotPtr& robot);

  /**
   * @brief Resets the planning scene and moves robot to home.
   */
  void reset();

  /**
   * @brief Callback to stacking action.
   *
   * @param goal
   */
  void stackBlocksCallback(const block_world::StackBlocksGoalConstPtr& goal);

  /**
   * @brief Callback to spawn objects query.
   *
   * @param req
   * @param res
   *
   * @return
   */
  bool spawnCallback(block_world::BlankSrv::Request& req,
                     block_world::BlankSrv::Response& res);
  /**
   * @brief Callback to reset query.
   *
   * @param req
   * @param res
   *
   * @return
   */
  bool resetCallback(block_world::BlankSrv::Request& req,
                     block_world::BlankSrv::Response& res);

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
  std::shared_ptr<actionlib::SimpleActionServer<block_world::StackBlocksAction>>
      as_;
  block_world::StackBlocksFeedback feedback_;
  block_world::StackBlocksResult result_;

  ros::ServiceServer spawn_service_;
  ros::ServiceServer reset_service_;

 public:
  StackBlocks(ros::NodeHandle& nh);

  /**
   * @brief Move arm to home.
   *
   * @return success.
   */
  bool homeArm();

  /**
   * @brief Stacks top object over bottom object.
   *
   * @param top Object on top.
   * @param bottom Object on bottom.
   * @param publish_feedback Flag to indicate if feedback should be published.
   *
   * @return
   */
  bool stackBlocks(std::string top, std::string bottom,
                   bool publish_feedback = false);

  /**
   * @brief Spawns objects on the table in valid positions.
   *
   * @param num Number of objects to spawn.
   */
  void spawnBlocks(int num = 2);
};
