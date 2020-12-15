/*********************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2019 PickNik LLC.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Henning Kayser, Simon Goldstein
   Desc:   A demo to show MoveIt Task Constructor in action
*/

#include <moveit_task_constructor_demo/pick_place_task.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_task_constructor_demo {
constexpr char LOGNAME[] = "pick_place_task";
PickPlaceTask::PickPlaceTask(const std::string& task_name, const ros::NodeHandle& nh)
  : nh_(nh), task_name_(task_name), execute_("execute_task_solution", true) {}

void PickPlaceTask::loadParameters() {
	/****************************************************
	 *                                                  *
	 *               Load Parameters                    *
	 *                                                  *
	 ***************************************************/
	ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
	ros::NodeHandle pnh("~");

	// Planning group properties
	size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_group_name", arm_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_group_name", hand_group_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "eef_name", eef_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_frame", hand_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "world_frame", world_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "grasp_frame_transform", grasp_frame_transform_);

	// Predefined pose targets
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_open_pose", hand_open_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "hand_close_pose", hand_close_pose_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "arm_home_pose", arm_home_pose_);

	// Target object
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "surface_link", surface_link_);
	support_surfaces_ = { surface_link_ };

	// Pick/Place metrics
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_min_dist", approach_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "approach_object_max_dist", approach_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_min_dist", lift_object_min_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "lift_object_max_dist", lift_object_max_dist_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_surface_offset", place_surface_offset_);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "place_pose", place_pose_);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);
}

void PickPlaceTask::init() {
	ROS_INFO_NAMED(LOGNAME, "Initializing task pipeline");
	// Sampling planner

	// Cartesian planner
	task_.reset();
	task_.reset(new moveit::task_constructor::Task("test"));
	Task& t = *task_;
	t.loadRobotModel();

	auto sampling_planner_ = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner_ = std::make_shared<solvers::PipelinePlanner>();
	sampling_planner_->setProperty("max_velocity_scaling_factor", .5);
	sampling_planner_->setProperty("max_acceleration_scaling_factor", 0.5);
	sampling_planner_->setProperty("goal_joint_tolerance", 1e-4);  // 1e-5

	auto cartesian_planner_ = std::make_shared<solvers::CartesianPath>();
	cartesian_planner_ = std::make_shared<solvers::CartesianPath>();
	cartesian_planner_->setMaxVelocityScaling(0.5);
	cartesian_planner_->setMaxAccelerationScaling(0.5);
	cartesian_planner_->setStepSize(.01);
	cartesian_planner_->setJumpThreshold(1.5);

	moveit::task_constructor::Stage* current_state_ = nullptr;

	/****************************************************
	 *                                                  *
	 *               Grasp pipeline                     *
	 *                                                  *
	 ***************************************************/
	{  // Current State
		auto _current_state = std::make_unique<stages::CurrentState>("current state");
		current_state_ = _current_state.get();
		t.add(std::move(_current_state));
	}
	{  // Move Gripper
		auto stage = std::make_unique<stages::MoveTo>(name, sampling_planner_);
		stage->setGoal(robot_state);
		stage->setGroup(group_name);
		t.add(std::move(stage));
	}
	{  // Connect
		auto stage = std::make_unique<stages::Connect>(
		    "Connect1", stages::Connect::GroupPlannerVector{ { group_name, sampling_planner_ } });
		stage->setTimeout(timeout_connect_);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
	{  // Alternatives + GenerateGraspPose
		auto parallel_container = std::make_unique<Alternatives>("Alternatives grasp container");

		for (int i = 0; i < size; ++i) {
			auto stage = std::make_unique<stages::GenerateGraspPose>(name);
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setEndEffector(eef_name);
			stage->setObject(target_frame_name);
			stage->setObjectPose(target_frame[i]);
			stage->setAngleDelta(target_angle_delta[i]);
			stage->setAngleAxis(target_axis[i]);
			stage->setEndEffector(eef_name);
			stage->setPreGraspPose(pre_grasp_state[i]);
			stage->setTimeout(timeout_);
			stage->setMonitoredStage(current_state_);

			auto wrapper = std::make_unique<stages::ComputeIK>("insertion pose IK", std::move(stage));
			wrapper->setTimeout(timeout_);
			wrapper->setMaxIKSolutions(max_ik_solutions);
			wrapper->setIKFrame(tcp_frame, tcp_frame_name);
			wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
			wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
			wrapper->setProperty("eef", eef_name);
			wrapper->setProperty("group", arm_group_name);
			parallel_container->insert(std::move(wrapper));
		}
		t.add(std::move(parallel_container));
	}
	{  // Allow collisions
		auto stage = std::make_unique<stages::ModifyPlanningScene>(name);
		stage->allowCollisions(
		    first_, task_->getRobotModel()->getJointModelGroup(second_)->getLinkModelNamesWithCollisionGeometry(), flag);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
	{  // Attach object
		auto stage = std::make_unique<stages::ModifyPlanningScene>(name);
		stage->attachObjects(objects, link_name, flag);
		current_state_ = stage.get();
		t.add(std::move(stage));
	}
	{  // Move relative up wrt object insertion
		auto stage = std::make_unique<stages::MoveRelative>(name, cartesian_planner_);
		const geometry_msgs::Vector3Stamped& direction = boost::any_cast<geometry_msgs::Vector3Stamped>(direction_any);
		stage->setDirection(direction);
		stage->setMinMaxDistance(min_distance, max_distance);
		stage->properties().set("link", tcp_frame_name);
		stage->setGroup(group_name);
		stage->setPoseTransform(tcp_frame);
		stage->setTimeout(timeout_);
		t.add(std::move(stage));
	}

	/****************************************************
	 *                                                  *
	 *        Push button (with stylus) pipeline        *
	 *                                                  *
	 ***************************************************/
	{  // Second Connect
		auto stage = std::make_unique<stages::Connect>(
		    "Connect2", stages::Connect::GroupPlannerVector{ { group_name, sampling_planner_ } });
		stage->setTimeout(timeout_connect_);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
	{  // GenerateInsertionPose this stage is identical to the generateGraspPose except that it dosent need a hand_state
		auto stage = std::make_unique<stages::GenerateInsertPose>(name);
		stage->properties().configureInitFrom(Stage::PARENT);
		stage->setEndEffector(eef_name);
		stage->setObject(target_frame_name);
		stage->setObjectPoseTransform(target_frame);
		stage->setAngleDelta(target_angle_delta);
		stage->setAxis(target_axis);
		stage->setMonitoredStage(current_state_);
		stage->setEndEffector(eef_name);
		stage->setTimeout(timeout_);

		auto wrapper = std::make_unique<stages::ComputeIK>("insertion pose IK", std::move(stage));
		wrapper->setTimeout(timeout_);
		wrapper->setMaxIKSolutions(8);
		wrapper->setIKFrame(tcp_frame, tcp_frame_name);
		wrapper->properties().configureInitFrom(Stage::PARENT, { "eef", "group" });
		wrapper->properties().configureInitFrom(Stage::INTERFACE, { "target_pose" });
		wrapper->setProperty("eef", eef_name);
		wrapper->setProperty("group", arm_group_name);
		t.add(std::move(wrapper));
	}
	{  // Allow collisions
		auto stage = std::make_unique<stages::ModifyPlanningScene>(name);
		stage->allowCollisions(
		    first_, task_->getRobotModel()->getJointModelGroup(second_)->getLinkModelNamesWithCollisionGeometry(), flag);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
	{  // Move relative Push button
		auto stage = std::make_unique<stages::MoveRelative>(name, cartesian_planner_);
		const geometry_msgs::Vector3Stamped& direction = boost::any_cast<geometry_msgs::Vector3Stamped>(direction_any);
		stage->setDirection(direction);
		stage->setMinMaxDistance(min_distance, max_distance);
		stage->properties().set("link", tcp_frame_name);
		stage->setGroup(group_name);
		stage->setPoseTransform(tcp_frame);
		stage->setTimeout(timeout_);
		t.add(std::move(stage));
	}

	{  // Move relative Retreat
		auto stage = std::make_unique<stages::MoveRelative>(name, cartesian_planner_);
		const geometry_msgs::Vector3Stamped& direction = boost::any_cast<geometry_msgs::Vector3Stamped>(direction_any);
		stage->setDirection(direction);
		stage->setMinMaxDistance(min_distance, max_distance);
		stage->properties().set("link", tcp_frame_name);
		stage->setGroup(group_name);
		stage->setPoseTransform(tcp_frame);
		stage->setTimeout(timeout_);
		t.add(std::move(stage));
	}
	{  // Frobid collisions
		auto stage = std::make_unique<stages::ModifyPlanningScene>(name);
		stage->allowCollisions(
		    first_, task_->getRobotModel()->getJointModelGroup(second_)->getLinkModelNamesWithCollisionGeometry(), flag);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
}

bool PickPlaceTask::plan() {
	ROS_INFO_NAMED(LOGNAME, "Start searching for task solutions");
	ros::NodeHandle pnh("~");
	int max_solutions = pnh.param<int>("max_solutions", 10);

	try {
		task_->plan(max_solutions);
	} catch (InitStageException& e) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Initialization failed: " << e);
		return false;
	}
	if (task_->numSolutions() == 0) {
		ROS_ERROR_NAMED(LOGNAME, "Planning failed");
		return false;
	}
	return true;
}

bool PickPlaceTask::execute() {
	ROS_INFO_NAMED(LOGNAME, "Executing solution trajectory");
	moveit_task_constructor_msgs::ExecuteTaskSolutionGoal execute_goal;
	task_->solutions().front()->fillMessage(execute_goal.solution);

	auto test = task_->solutions().front()->start()->scene();
	auto a = execute_goal.solution.start_scene;
	execute_.sendGoal(execute_goal);
	execute_.waitForResult();
	moveit_msgs::MoveItErrorCodes execute_result = execute_.getResult()->error_code;

	if (execute_result.val != moveit_msgs::MoveItErrorCodes::SUCCESS) {
		ROS_ERROR_STREAM_NAMED(LOGNAME, "Task execution failed and returned: " << execute_.getState().toString());
		return false;
	}

	return true;
}
}  // namespace moveit_task_constructor_demo
