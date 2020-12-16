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

Eigen::Isometry3d createIsomtery3d(double x, double y, double z, double qw, double qx, double qy, double qz) {
	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
	tf.translate(Eigen::Vector3d(x, y, z));

	Eigen::Quaterniond quat = Eigen::Quaterniond(qw, qx, qy, qz);
	tf.rotate(quat);

	return tf;
}
void PickPlaceTask::loadParameters() {
	/****************************************************
	 *                                                  *
	 *               Load Parameters                    *
	 *                                                  *
	 ***************************************************/
	ROS_INFO_NAMED(LOGNAME, "Loading task parameters");
	ros::NodeHandle pnh("~");

	// Planning group properties
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

	double timeout_ = 0.1;
	double timeout_connect_ = 5.0;
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
	moveit_msgs::RobotState r;
	{  // Open Gripper

		r.is_diff = true;
		r.joint_state.name.resize(1);
		r.joint_state.name[0] = "robotiq_2f_140_left_outer_knuckle_joint";
		r.joint_state.position.resize(1);
		r.joint_state.position[0] = 0.43197133851644276;

		r.joint_state.effort.resize(1);
		r.joint_state.effort[0] = 100000;

		auto stage = std::make_unique<stages::MoveTo>("Open Gripper", sampling_planner_);
		stage->setGoal(r);
		stage->setGroup("robotiq_2f_140");
		t.add(std::move(stage));
	}
	{  // Connect
		auto stage = std::make_unique<stages::Connect>(
		    "Connect1", stages::Connect::GroupPlannerVector{ { "left_eef", sampling_planner_ } });
		stage->setTimeout(timeout_connect_);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
	{  // Alternatives + GenerateGraspPose
		auto parallel_container = std::make_unique<Alternatives>("Alternatives grasp container");

		std::string arm_group_name = "left_eef";
		std::string eef_name = "gripper";
		std::string tcp_frame_name = "robotiq_2f_140";

		std::string target_frame_name = "stylus";
		int max_ik_solutions = 8;

		std::vector<Eigen::Isometry3d> target_frame_list;
		std::vector<Eigen::Vector3d> target_axis_list;
		std::vector<double> target_angle_delta_list;
		std::vector<moveit_msgs::RobotState> pre_grasp_state_list;

		int pose_nb = 4;

		Eigen::Isometry3d dst_frame = createIsomtery3d(0, 0, 0.06, 0.707107, 0, -0.707107, 0);
		Eigen::Isometry3d frame = dst_frame;
		Eigen::Isometry3d tcp_frame = createIsomtery3d(-0.000323486, 0, 0, 1, 0, 0, 0);

		std::vector<double> delta_lengths;
		delta_lengths.push_back(0);
		delta_lengths.push_back(0.0089999999999999993);
		delta_lengths.push_back(-0.0089999999999999993);

		std::vector<double> delta_heights;
		delta_heights.push_back(0);
		delta_heights.push_back(0.014999999999999999);
		delta_heights.push_back(-0.014999999999999999);

		for (int i = 0; i < pose_nb; ++i) {
			for (const auto& curr_length : delta_lengths) {
				frame = dst_frame;
				frame.translate(Eigen::Vector3d(0, 0, curr_length));
				for (const auto& curr_height : delta_heights) {
					Eigen::Isometry3d frame2 = frame;
					frame2.translate(Eigen::Vector3d(curr_height, 0, 0));

					frame2 = frame2 * Eigen::AngleAxisd(i * 1.5707963267948966, Eigen::Vector3d(1, 0, 0));

					target_frame_list.push_back(frame2);
					target_axis_list.push_back(Eigen::Vector3d::UnitY());
					target_angle_delta_list.push_back(2.0 * M_PI / 8.0);
					pre_grasp_state_list.push_back(r);
				}
			}
		}
		for (int i = 0; i < target_frame_list.size(); ++i) {
			auto stage = std::make_unique<stages::GenerateGraspPose>("Generate grasp pose");
			stage->properties().configureInitFrom(Stage::PARENT);
			stage->setEndEffector(eef_name);
			stage->setObject(target_frame_name);
			stage->setObjectPose(target_frame_list[i]);
			stage->setAngleDelta(target_angle_delta_list[i]);
			stage->setAngleAxis(target_axis_list[i]);
			stage->setEndEffector(eef_name);
			stage->setPreGraspPose(pre_grasp_state_list[i]);
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

	{  // Close Gripper
		auto stage = std::make_unique<stages::MoveTo>("Close Gripper", sampling_planner_);
		r.is_diff = true;
		r.joint_state.name.resize(1);
		r.joint_state.name[0] = "robotiq_2f_140_left_outer_knuckle_joint";
		r.joint_state.position.resize(1);
		r.joint_state.position[0] = 0.63634669853952763;

		r.joint_state.effort.resize(1);
		r.joint_state.effort[0] = 100000;

		stage->setGoal(r);
		stage->setGroup("robotiq_2f_140");
		t.add(std::move(stage));
	}
	{  // Allow collisions
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Allow Collisions");
		stage->allowCollisions("support_tool_rack", "stylus", true);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
	{  // Allow collisions
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Allow Collisions");
		stage->allowCollisions("stylus", "stylus", true);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
	{  // Allow collisions
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Allow Collisions");
		stage->allowCollisions(
		    "stylus",
		    task_->getRobotModel()->getJointModelGroup("robotiq_2f_140")->getLinkModelNamesWithCollisionGeometry(), true);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}
	{  // Attach object
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Attach object to Gripper");
		stage->attachObjects(std::vector<std::string>{ "stylus" }, "robotiq_2f_140_tcp", true);
		current_state_ = stage.get();
		t.add(std::move(stage));
	}
	{  // Move relative up wrt object insertio
		geometry_msgs::Vector3Stamped direction = geometry_msgs::Vector3Stamped();
		direction.header.frame_id = "stylus";
		direction.vector.x = -0.070390000000000008;

		auto stage = std::make_unique<stages::MoveRelative>("Move relative", cartesian_planner_);

		stage->setDirection(direction);
		stage->setMinMaxDistance(-1, 0);
		stage->properties().set("link", "stylus");
		stage->setGroup("left_eef");
		stage->setPoseTransform(createIsomtery3d(0, 0, 0, 0, 0.707107, 0, -0.707107));
		{  // Allow collisions
			auto stage = std::make_unique<stages::ModifyPlanningScene>("Allow Collisions");
			stage->allowCollisions("support_tool_rack", "stylus", true);
			stage->properties().configureInitFrom(Stage::PARENT);
			t.add(std::move(stage));
		}
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
		    "Connect2", stages::Connect::GroupPlannerVector{ { "left_eef", sampling_planner_ } });
		stage->setTimeout(timeout_connect_);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{  // GenerateInsertionPose this stage is identical to the generateGraspPose except that it dosent need a hand_state
		std::string eef_name = "gripper";
		std::string target_frame_name = "transporator";
		std::string tcp_frame_name = "stylus";
		std::string arm_group_name = "left_eef";

		double target_angle_delta = 0.78539816339744828;
		Eigen::Vector3d target_axis = Eigen::Vector3d(1, 0, 0);

		Eigen::Isometry3d tcp_frame = createIsomtery3d(0, 0, 0, 0, 0.707107, 0, -0.707107);
		Eigen::Isometry3d target_frame = createIsomtery3d(0.06074, -0.17265, 0.05685, 0.5, -0.5, -0.5, 0.5);

		auto stage = std::make_unique<stages::GenerateInsertPose>("Generate Insert Pose");
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

	{  // 2- Allow collisions: tool_attached_objects -> dst_object
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Allow Collision");
		stage->allowCollisions("stylus", "transporator", true);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{  // Allow collisions
		auto stage = std::make_unique<stages::ModifyPlanningScene>("Allow Collision");
		stage->allowCollisions(
		    "transporator",
		    task_->getRobotModel()->getJointModelGroup("robotiq_2f_140")->getLinkModelNamesWithCollisionGeometry(), true);
		stage->properties().configureInitFrom(Stage::PARENT);
		t.add(std::move(stage));
	}

	{  // Move relative Push button
		geometry_msgs::Vector3Stamped direction = geometry_msgs::Vector3Stamped();
		direction.header.frame_id = "stylus";
		direction.vector.x = 0.017999999999999999;
		Eigen::Isometry3d tcp_frame = createIsomtery3d(0, 0, 0, 0, 0.707107, 0, -0.707107);

		auto stage = std::make_unique<stages::MoveRelative>("Move relative", cartesian_planner_);
		stage->setDirection(direction);
		stage->setMinMaxDistance(-1, 0);
		stage->properties().set("link", "stylus");
		stage->setGroup("left_eef");
		stage->setPoseTransform(tcp_frame);
		stage->setTimeout(timeout_);
		t.add(std::move(stage));
	}

	{  // Move relative Retreat
		geometry_msgs::Vector3Stamped direction = geometry_msgs::Vector3Stamped();
		direction.header.frame_id = "stylus";
		direction.vector.x = -0.017999999999999999;
		Eigen::Isometry3d tcp_frame = createIsomtery3d(0, 0, 0, 0, 0.707107, 0, -0.707107);

		auto stage = std::make_unique<stages::MoveRelative>("Move relative", cartesian_planner_);
		stage->setDirection(direction);
		stage->setMinMaxDistance(-1, 0);
		stage->properties().set("link", "stylus");
		stage->setGroup("left_eef");
		stage->setPoseTransform(tcp_frame);
		stage->setTimeout(timeout_);
		t.add(std::move(stage));
	}

	/*
	   {  // Frobid collisions
	      auto stage = std::make_unique<stages::ModifyPlanningScene>(name);
	      stage->allowCollisions(
	          first_, task_->getRobotModel()->getJointModelGroup(second_)->getLinkModelNamesWithCollisionGeometry(),
	   flag); stage->properties().configureInitFrom(Stage::PARENT); t.add(std::move(stage));
	   }
	*/
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
