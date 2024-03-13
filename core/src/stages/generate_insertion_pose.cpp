/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Sherbrooke University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Authors: Captain Yoshi */

#include <moveit/task_constructor/stages/generate_insertion_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

namespace moveit {
namespace task_constructor {
namespace stages {

GenerateInsertionPose::GenerateInsertionPose(const std::string& name) : GeneratePose(name) {
	auto& p = properties();
	p.declare<std::string>("eef", "name of end-effector");
	p.declare<std::string>("object");
	p.declare<geometry_msgs::Pose>("object_offset", "frame offset wrt. the object");
	p.declare<double>("angle_delta", 0.1, "angular steps (rad)");
	p.declare<Eigen::Vector3d>("rotation_axis", Eigen::Vector3d::UnitZ(), "rotate object pose about given axis");

	p.set("object_offset", [] {
		geometry_msgs::Pose pose;
		pose.orientation.w = 1.0;
		return pose;
	}());
}

void GenerateInsertionPose::setObjectOffset(const Eigen::Isometry3d& object_offset) {
	auto object_offset_msg = tf2::toMsg(object_offset);
	setObjectOffset(object_offset_msg);
}

void GenerateInsertionPose::init(const core::RobotModelConstPtr& robot_model) {
	InitStageException errors;
	try {
		GeneratePose::init(robot_model);
	} catch (InitStageException& e) {
		errors.append(e);
	}

	const auto& props = properties();

	// check angle_delta
	if (props.get<double>("angle_delta") == 0.)
		errors.push_back(*this, "angle_delta must be non-zero");

	// check availability of object
	props.get<std::string>("object");
	// check availability of eef
	const std::string& eef = props.get<std::string>("eef");
	if (!robot_model->hasEndEffector(eef)) {
		errors.push_back(*this, "unknown end effector: " + eef);
		throw errors;
	}

	if (errors)
		throw errors;
}

void GenerateInsertionPose::onNewSolution(const SolutionBase& s) {
	planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

	const auto& props = properties();
	const std::string& object = props.get<std::string>("object");
	if (!scene->knowsFrameTransform(object)) {
		const std::string msg = "object '" + object + "' not in scene";
		spawn(InterfaceState{ scene }, SubTrajectory::failure(msg));
		return;
	}

	upstream_solutions_.push(&s);
}

void GenerateInsertionPose::compute() {
	if (upstream_solutions_.empty())
		return;
	planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

	// set end effector pose
	const auto& props = properties();
	const std::string& eef = props.get<std::string>("eef");
	const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

	moveit::core::RobotState& robot_state = scene->getCurrentStateNonConst();

	geometry_msgs::PoseStamped target_pose_msg;
	target_pose_msg.header.frame_id = props.get<std::string>("object");

	geometry_msgs::Pose object_offset_msg = props.get<geometry_msgs::Pose>("object_offset");
	Eigen::Vector3d rotation_axis = props.get<Eigen::Vector3d>("rotation_axis");

	Eigen::Isometry3d object_offset;
	tf2::fromMsg(object_offset_msg, object_offset);

	double current_angle = 0.0;
	while (current_angle < 2. * M_PI && current_angle > -2. * M_PI) {
		// rotate object pose about z-axis
		Eigen::Isometry3d target_pose(Eigen::AngleAxisd(current_angle, rotation_axis));
		target_pose = object_offset * target_pose;

		current_angle += props.get<double>("angle_delta");

		InterfaceState state(scene);
		target_pose_msg.pose = tf2::toMsg(target_pose);
		state.properties().set("target_pose", target_pose_msg);
		// props.exposeTo(state.properties(), { "pregrasp", "grasp" });

		SubTrajectory trajectory;
		trajectory.setCost(0.0);
		trajectory.setComment(std::to_string(current_angle));

		// add frame at target pose
		rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

		spawn(std::move(state), std::move(trajectory));
	}
}
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
