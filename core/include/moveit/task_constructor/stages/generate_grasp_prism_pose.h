/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Hamburg University
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

/* Authors: Michael Goerner
   Desc:    Generator Stage for simple grasp poses
*/

#pragma once

#include <moveit/task_constructor/stages/generate_pose.h>
#include <Eigen/Geometry>

namespace moveit {
namespace task_constructor {
namespace stages {

class GenerateGraspPrismPose : public GeneratePose
{
public:
	GenerateGraspPrismPose(const std::string& name = "generate grasp prism pose");

	void init(const core::RobotModelConstPtr& robot_model) override;
	void compute() override;

	void setEndEffector(const std::string& eef) { setProperty("eef", eef); }
	void setObject(const std::string& object) { setProperty("object", object); }
	void setObjectPoseTransform(const Eigen::Isometry3d& object_pose) {
		setProperty("object_pose_transform", object_pose);
	}

	void setAngleDelta(double delta) { setProperty("angle_delta", delta); }
	void setAxis(const Eigen::Vector3d& axis) { setProperty("axis", axis); }
	void setAngleShapeDelta(double delta) { setProperty("angle_shape_delta", delta); }
	void setAxisShape(const Eigen::Vector3d& axis) { setProperty("axis_shape", axis); }

	void setGap(double gap) { setProperty("gap", gap); }
	void setLength(double length) { setProperty("length", length); }
	void setHeight(double height) { setProperty("height", height); }

	void setLengthSize(double length) { setProperty("length_max", length); }
	void setHeightSize(double height) { setProperty("height_max", height); }

	void setLengthDelta(double length) { setProperty("length_delta", length); }
	void setHeightDelta(double height) { setProperty("height_delta", height); }

	void setPreGraspPose(const moveit_msgs::RobotState& pregrasp) { properties().set("pregrasp", pregrasp); }
	void setGraspPose(const std::string& grasp) { properties().set("grasp", grasp); }
	void setGraspPose(const moveit_msgs::RobotState& grasp) { properties().set("grasp", grasp); }

protected:
	void onNewSolution(const SolutionBase& s) override;
};
}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
