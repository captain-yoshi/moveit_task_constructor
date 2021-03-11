/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017-2018, Hamburg University
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
 *   * Neither the name of Hamburg University nor the names of its
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
/* Authors: Michael Goerner, Henning Kayser
   Desc:    Pour from attached bottle into(onto) an object
*/

#include <moveit/task_constructor/stages/spread_over.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
//#if MOVEIT_CARTESIAN_INTERPOLATOR
#include <moveit/robot_state/cartesian_interpolator.h>
//#endif

#include <moveit/trajectory_processing/iterative_spline_parameterization.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <eigen_conversions/eigen_msg.h>

#include <geometric_shapes/shape_extents.h>

#include <shape_msgs/SolidPrimitive.h>

#include <eigen_conversions/eigen_msg.h>

#include <rviz_marker_tools/marker_creation.h>

#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>

namespace {
/** Compute prototype waypoints for pouring.
 * This generates a trajectory that pours in the Y-Z axis.
 * The generated poses are for the bottle-tip and are relative to the container top-center */
void computePouringWaypoints(const Eigen::Isometry3d& start_tip_pose, double tilt_angle,
                             const Eigen::Translation3d& pouring_offset, EigenSTL::vector_Isometry3d& waypoints,
                             unsigned long nr_of_waypoints = 10) {
	/*
	   Eigen::Isometry3d start_tip_rotation(start_tip_pose);
	   start_tip_rotation.translation().fill(0);

	   waypoints.push_back(start_tip_pose);

	   for (unsigned int i = 1; i <= nr_of_waypoints; ++i) {
	      const double fraction = static_cast<double>(i) / nr_of_waypoints;
	      const double exp_fraction = fraction * fraction;
	      const double offset_fraction =
	          std::pow(fraction, 1.0 / 5.0) +
	          (-4.5 * fraction * fraction + 4.5 * fraction);  // custom trajectory translating away from cup center

	      // linear interpolation for tilt angle
	      Eigen::Isometry3d rotation =
	          Eigen::AngleAxisd(fraction * tilt_angle, Eigen::Vector3d::UnitX()) * start_tip_rotation;

	      // exponential interpolation towards container rim + offset
	      Eigen::Translation3d translation(start_tip_pose.translation() * (1 - exp_fraction) +
	                                       pouring_offset.translation() * exp_fraction);

	      translation.y() = start_tip_pose.translation().y() * (1 - offset_fraction) +
	                        pouring_offset.translation().y() * (offset_fraction);

	      waypoints.push_back(translation * rotation);
	   }
	*/
}

} /* anonymous namespace */

namespace moveit {
namespace task_constructor {
namespace stages {

SpreadOver::SpreadOver(std::string name) : PropagatingForward(std::move(name)) {
	auto& p = properties();
	p.declare<std::string>("group", "name of planning group");

	p.declare<std::string>("bottle", "attached bottle-like object");
	p.declare<std::string>("container", "container object to be filled");

	p.declare<double>("tilt_angle", "maximum tilt-angle for the bottle");
	p.declare<double>("min_path_fraction", 0.9, "minimum valid fraction of the planned pouring path");
	p.declare<size_t>("waypoint_count", 10, "Number of Cartesian waypoints to approximate pouring trajectory");
	p.declare<Eigen::Vector3d>("pour_offset", "offset for the bottle tip w.r.t. container top-center during pouring");
	p.declare<geometry_msgs::Vector3Stamped>("pouring_axis", "Axis around which to pour");
	p.declare<ros::Duration>("pour_duration", ros::Duration(1.0), "duration to stay in pouring pose");
	p.declare<ros::Duration>("waypoint_duration", ros::Duration(0.5), "duration between pouring waypoints");

	p.declare<std::string>("object_name", "duration to stay in pouring pose");
	p.declare<Eigen::Isometry3d>("object_pose", Eigen::Isometry3d(), "pose transform list wrt object mesh origin");
	p.declare<EigenSTL::vector_Isometry3d>("waypoints", EigenSTL::vector_Isometry3d(),
	                                       "duration between pouring waypoints");
}

/* MTC stage interface */
void SpreadOver::computeForward(const InterfaceState& from) {
	planning_scene::PlanningScenePtr to;
	SubTrajectory trajectory;

	compute(from, to, trajectory);
	sendForward(from, InterfaceState(to), std::move(trajectory));
	return;
}

void SpreadOver::compute(const InterfaceState& input, planning_scene::PlanningScenePtr& result,
                         SubTrajectory& trajectory) {
	// NOTE new api
	{
		const auto& props = properties();
		const auto& min_path_fraction = props.get<double>("min_path_fraction");
		const ros::Duration waypoint_duration(props.get<ros::Duration>("waypoint_duration"));

		// object MUST be attached
		const std::string& object_name = props.get<std::string>("object_name");
		const Eigen::Isometry3d& frame_object_tip = props.get<Eigen::Isometry3d>("object_pose");

		// wrt objecttip
		EigenSTL::vector_Isometry3d waypoints = props.get<EigenSTL::vector_Isometry3d>("waypoints");

		// Get planning scene, robot model, joint model group and robot state
		const planning_scene::PlanningScene& scene = *input.scene();
		moveit::core::RobotModelConstPtr robot_model = scene.getRobotModel();
		const moveit::core::JointModelGroup* group = robot_model->getJointModelGroup(props.get<std::string>("group"));
		moveit::core::RobotState state(scene.getCurrentState());

		moveit_msgs::AttachedCollisionObject spreader;
		if (!scene.getAttachedCollisionObjectMsg(spreader, object_name))
			throw std::runtime_error("bottle '" + object_name +
			                         "' is not an attached collision object in input planning scene");

		Eigen::Isometry3d frame_base_object = scene.getFrameTransform(object_name);
		Eigen::Isometry3d frame_base_tip = frame_base_object * frame_object_tip;

		// transform waypoints to planning frame
		for (auto& waypoint : waypoints)
			waypoint = frame_base_tip * waypoint;

		for (auto waypoint : waypoints) {
			geometry_msgs::PoseStamped p;
			p.header.frame_id = scene.getPlanningFrame();
			tf::poseEigenToMsg(waypoint, p.pose);

			rviz_marker_tools::appendFrame(trajectory.markers(), p, 0.1, markerNS());
		}

		/* specify waypoints for tool link, not for bottle tip */
		auto& attached_bottle_tfs = state.getAttachedBody(object_name)->getFixedTransforms();

		const Eigen::Isometry3d bottle_tip_in_tool_link(attached_bottle_tfs[0] * frame_object_tip);
		for (auto& waypoint : waypoints) {
			// waypoint = waypoint * frame_base_tip.inverse();
			waypoint = waypoint * bottle_tip_in_tool_link.inverse();
		}
		std::vector<moveit::core::RobotStatePtr> traj;

		// TODO: this has to use computeCartesianPath because
		// there is currently no multi-waypoint callback in cartesian_planner
		//#if MOVEIT_CARTESIAN_INTERPOLATOR
		double path_fraction = moveit::core::CartesianInterpolator::computeCartesianPath(
		    &state, group, traj, state.getLinkModel(spreader.link_name), waypoints, true /* global reference_frame */,
		    moveit::core::MaxEEFStep(.03) /* max step size */, moveit::core::JumpThreshold(2.0) /* jump threshold */,
		    [&scene](moveit::core::RobotState* rs, const moveit::core::JointModelGroup* jmg,
		             const double* joint_positions) {
			    rs->setJointGroupPositions(jmg, joint_positions);
			    rs->update();
			    return !scene.isStateColliding(*rs, jmg->getName());
		    });
		//#else
		/*
		      ROS_DEBUG_STREAM("Link name = " << state.getLinkModel(spreader.link_name)->getName());

		      double path_fraction = state.computeCartesianPath(
		          group, traj, state.getLinkModel(spreader.link_name), waypoints, true ,
		          .01 , 1.5 ,
		          [&scene](moveit::core::RobotState* rs, const moveit::core::JointModelGroup* jmg,
		                   const double* joint_positions) {
		         rs->setJointGroupPositions(jmg, joint_positions);
		         rs->update();
		         return !scene.isStateColliding(*rs, jmg->getName());
		          });
		* /
		    //#endif

		    /* build executable RobotTrajectory (downward and back up) */

		auto robot_trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group);

		for (const auto& waypoint : traj) {
			robot_trajectory->addSuffixWayPoint(waypoint, 0);
		}

		/* generate time parameterization */
		// TODO: revert code and interfaces to ISP / needs testing on hardware
		trajectory_processing::IterativeSplineParameterization isp;
		const double velocity_scaling = 0.2;
		const double acceleration_scaling = 1.0;

		bool tp_rc = false;
		{
			trajectory_processing::IterativeParabolicTimeParameterization iptp;
			tp_rc = isp.computeTimeStamps(*robot_trajectory, velocity_scaling, acceleration_scaling);
		}

		trajectory.setTrajectory(robot_trajectory);

		result = scene.diff();
		result->setCurrentState(robot_trajectory->getLastWayPoint());
		if (path_fraction < 1.0) {
			ROS_WARN_STREAM("SpreadOver only produced motion for " << path_fraction << " of the way. Rendering invalid");
			trajectory.setCost(std::numeric_limits<double>::infinity());
			trajectory.setComment("computed path_fraction = " + std::to_string(path_fraction));
			return;
		}
		auto waypoint_count = robot_trajectory->getWayPointCount();
		for (int i = 0; i < waypoint_count; ++i) {
			double duration = robot_trajectory->getWaypointDurationFromStart(i);
			ROS_DEBUG_STREAM("Waypoint duration from start = " << std::to_string(duration));
		}
		if (!tp_rc) {
			ROS_WARN_STREAM("IPTP error");
			trajectory.setCost(std::numeric_limits<double>::infinity());
			trajectory.setComment("IPTP time parametrization failed");
			return;
		}
	}
	return;
}

}  // namespace stages
}  // namespace task_constructor
}  // namespace moveit
