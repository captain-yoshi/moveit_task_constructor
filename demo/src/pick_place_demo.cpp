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

// ROS
#include <ros/ros.h>

// MTC pick/place demo implementation
#include <moveit_task_constructor_demo/pick_place_task.h>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <tf2_ros/transform_broadcaster.h>

//#include <moveit_msgs/CollisionObject.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <eigen_conversions/eigen_msg.h>
//#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
//#include <geometric_shapes/shape_extents.h>
constexpr char LOGNAME[] = "moveit_task_constructor_demo";

Eigen::Isometry3d createIsometry(double x, double y, double z, double qw, double qx, double qy, double qz) {
	Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
	tf.translate(Eigen::Vector3d(x, y, z));

	Eigen::Quaterniond quat = Eigen::Quaterniond(qw, qx, qy, qz);
	tf.rotate(quat);

	return tf;
}

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi, const moveit_msgs::CollisionObject& object) {
	if (!psi.applyCollisionObject(object))
		throw std::runtime_error("Failed to spawn object: " + object.id);
}

moveit_msgs::CollisionObject createTable() {
	ros::NodeHandle pnh("~");
	std::string table_name, table_reference_frame;
	std::vector<double> table_dimensions;
	geometry_msgs::Pose pose;
	std::size_t errors = 0;
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_name", table_name);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_reference_frame", table_reference_frame);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_dimensions", table_dimensions);
	errors += !rosparam_shortcuts::get(LOGNAME, pnh, "table_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, errors);

	moveit_msgs::CollisionObject object;
	object.id = table_name;
	object.header.frame_id = table_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	object.primitives[0].dimensions = table_dimensions;
	pose.position.z -= 0.5 * table_dimensions[2];  // align surface with world
	object.primitive_poses.push_back(pose);
	return object;
}

moveit_msgs::CollisionObject createObject() {
	ros::NodeHandle pnh("~");
	std::string object_name, object_reference_frame;
	std::vector<double> object_dimensions;
	geometry_msgs::Pose pose;
	std::size_t error = 0;
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_name", object_name);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_reference_frame", object_reference_frame);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_dimensions", object_dimensions);
	error += !rosparam_shortcuts::get(LOGNAME, pnh, "object_pose", pose);
	rosparam_shortcuts::shutdownIfError(LOGNAME, error);

	moveit_msgs::CollisionObject object;
	object.id = object_name;
	object.header.frame_id = object_reference_frame;
	object.primitives.resize(1);
	object.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	object.primitives[0].dimensions = object_dimensions;
	pose.position.z += 0.5 * object_dimensions[0];
	object.primitive_poses.push_back(pose);
	return object;
}
void addCollisionObject(std::vector<moveit_msgs::CollisionObject>& object, const std::string& object_id,
                        const std::string& object_mesh, Eigen::Isometry3d pose, const std::string& frame_id) {
	geometry_msgs::PoseStamped obj_ps;
	tf::poseEigenToMsg(pose, obj_ps.pose);
	obj_ps.header.frame_id = frame_id;
	object.emplace_back();

	object.back().meshes.resize(1);

	// load mesh
	const Eigen::Vector3d scaling(1, 1, 1);
	shapes::Shape* shape = shapes::createMeshFromResource(object_mesh, scaling);
	shapes::ShapeMsg shape_msg;
	shapes::constructMsgFromShape(shape, shape_msg);
	object.back().meshes[0] = boost::get<shape_msgs::Mesh>(shape_msg);

	// set pose
	object.back().mesh_poses.resize(1);
	object.back().mesh_poses[0].orientation.w = 1.0;

	// fill in details for MoveIt
	object.back().id = object_id;
	object.back().operation = moveit_msgs::CollisionObject::ADD;
	object.back().header = obj_ps.header;
	object.back().mesh_poses[0] = obj_ps.pose;

	// The input pose is interpreted as a point *on* the table
	object.back().mesh_poses[0].position.z += .00001;
}

int main(int argc, char** argv) {
	ROS_INFO_NAMED(LOGNAME, "Init moveit_task_constructor_demo");
	ros::init(argc, argv, "moveit_task_constructor_demo");
	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Add table and object to planning scene
	ros::Duration(1.0).sleep();  // Wait for ApplyPlanningScene service
	moveit::planning_interface::PlanningSceneInterface psi;
	ros::NodeHandle pnh("~");

	std::vector<moveit_msgs::CollisionObject> obj;
	// addCollisionObject(obj, it->getId(), it->getMeshPath(), frame->pose, frame->id);

	addCollisionObject(obj, "stylus", "package://bob_description/meshes/udes/collision/stylus.stl",
	                   createIsometry(0.353954, -0.245277, 0.0114673, 0.707206, -0.00032488, -0.00485013, 0.706991),
	                   "optical_table");
	addCollisionObject(obj, "support_tool_rack", "package://bob_description/meshes/udes/collision/tool_rack.stl",
	                   createIsometry(0.354018, -0.267016, 0.00171818, 0.999988, 0.00319984, -0.00365929, -0.000151627),
	                   "optical_table");
	addCollisionObject(obj, "transporator", "package://bob_description/meshes/btx/collision/transporator.stl",
	                   createIsometry(-0.45, 0.16, 0, 0.999974, 0, 0, 0.00724553), "optical_table");

	addCollisionObject(obj, "optical_table", "package://bob_description/meshes/udes/collision/dummy.stl",
	                   createIsometry(0, 0, 0, 1, 0, 0, 0), "optical_table");

	moveit_msgs::AttachedCollisionObject attached_object;
	attached_object.link_name = "optical_table";
	attached_object.object = obj.back();
	psi.applyAttachedCollisionObject(attached_object);

	addCollisionObject(obj, "udes_pipette", "package://bob_description/meshes/udes/collision/dummy.stl",
	                   createIsometry(0, 0, 0, 1, 0, 0, 0), "pipette_tool_tcp");
	attached_object.link_name = "pipette_tool_tcp";
	attached_object.object = obj.back();
	psi.applyAttachedCollisionObject(attached_object);

	addCollisionObject(obj, "robotiq_2f_140", "package://bob_description/meshes/udes/collision/dummy.stl",
	                   createIsometry(0, 0, 0, 1, 0, 0, 0), "robotiq_2f_140_tcp");
	attached_object.link_name = "robotiq_2f_140_tcp";
	attached_object.object = obj.back();
	psi.applyAttachedCollisionObject(attached_object);
	// addCollisionObject(obj, "", "", createIsometry(, , , , , , ), "");

	psi.applyCollisionObjects(obj);

	/*
	   if (pnh.param("spawn_table", true))
	      spawnObject(psi, createTable());
	   spawnObject(psi, createObject());
	*/
	// Construct and run pick/place task
	moveit_task_constructor_demo::PickPlaceTask pick_place_task("pick_place_task", nh);
	pick_place_task.loadParameters();
	pick_place_task.init();
	if (pick_place_task.plan()) {
		ROS_INFO_NAMED(LOGNAME, "Planning succeded");
		if (pnh.param("execute", false)) {
			pick_place_task.execute();
			ROS_INFO_NAMED(LOGNAME, "Execution complete");
		} else {
			ROS_INFO_NAMED(LOGNAME, "Execution disabled");
		}
	} else {
		ROS_INFO_NAMED(LOGNAME, "Planning failed");
	}

	// Keep introspection alive
	ros::waitForShutdown();
	return 0;
}
