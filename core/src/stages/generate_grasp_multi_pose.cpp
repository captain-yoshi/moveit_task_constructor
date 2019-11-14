/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Bielefeld + Hamburg University
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

/* Authors: Robert Haschke, Michael Goerner */

#include <moveit/task_constructor/stages/generate_grasp_multi_pose.h>
#include <moveit/task_constructor/storage.h>
#include <moveit/task_constructor/marker_tools.h>
#include <rviz_marker_tools/marker_creation.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

namespace moveit {
    namespace task_constructor {
        namespace stages {

            GenerateGraspMultiPose::GenerateGraspMultiPose(const std::string& name) : GeneratePose(name) {
                auto& p = properties();
                p.declare<std::string>("eef", "name of end-effector");
                p.declare<std::string>("object");
                p.declare<std::vector<double>>("angle_delta", "angular steps (rad)");
                p.declare<std::vector<Eigen::Vector3d>>("axis", "axis for angular steps wrt object or object_pose_transform");

                p.declare<std::vector<Eigen::Isometry3d>>("object_pose_transform", "pose transform wrt to object mesh origin");


                p.declare<boost::any>("pregrasp", "pregrasp posture");
                p.declare<boost::any>("grasp", "grasp posture");
            }

            void GenerateGraspMultiPose::init(const core::RobotModelConstPtr& robot_model) {
                InitStageException errors;
                try {
                    GeneratePose::init(robot_model);
                } catch (InitStageException& e) {
                    errors.append(e);
                }

                const auto& props = properties();

                // check angle_delta
                std::vector<double>  angle_list = props.get<std::vector<double>>("angle_delta");
                for (auto it=angle_list.begin(); it!=angle_list.end(); ++it) {
                    if (*it == 0.)
                        errors.push_back(*this, "angle_delta must be non-zero");
                }

                // check availability of object
                props.get<std::string>("object");
                // check availability of eef
                const std::string& eef = props.get<std::string>("eef");
                if (!robot_model->hasEndEffector(eef))
                    errors.push_back(*this, "unknown end effector: " + eef);
                else {
                    // check availability of eef pose
                    //const moveit::core::JointModelGroup* jmg = robot_model->getEndEffector(eef);
                    //const std::string& name = props.get<std::string>("pregrasp");
                    //std::map<std::string, double> m;
                    //if (!jmg->getVariableDefaultPositions(name, m))
                    //	errors.push_back(*this, "unknown end effector pose: " + name);
                }

                if (errors)
                    throw errors;
            }

            void GenerateGraspMultiPose::onNewSolution(const SolutionBase& s) {
                planning_scene::PlanningSceneConstPtr scene = s.end()->scene();

                const auto& props = properties();
                const std::string& object = props.get<std::string>("object");
                if (!scene->knowsFrameTransform(object)) {
                    const std::string msg = "object '" + object + "' not in scene";
                    if (storeFailures()) {
                        InterfaceState state(scene);
                        SubTrajectory solution;
                        solution.markAsFailure();
                        solution.setComment(msg);
                        spawn(std::move(state), std::move(solution));
                    } else
                        ROS_WARN_STREAM_NAMED("GenerateGraspPose", msg);
                    return;
                }

                upstream_solutions_.push(&s);
            }

            void GenerateGraspMultiPose::compute() {
                if (upstream_solutions_.empty())
                    return;

                const auto& props = properties();
                std::vector<moveit_msgs::RobotState> robot_state_list = props.get<std::vector<moveit_msgs::RobotState>>("pregrasp");
                std::vector<double> angle_list = props.get<std::vector<double>>("angle_delta");
                std::vector<Eigen::Vector3d> axis_list = props.get<std::vector<Eigen::Vector3d>>("axis");
                std::vector<Eigen::Isometry3d> transform_list = props.get<std::vector<Eigen::Isometry3d>>("object_pose_transform");

                // TODO check that each vector has the same size
                double list_size = robot_state_list.size();

                // create scene clone list
                planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

                std::vector<planning_scene::PlanningScenePtr> scene_list;
                for(int i = 0; i < list_size; i++) {
                    scene_list.push_back(scene->clone(scene));
                }

                int ctr = 0;
                for (auto scene_it=scene_list.begin(); scene_it!=scene_list.end(); ++scene_it) {


                    const std::string& eef = props.get<std::string>("eef");
                    const moveit::core::JointModelGroup* jmg = scene_it->get()->getRobotModel()->getEndEffector(eef);

                    robot_state::RobotState& robot_state = scene_it->get()->getCurrentStateNonConst();
                    robotStateMsgToRobotState(robot_state_list[ctr], robot_state);

                    geometry_msgs::PoseStamped target_pose_msg;
                    target_pose_msg.header.frame_id = props.get<std::string>("object");

                    Eigen::Isometry3d object_pose_transform = transform_list[ctr];

                    double current_angle_ = 0.0;
                    while (current_angle_ < 2. * M_PI && current_angle_ > -2. * M_PI) {
                        // rotate object pose about x-axis
                        Eigen::Isometry3d target_pose(object_pose_transform * Eigen::AngleAxisd(current_angle_, axis_list[ctr]));
                        current_angle_ += angle_list[ctr];

                        InterfaceState state(*scene_it);


                        tf::poseEigenToMsg(target_pose, target_pose_msg.pose);
                        state.properties().set("target_pose", target_pose_msg);
                        props.exposeTo(state.properties(), { "pregrasp", "grasp" });

                        SubTrajectory trajectory;
                        trajectory.setCost(0.0);
                        trajectory.setComment(std::to_string(current_angle_));

                        // add frame at target pose
                        rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1, "grasp frame");

                        spawn(std::move(state), std::move(trajectory));

                    }
                    ctr++;
                }
            }
        }
    }
}