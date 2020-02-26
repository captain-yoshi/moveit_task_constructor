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

#include <moveit/task_constructor/stages/generate_grasp_prism_pose.h>
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

            GenerateGraspPrismPose::GenerateGraspPrismPose(const std::string& name) : GeneratePose(name) {
                auto& p = properties();
                p.declare<std::string>("eef", "name of end-effector");
                p.declare<std::string>("object");


                p.declare<double>("angle_shape_delta", "angular steps (rad)");
                p.declare<Eigen::Vector3d>("axis_shape", "axis for angular steps wrt object or object_pose_transform");

                p.declare<double>("angle_delta", "angular steps (rad)");
                p.declare<Eigen::Vector3d>("axis", "axis for angular steps wrt object or object_pose_transform");

                p.declare<Eigen::Isometry3d>("object_pose_transform", "pose transform wrt to object mesh origin");

                p.declare<double>("gap", "gap width");
                p.declare<double>("length", "length");
                p.declare<double>("height", "height ");

                p.declare<double>("length_delta", "translate steps (m)");
                p.declare<double>("height_delta", "translate steps (m)");

                p.declare<boost::any>("pregrasp", "pregrasp posture");
                p.declare<boost::any>("grasp", "grasp posture");
            }

            void GenerateGraspPrismPose::init(const core::RobotModelConstPtr& robot_model) {
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

                if (props.get<double>("angle_shape_delta") == 0.)
                    errors.push_back(*this, "angle_shape_delta must be non-zero");

                if (props.get<double>("length_delta") == 0.)
                    errors.push_back(*this, "length_delta must be non-zero");

                if (props.get<double>("height_delta") == 0.)
                    errors.push_back(*this, "height_delta must be non-zero");

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

            void GenerateGraspPrismPose::onNewSolution(const SolutionBase& s) {
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

            void GenerateGraspPrismPose::compute() {
                if (upstream_solutions_.empty())
                    return;
                planning_scene::PlanningScenePtr scene = upstream_solutions_.pop()->end()->scene()->diff();

                // set end effector pose
                const auto& props = properties();
                const std::string& eef = props.get<std::string>("eef");
                const moveit::core::JointModelGroup* jmg = scene->getRobotModel()->getEndEffector(eef);

                robot_state::RobotState& robot_state = scene->getCurrentStateNonConst();
                moveit_msgs::RobotState robot_state_msg = props.get<moveit_msgs::RobotState>("pregrasp");
                robotStateMsgToRobotState(robot_state_msg, robot_state);


                geometry_msgs::PoseStamped target_pose_msg;
                target_pose_msg.header.frame_id = props.get<std::string>("object");

                Eigen::Vector3d axis_shape = props.get<Eigen::Vector3d>("axis_shape");
                Eigen::Vector3d axis = props.get<Eigen::Vector3d>("axis");
                Eigen::Isometry3d object_pose_transform = props.get<Eigen::Isometry3d>("object_pose_transform");


                double length = props.get<double>("length");
                double height = props.get<double>("height");

                double length_delta = props.get<double>("length_delta");
                double height_delta = props.get<double>("height_delta");



                // Translation length
                //double current_length_ = -length/2.0;
                double current_length_ = 0.0;
                while (current_length_ < length/2.0) {

                    Eigen::Isometry3d temp_pose(object_pose_transform);
                    temp_pose.pretranslate(Eigen::Vector3d(current_length_, 0, 0));
                    current_length_ += length_delta;

                    // Translation height
                    //double current_height_ = -height / 2.0;
		    double current_height_ = 0.0;
                    while (current_height_ < height / 2.0) {

                        temp_pose.translate(Eigen::Vector3d(0, current_height_, 0));
                        current_height_ += height_delta;

                        // ROtate along prism
                        double current_angle_ = 0.0;
                        while (current_angle_ < 2. * M_PI && current_angle_ > -2. * M_PI) {

                            // TODO add translation first


                            temp_pose = temp_pose * Eigen::AngleAxisd(current_angle_, axis);
                            current_angle_ += props.get<double>("angle_delta");

                            // Try grasping each of pair of prism sides
                            double current_shape_angle_ = 0.0;
                            while (current_shape_angle_ < 2. * M_PI && current_shape_angle_ > -2. * M_PI) {
                                // rotate object pose about z-axis
                                Eigen::Isometry3d target_pose(
                                        temp_pose * Eigen::AngleAxisd(current_shape_angle_, axis_shape));
                                current_shape_angle_ += props.get<double>("angle_shape_delta");

                                InterfaceState state(scene);
                                tf::poseEigenToMsg(target_pose, target_pose_msg.pose);
                                state.properties().set("target_pose", target_pose_msg);
                                props.exposeTo(state.properties(), {"pregrasp", "grasp"});

                                SubTrajectory trajectory;
                                trajectory.setCost(0.0);
                                trajectory.setComment(std::to_string(current_shape_angle_));

                                // add frame at target pose
                                rviz_marker_tools::appendFrame(trajectory.markers(), target_pose_msg, 0.1,
                                                               "grasp frame");

                                spawn(std::move(state), std::move(trajectory));
                            }
                        }
                    }
                }
            }
        }
    }
}
