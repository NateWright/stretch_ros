/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, SRI International
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
 *   * Neither the name of SRI International nor the names of its
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

/* Author: Sachin Chitta, Dave Coleman, Mike Lautman */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const std::string PLANNING_GROUP = "stretch_arm";

moveit::planning_interface::MoveGroupInterface *move_group_interface;

void stretchCallback(const geometry_msgs::Pose target_pose1){
  // Start spinner to be able to access Current position
  ros::AsyncSpinner s(1);
  s.start();
  // Get current pose
  //geometry_msgs::PoseStamped ps = move_group_interface->getCurrentPose();
  //geometry_msgs::Pose target_pose2 = ps.pose;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // Debug Print
  //ROS_INFO("1: %f, %f, %f", target_pose1.position.x, target_pose1.position.y, target_pose1.position.z);
  //ROS_INFO("2b: %f, %f, %f, %f", target_pose2.orientation.x, target_pose2.orientation.y, target_pose2.orientation.z, target_pose2.orientation.w);

  //target_pose2.position.z = target_pose2.position.z + target_pose1.position.z;
  //target_pose2.position.y = target_pose2.position.y - target_pose1.position.y;
  //target_pose2.orientation = target_pose1.orientation;

  //ROS_INFO("2a: %f, %f, %f, %f", target_pose2.orientation.x, target_pose2.orientation.y, target_pose2.orientation.z, target_pose2.orientation.w);

  // Setting pose
  move_group_interface->setPoseTarget(target_pose1);
  // Planning
  bool success = (move_group_interface->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  // Executing
  move_group_interface->execute(my_plan);
  s.stop();
}

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroupInterface m(PLANNING_GROUP);
  move_group_interface = &m;
  move_group_interface->getCurrentPose();
  move_group_interface->setGoalTolerance(0.01);
  move_group_interface->setPlanningTime(20.0);
  ros::Subscriber sub = node_handle.subscribe("moveboi", 1000, stretchCallback);
  ros::waitForShutdown();
  return 0;
}
