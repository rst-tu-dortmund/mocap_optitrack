/*
 *      _____
 *     /  _  \
 *    / _/ \  \
 *   / / \_/   \
 *  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
 *  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
 *   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
 *    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
 *     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
 *             ROBOTICS™ 
 *
 *  File: mocap_config.h
 *  Desc: Classes representing ROS configuration for mocap_optitrack node. Data
 *  will be published to differed topics based on the configuration provided.
 *  Auth: Alex Bencz
 *
 *  Copyright (c) 2012, Clearpath Robotics, Inc. 
 *  All Rights Reserved
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Please send comments, questions, or patches to skynet@clearpathrobotics.com 
 *
 */

#ifndef __MOCAP_CONFIG_H__
#define __MOCAP_CONFIG_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "mocap_datapackets.h"

#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_listener.h>

class footprint
{
  public:
    footprint(tf::Transform tf);
    
    tf::Transform transform;
    geometry_msgs::PolygonStamped footprint_base_link;
    geometry_msgs::PolygonStamped footprint_world;
    geometry_msgs::PolygonStamped transform_footprint(geometry_msgs::PolygonStamped polygon);
};

class PublishedRigidBody
{
  private:
  ros::NodeHandle n;

  std::string pose_topic;
  std::string pose2d_topic;
  std::string parent_frame_id;
  std::string child_frame_id;

  std::string marker_topic;
  std::string odom_topic;
  std::string footprint_topic;
 
  bool publish_pose;
  bool publish_tf;
  bool publish_pose2d;

  bool publish_marker;
  bool publish_odom;
  bool publish_footprint;

  tf::TransformBroadcaster tf_pub;
  ros::Publisher pose_pub;
  ros::Publisher pose2d_pub;

  ros::Publisher marker_pub;
  ros::Publisher odom_pub;
  ros::Publisher footprint_pub;

  bool validateParam(XmlRpc::XmlRpcValue &, const std::string &);

  public:
  PublishedRigidBody(XmlRpc::XmlRpcValue &);
  void publish(RigidBody &, RigidBodyOdomHelper& odom_helper);
};

typedef std::map<string, PublishedRigidBody> RigidBodyMap;
typedef std::pair<string, PublishedRigidBody> RigidBodyItem;

#endif  // __MOCAP_CONFIG_H__
