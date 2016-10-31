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
 *  File: mocap_config.cpp
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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include "mocap_optitrack/mocap_config.h"

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>

const std::string POSE_TOPIC_PARAM_NAME = "pose";
const std::string POSE2D_TOPIC_PARAM_NAME = "pose2d";
const std::string CHILD_FRAME_ID_PARAM_NAME = "child_frame_id";
const std::string PARENT_FRAME_ID_PARAM_NAME = "parent_frame_id";

const std::string MARKER_TOPIC_PARAM_NAME = "marker";
const std::string ODOM_TOPIC_PARAM_NAME = "odom";
const std::string FOOTPRINT_TOPIC_PARAM_NAME = "footprint";



/**
 * normalize the angle
 */
inline double normalize_theta(double theta)
{
  if (theta >= -M_PI && theta < M_PI)
    return theta;

  double multiplier = floor(theta / (2*M_PI));
  theta = theta - multiplier*2*M_PI;
  if (theta >= M_PI)
    theta -= 2*M_PI;
  if (theta < -M_PI)
    theta += 2*M_PI;

  return theta;
}

template <typename T> int sign(T val) {
    return (T(0) < val) - (val < T(0));
}


footprint::footprint(tf::Transform tf)
{
  transform = tf;
}

geometry_msgs::PolygonStamped footprint:: transform_footprint(geometry_msgs::PolygonStamped polygon)
{
  footprint_base_link = polygon;
  footprint_world.header.stamp = footprint_base_link.header.stamp;
  for(int i = 0; i < footprint_base_link.polygon.points.size(); i++)
  {
    geometry_msgs::PointStamped point_base_link;
    point_base_link.point.x = footprint_base_link.polygon.points.at(i).x;
    point_base_link.point.y = footprint_base_link.polygon.points.at(i).y;
    geometry_msgs::PointStamped point_world;
    
    tf::Stamped<tf::Point> pin, pout;
    tf::pointStampedMsgToTF(point_base_link, pin);
    pout.setData(transform * pin);
    tf::pointStampedTFToMsg(pout, point_world);
    
    geometry_msgs::Point32 pt;
    pt.x = point_world.point.x;
    pt.y = point_world.point.y;
    footprint_world.polygon.points.push_back(pt);
  }
  
  return footprint_world;
}

PublishedRigidBody::PublishedRigidBody(XmlRpc::XmlRpcValue &config_node)
{
  // load configuration for this rigid body from ROS
  publish_pose = validateParam(config_node, POSE_TOPIC_PARAM_NAME);
  publish_pose2d = validateParam(config_node, POSE2D_TOPIC_PARAM_NAME);

  publish_marker = validateParam(config_node, MARKER_TOPIC_PARAM_NAME);
  publish_odom = validateParam(config_node, ODOM_TOPIC_PARAM_NAME);
  publish_footprint = validateParam(config_node, FOOTPRINT_TOPIC_PARAM_NAME);
  // only publish tf if a frame ID is provided
  publish_tf = (validateParam(config_node, CHILD_FRAME_ID_PARAM_NAME) && 
               validateParam(config_node, PARENT_FRAME_ID_PARAM_NAME));

  if (publish_pose)
  {
    pose_topic = (std::string&) config_node[POSE_TOPIC_PARAM_NAME];
    pose_pub = n.advertise<geometry_msgs::PoseStamped>(pose_topic, 1000);
  }

  if (publish_pose2d)
  {
    pose2d_topic = (std::string&) config_node[POSE2D_TOPIC_PARAM_NAME];
    pose2d_pub = n.advertise<geometry_msgs::Pose2D>(pose2d_topic, 1000);
  }

  if (publish_marker)
  {
    marker_topic = (std::string&) config_node[MARKER_TOPIC_PARAM_NAME];
    marker_pub = n.advertise<geometry_msgs::PointStamped>(marker_topic, 1000);
  }

  if (publish_odom)
  {
    odom_topic = (std::string&) config_node[ODOM_TOPIC_PARAM_NAME];
    odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1000);
  }

  if (publish_tf)
  {
    child_frame_id = (std::string&) config_node[CHILD_FRAME_ID_PARAM_NAME];
    parent_frame_id = (std::string&) config_node[PARENT_FRAME_ID_PARAM_NAME];
  }
}




void PublishedRigidBody::publish(RigidBody &body, RigidBodyOdomHelper& odom_helper)
{
  // don't do anything if no new data was provided
  if (!body.has_data())
  {
    return;
  }
  // NaN?
  if (body.pose.position.x != body.pose.position.x)
  {
    return;
  }

  // TODO Below was const, see if there a way to keep it like that.
  geometry_msgs::PoseStamped pose = body.get_ros_pose();

  geometry_msgs::PointStamped marker = body.get_ros_marker();
  nav_msgs::Odometry odom;
  odom.header = pose.header;
  odom.pose.pose = pose.pose;
  // http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom
 
  
  double v = 0;
  double omega = 0;

   ros::Time current_time = ros::Time::now();
   double current_x = pose.pose.position.x;
   double current_y = pose.pose.position.y;
   double current_th = tf::getYaw(pose.pose.orientation);
   
   if (odom_helper.first_values)
   {
     odom_helper.first_values = false;
     odom_helper.last_time = current_time;
     odom_helper.last_x = current_x;
     odom_helper.last_y = current_y;
     odom_helper.last_theta = current_th;
     return;
   }
   
   double dt = (current_time - odom_helper.last_time).toSec();
   double dx = (current_x - odom_helper.last_x);
   double dy = (current_y - odom_helper.last_y);
   double dtheta = normalize_theta(current_th - odom_helper.last_theta);
   
   
   v = (sqrt((dx*dx)+(dy*dy)))/dt;
   omega = dtheta/dt;
   
   // get velocity sign
   double orient_vec_x = cos(odom_helper.last_theta);
   double orient_vec_y = sin(odom_helper.last_theta);
   v = v * (double)sign(orient_vec_x*dx + orient_vec_y*dy);
  
   odom_helper.movav_vel(v);
   odom_helper.movav_omega(omega);
   
   odom.twist.twist.linear.x = odom_helper.movav_vel.getEstimate();
   odom.twist.twist.angular.z = odom_helper.movav_omega.getEstimate();
   
   // store current values for next call
   odom_helper.last_time = current_time;
   odom_helper.last_x = current_x;
   odom_helper.last_y = current_y;
   odom_helper.last_theta = current_th;

  
  /*
  
  static double current_x = pose.pose.position.x;
  static double current_y = pose.pose.position.y;
  static double current_z = pose.pose.position.z;
  static double current_th = pose.pose.orientation.z;

  static double last_x = pose.pose.position.x;
  static double last_y = pose.pose.position.y;
  static double last_th = pose.pose.orientation.z;
  
  static double vx = 0;
  static double vth = 0;
  double v = 0;
  double th = 0;
  
  static double dt = 0;
  static double dx = 0;
  static double dy = 0;
  static double dth = 0;
  const int array_size_v = 10;
  const int array_size_th = 10;
  
  static double vel_array[array_size_v] {};
  static double ang_array[array_size_th] {};
  
  static ros::Time current_time = ros::Time::now(); 
  static ros::Time last_time = ros::Time::now();
 
    current_time = ros::Time::now();
    current_x = pose.pose.position.x;
    current_y = pose.pose.position.y;
 
    
  // Umrechnen in Euler-Winkel
    current_th = tf::getYaw(pose.pose.orientation);
//   current_th = atan2(2* (pose.pose.orientation.x *pose.pose.orientation.y +pose.pose.orientation.z * pose.pose.orientation.w), 1 - 2*(pose.pose.orientation.y *pose.pose.orientation.y +pose.pose.orientation.z * pose.pose.orientation.z));
    
  dt = (current_time - last_time).toSec();
  dx = (current_x - last_x);
  dy = (current_y - last_y);
  dth = normalize_theta(current_th - last_th);
 
 //Winkelgeschwindigkeit berechnen
  
  vth = dth/dt;

 for(int k=array_size_th; k>0; k--)
      {
      ang_array[k] = ang_array[k-1];
  }
    
    ang_array[0] = vth;
   
    
  for(int i=array_size_th; i>0; i--)
  {
   th += ang_array[i];
  }
  
   th = th/array_size_th; 
 
   //Lineare Geschwindigkeit berechnen
   
  vx = (sqrt((dx*dx)+(dy*dy)))/dt;
    
    
  for(int k=array_size_v; k>0; k--){
    vel_array[k] = vel_array[k-1];

  }
  
    vel_array[0] = vx;
  
  for(int i=array_size_v; i>0; i--){
    v += vel_array[i];
 
  }
  
    v = v/array_size_v;    
   
    
    double orient_vec_x = cos(last_th);
    double orient_vec_y = sin(last_th);
    v = v * (double)sign(orient_vec_x*dx + orient_vec_y*dy);

    */
    
     //Vorzeichen der linearen Geschindigkeit berechnen
     /*
     if(current_th > (M_PI/2) && current_th < (-M_PI/2) && dx > 0)
     {
       v = -v;
     }
     
     if(current_th > (-M_PI/2) && current_th < (M_PI/2) && dx <0)
     {
       v = -v;
     }
    
    */
    
    
    /*  
    if(current_th < M_PI/4 && current_th > -M_PI/4 && dx < 0)
    {
      v = -v;
    }
    
    if(current_th > -(3/4)*M_PI && current_th < -(1/4)*M_PI && dy >0)
    {
      v = -v;
    }
    
    if(current_th > (3/4)*M_PI && current_th < -(3/4)*M_PI && dx > 0)
    {
      v = -v;
    }
    
    if(current_th < (3/4)*M_PI && current_th > (1/4)*M_PI && dy < 0)
    {
      v = -v;
    }
    
    */
    
    
//   last_time = current_time;
//   last_x = current_x;
//   last_y = current_y;
//   last_th = current_th;  
//    
//   odom.twist.twist.linear.x = v;
//   odom.twist.twist.angular.z = th;
//   


 
  geometry_msgs::PolygonStamped footprint_base_link = body.get_ros_footprint();
  footprint_base_link.header.frame_id = child_frame_id;
 
  if (publish_pose)
  {
    pose.header.frame_id = parent_frame_id;
    pose_pub.publish(pose);
  }

  if (!publish_pose2d && !publish_tf)
  {
    // nothing to do, bail early
    return;
  }

  tf::Quaternion q(pose.pose.orientation.x,
                   pose.pose.orientation.y,
                   pose.pose.orientation.z,
                   pose.pose.orientation.w);

  // publish 2D pose
  if (publish_pose2d)
  {
    geometry_msgs::Pose2D pose2d;
    pose2d.x = pose.pose.position.x;
    pose2d.y = pose.pose.position.y;
    pose2d.theta = tf::getYaw(q);
    pose2d_pub.publish(pose2d);
  }

  if (publish_marker)
  {
    marker.header.frame_id = parent_frame_id;
    marker_pub.publish(marker);
  }

  if (publish_odom)
  {
    odom.header.frame_id = parent_frame_id; // "odom"?
    odom_pub.publish(odom);
  }
  
  if (publish_tf)
  {
    // publish transform
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.pose.position.x,
                                     pose.pose.position.y,
                                     pose.pose.position.z));

    // Handle different coordinate systems (Arena vs. rviz)
    transform.setRotation(q);
    ros::Time timestamp(ros::Time::now());
    tf_pub.sendTransform(tf::StampedTransform(transform, timestamp, parent_frame_id, child_frame_id));
    
    if (publish_footprint)
    {
      footprint footprint_object(transform);
      geometry_msgs::PolygonStamped footprint_world = footprint_object.transform_footprint(footprint_base_link);
      footprint_world.header.frame_id = parent_frame_id;
      footprint_topic = body.name;
      footprint_topic += "/footprint";
      footprint_pub = n.advertise<geometry_msgs::PolygonStamped>(footprint_topic, 1000);
      footprint_pub.publish(footprint_world);
    }
  }
}

bool PublishedRigidBody::validateParam(XmlRpc::XmlRpcValue &config_node, const std::string &name)
{
  if (!config_node.hasMember(name))
  {
    return false;
  }

  if ((config_node[name].getType() != XmlRpc::XmlRpcValue::TypeString) && (config_node[name].getType() != XmlRpc::XmlRpcValue::TypeArray))
  {
    return false;
  }

  return true;
}

