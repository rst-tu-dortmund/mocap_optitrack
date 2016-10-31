#include "mocap_optitrack/mocap_datapackets.h"

#include <stdio.h>
#include <string>
#include <iostream>
#include <ros/console.h>
using namespace std;

RigidBody::RigidBody() 
  : NumberOfMarkers(0), marker(0)
{
}

RigidBody::~RigidBody()
{
  delete[] marker;
}

const geometry_msgs::PoseStamped RigidBody::get_ros_pose()
{
  geometry_msgs::PoseStamped ros_pose;
  ros_pose.header.stamp = ros::Time::now();
  // y & z axes are swapped in the Optitrack coordinate system
  ros_pose.pose.position.x = pose.position.x;
  ros_pose.pose.position.y = -pose.position.z;
  ros_pose.pose.position.z = pose.position.y;

  ros_pose.pose.orientation.x = pose.orientation.x;
  ros_pose.pose.orientation.y = -pose.orientation.z;
  ros_pose.pose.orientation.z = pose.orientation.y;
  ros_pose.pose.orientation.w = pose.orientation.w;

  return ros_pose;
}

bool RigidBody::has_data()
{
    static const char zero[sizeof(pose)] = { 0 };
    return memcmp(zero, (char*) &pose, sizeof(pose));
}

const geometry_msgs::PointStamped RigidBody::get_ros_marker()
{
  geometry_msgs::PointStamped ros_marker;
  ros_marker.header.stamp = ros::Time::now();
  /*ros_marker.point.x = marker[0].positionX;
  ros_marker.point.y = marker[0].positionY;
  ros_marker.point.z = marker[0].positionZ;*/

  ros_marker.point.x = distances[0]; // 12
  ros_marker.point.y = distances[1]; // 23
  ros_marker.point.z = distances[2]; // 13

  return ros_marker;
}

void RigidBody::pushBackDistanceVector(int a, int b)
{
  Marker marker;
  marker.positionX = this->marker[b].positionX - this->marker[a].positionX;
  marker.positionY = this->marker[b].positionY - this->marker[a].positionY;
  marker.positionZ = this->marker[b].positionZ - this->marker[a].positionZ;
  distance_vectors.push_back(marker);

}

void RigidBody::setNameFromKnownObjects(const std::vector<Object>& known_objects)
{
  float dist_threshold = 5;
  int min_hits = 3;
  
  std::vector<int> candidate_idx;
  std::vector<double> candidate_meandist;
  
  for (int i = 0; i < known_objects.size(); i++)
  {
    int hits_inside = 0;
    
    double distsum = 0;
    
    std::vector<int> object_distances = known_objects[i].distances_mm; // we make a copy in order to remove found values from the list of known candidates 
    
    for (int j=0; j< distances.size(); ++j)
    {
      // check if "distance" is contained in distances from the known object
      // we want to remove the found distance from the set of reference distances in order to allow the existence of multiple reference distances with the same length.
      // but since we have some threshold that we check (dist_threshold) we want to make sure to delete the correct distance!
      // hence we query all distances and only compare the smallest one with our threshold!
    std::vector<float> dist_cont; // distance, idx
    for (int k=0; k<object_distances.size(); ++k)
	dist_cont.push_back(std::abs( (float)object_distances[k]-distances[j]));
    
    // get minimum dist
    auto min_dist_it = std::min_element(dist_cont.begin(), dist_cont.end());
    if (min_dist_it == dist_cont.end())
	continue;
    
    if (*min_dist_it < dist_threshold)
    {
	// hit!!
	++hits_inside;
	distsum += *min_dist_it;
	// remove candidate from the current reference distances
	object_distances.erase(object_distances.begin()+std::distance(dist_cont.begin(), min_dist_it));
    }
  }
    
    if (hits_inside >= min_hits && hits_inside >= known_objects[i].distances_mm.size())
    {
      candidate_idx.push_back(i);
      candidate_meandist.push_back( distsum / double(hits_inside) );
    }
  }
  
  // find object that minimizes mean distance
  std::vector<double>::iterator min_iter = std::min_element(candidate_meandist.begin(), candidate_meandist.end());
  if (min_iter!= candidate_meandist.end())
  {
    int min_idx = std::distance(candidate_meandist.begin(), min_iter);
    candidate_meandist.data();
    int list_idx = candidate_idx[min_idx];
    this->name = known_objects[list_idx].name;
    this->footprint = known_objects[list_idx].footprint; //footprint_init
  }
  
  
  //     if (hits_inside > 0 && hits_inside >= Liste[i].Abstand.size())
//     {
//       ROS_INFO_STREAM(Liste[i].name << " found.");

//     }
  
//   if (this->name == "unbekannt")
//     ROS_INFO("Rigid Body %i unbekannt", this->ID);
}

const geometry_msgs::PolygonStamped RigidBody::get_ros_footprint()
{
  geometry_msgs::PolygonStamped ros_footprint;
  ros_footprint.header.stamp = ros::Time::now();
  ros_footprint.polygon.points.reserve(footprint.size());

  for (int i = 0; i < footprint.size(); i++)
  {
    ros_footprint.polygon.points.push_back(footprint.at(i));
  }
  
  return ros_footprint;
}

Object::Object(const vector< int >& dists_mm, const string& str, const vector< geometry_msgs::Point32 >& points)
{
  distances_mm = dists_mm;
  name = str;
  footprint = points;
}

ModelDescription::ModelDescription()
  : numMarkers(0), markerNames(0)
{
}

ModelDescription::~ModelDescription()
{
  delete[] markerNames;
}

ModelFrame::ModelFrame()
  : markerSets(0), otherMarkers(0), rigidBodies(0), 
    numMarkerSets(0), numOtherMarkers(0), numRigidBodies(0),
    latency(0.0)
{
}

ModelFrame::~ModelFrame()
{
  delete[] markerSets;
  delete[] otherMarkers;
  delete[] rigidBodies;
}

MoCapDataFormat::MoCapDataFormat(const char *packet, unsigned short length) 
  : packet(packet), length(length), frameNumber(0)
{
}

MoCapDataFormat::~MoCapDataFormat()
{
}

void MoCapDataFormat::seek(size_t count)
{
  packet += count;
  length -= count;
}

void MoCapDataFormat::parse()
{
  seek(4);

  // parse frame number
  read_and_seek(frameNumber);

  // count number of packetsets
  read_and_seek(model.numMarkerSets);
  model.markerSets = new MarkerSet[model.numMarkerSets];
  ROS_DEBUG("Number of marker sets: %d\n", model.numMarkerSets);

  for (int i = 0; i < model.numMarkerSets; i++)
  {
      strcpy(model.markerSets[i].name, packet);
      seek(strlen(model.markerSets[i].name) + 1);

      ROS_DEBUG("Parsing marker set named: %s\n", model.markerSets[i].name);

      // read number of markers that belong to the model
      read_and_seek(model.markerSets[i].numMarkers);
      ROS_DEBUG("Number of markers in set: %d\n", model.markerSets[i].numMarkers);

      model.markerSets[i].markers = new Marker[model.markerSets[i].numMarkers];
      for (int k = 0; k < model.markerSets[i].numMarkers; k++)
      {
	// read marker positions
	read_and_seek(model.markerSets[i].markers[k]);
      }
  }

  // read number of 'other' markers (cf. NatNet specs)
  read_and_seek(model.numOtherMarkers);
  model.otherMarkers = new Marker[model.numOtherMarkers];
  ROS_DEBUG("Number of markers not in sets: %d\n", model.numOtherMarkers);
  for (int l = 0; l < model.numOtherMarkers; l++)
  {
      // read positions of 'other' markers
      read_and_seek(model.otherMarkers[l]);
  }

  // read number of rigid bodies of the model
  read_and_seek(model.numRigidBodies);
  ROS_DEBUG("Number of rigid bodies: %d\n", model.numRigidBodies);

  model.rigidBodies = new RigidBody[model.numRigidBodies];
  for (int m = 0; m < model.numRigidBodies; m++)
  {
      // read id, position and orientation of each rigid body
      read_and_seek(model.rigidBodies[m].ID);
      read_and_seek(model.rigidBodies[m].pose);

      // get number of markers per rigid body
      read_and_seek(model.rigidBodies[m].NumberOfMarkers);
      ROS_DEBUG("Rigid body ID: %d\n", model.rigidBodies[m].ID);
      ROS_DEBUG("Number of rigid body markers: %d\n", model.rigidBodies[m].NumberOfMarkers);
      if (model.rigidBodies[m].NumberOfMarkers > 0)
      {
	model.rigidBodies[m].marker = new Marker [model.rigidBodies[m].NumberOfMarkers];

	size_t byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(Marker);
	memcpy(model.rigidBodies[m].marker, packet, byte_count);
	seek(byte_count);

	// skip marker IDs
	byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(int);
	seek(byte_count);

	// skip marker sizes
	byte_count = model.rigidBodies[m].NumberOfMarkers * sizeof(float);
	seek(byte_count);
	
	// Verbindungsvektoren
	model.rigidBodies[m].distance_vectors.clear();
	model.rigidBodies[m].distances.clear();
	for (int i_marker=0; i_marker < model.rigidBodies[m].NumberOfMarkers; ++i_marker)
	{
	  for (int j_marker=i_marker+1; j_marker < model.rigidBodies[m].NumberOfMarkers; ++j_marker)
	  {
	    if (i_marker == j_marker)
	      continue;
	    
	    model.rigidBodies[m].pushBackDistanceVector(i_marker, j_marker);
	  }
	}

	// Abstände
	for (int i = 0; i < model.rigidBodies[m].distance_vectors.size(); i++)
	{
	  model.rigidBodies[m].distances.push_back(Marker::norm(model.rigidBodies[m].distance_vectors[i]) * 1000); // [mm]
	}
  //       sort (model.rigidBodies[m].Abstand.begin(), model.rigidBodies[m].Abstand.end()); // aufsteigend sortiert
	model.rigidBodies[m].setNameFromKnownObjects(model.known_objects);
	
	ROS_INFO_STREAM("rigid body " << m << ": " << model.rigidBodies[m].name);
	
      }
      
      // skip mean marker error
      seek(sizeof(float));

      seek(2); // http://answers.ros.org/question/217341/multiple-rigid-bodies-crash-optitrack-node/
  }

  // TODO: read skeletons
  int numSkeletons = 0;
  read_and_seek(numSkeletons);

  // get latency
  read_and_seek(model.latency);
}
