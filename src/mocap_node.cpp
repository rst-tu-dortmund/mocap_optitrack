/// \author <a href="mailto:graeve@ais.uni-bonn.de">Kathrin Gräve</a>
///
/// ROS node that translates motion capture data from an OptiTrack rig to tf transforms.
/// The node receives the binary packages that are streamed by the Arena software,
/// decodes them and broadcasts the poses of rigid bodies as tf transforms.
///
/// Currently, this node supports the NatNet streaming protocol v1.4.

// Local includes
#include "mocap_optitrack/socket.h"
#include "mocap_optitrack/mocap_datapackets.h"
#include "mocap_optitrack/mocap_config.h"
#include "mocap_optitrack/skeletons.h"

// ROS includes
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <unordered_map>

// System includes
#include <string>
#include <unistd.h>

////////////////////////////////////////////////////////////////////////
// Constants

// ip on multicast group - cannot be changed in Arena
const std::string MULTICAST_IP = "224.0.0.1";

const std::string MOCAP_MODEL_KEY = "mocap_model";
const std::string RIGID_BODIES_KEY = "rigid_bodies";
const char ** DEFAULT_MOCAP_MODEL = SKELETON_WITHOUT_TOES;

const int LOCAL_PORT = 1511;

////////////////////////////////////////////////////////////////////////

void processMocapData( const char** mocap_model, RigidBodyMap& published_rigid_bodies, vector<Object> Liste)
{
  UdpMulticastSocket multicast_client_socket( LOCAL_PORT, MULTICAST_IP );

   std::unordered_map<std::string,RigidBodyOdomHelper> odom_helpers;
  
  ushort payload;
  int numberOfPackets = 0;
  while(ros::ok())
  {
    bool packetread = false;
    int numBytes = 0;

    do
    {
      // Receive data from mocap device
      numBytes = multicast_client_socket.recv();

      // Parse mocap data
      if( numBytes > 0 )
      {
        const char* buffer = multicast_client_socket.getBuffer();
        unsigned short header = *((unsigned short*)(&buffer[0]));

        // Look for the beginning of a NatNet package
        if (header == 7)
        {
          payload = *((ushort*) &buffer[2]);
          MoCapDataFormat format(buffer, payload);
          format.model.known_objects = Liste;
          format.parse();

          packetread = true;
          numberOfPackets++;

          if( format.model.numRigidBodies > 0 )
          {
            for( int i = 0; i < format.model.numRigidBodies; i++ )
            {

// Ausgabe Test
/*std::cout << "Start " << i << std::endl;
std::cout << format.model.Liste[1][0] << std::endl;
std::cout << format.model.Liste[1][1] << std::endl;
std::cout << format.model.Liste[1][2] << std::endl;*/
                
              string name = format.model.rigidBodies[i].name;
              RigidBodyMap::iterator item = published_rigid_bodies.find(name);

              if (item != published_rigid_bodies.end())
              {
		  RigidBodyOdomHelper& current_odom_helper = odom_helpers[name];
		  
                  item->second.publish(format.model.rigidBodies[i], current_odom_helper);
              }
            }
          }
        }
        // else skip packet
      }
    } while( numBytes > 0 );

    // Don't try again immediately
    if( !packetread )
    {
      usleep( 10 );
    }
  }
}

double getNumberFromXMLRPC(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
{
  return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
}



////////////////////////////////////////////////////////////////////////

int main( int argc, char* argv[] )
{ 
  
  // Initialize ROS node
  ros::init(argc, argv, "mocap_node");
  ros::NodeHandle n("~");

  // Get configuration from ROS parameter server  
  const char** mocap_model( DEFAULT_MOCAP_MODEL );
  if( n.hasParam( MOCAP_MODEL_KEY ) )
  {    std::string tmp;
    if( n.getParam( MOCAP_MODEL_KEY, tmp ) )
    {
      if( tmp == "SKELETON_WITH_TOES" )
        mocap_model = SKELETON_WITH_TOES;
      else if( tmp == "SKELETON_WITHOUT_TOES" )
        mocap_model = SKELETON_WITHOUT_TOES;
      else if( tmp == "OBJECT" )
        mocap_model = OBJECT;
    }
  }

  RigidBodyMap published_rigid_bodies;
  
  vector<Object> Liste;

  if (n.hasParam(RIGID_BODIES_KEY))
  {
      XmlRpc::XmlRpcValue body_list;
      n.getParam("rigid_bodies", body_list);
      if (body_list.getType() == XmlRpc::XmlRpcValue::TypeStruct && body_list.size() > 0)
      {
          XmlRpc::XmlRpcValue::iterator i;
          for (i = body_list.begin(); i != body_list.end(); ++i) {
              if (i->second.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                  PublishedRigidBody body(i->second);
                  string abstand = (string&) (i->second["abstand"]);
                  string name = (string&) (i->first);
                  
                  // string to char*
                  char *cstr = new char[abstand.length() + 1];
                  strcpy(cstr, abstand.c_str());

                  // Zahlen trennen und als int abspeichern
                  char *pch = strtok(cstr, ",");
                  vector<int> Abstand;
                  while (pch != NULL)
                  {
                    Abstand.push_back(atoi(pch));
                    pch = strtok (NULL, ",");
                  }
                  delete [] cstr;
                  sort (Abstand.begin(), Abstand.end());
  
                  std::string full_param_name;
                  vector<geometry_msgs::Point32> footprint;
  
                  std::string footprint_param = "/mocap_node/rigid_bodies/";
		  footprint_param += name;
                  footprint_param += "/footprint";
		  
                  if (n.searchParam(footprint_param, full_param_name))  
                  {
		    XmlRpc::XmlRpcValue footprint_xmlrpc;
		    n.getParam(full_param_name, footprint_xmlrpc);
		    if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
		    {
		      geometry_msgs::Point32 pt;

		      for (int i = 0; i < footprint_xmlrpc.size(); ++i)
		      {
			XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];

			pt.x = getNumberFromXMLRPC(point[ 0 ], full_param_name);
			pt.y = getNumberFromXMLRPC(point[ 1 ], full_param_name);
			
			footprint.push_back(pt);
		      }
		    }
                  }

                  // Liste ausfüllen
                  Liste.push_back({Abstand, name, footprint});

                  RigidBodyItem item(name, body);

                  std::pair<RigidBodyMap::iterator, bool> result = published_rigid_bodies.insert(item);
                  if (!result.second)
                  {
                      ROS_ERROR("Could not insert configuration for rigid body ID %s", name.c_str());
                  }
              }
          }
      }
  }

  // Process mocap data until SIGINT
  processMocapData(mocap_model, published_rigid_bodies, Liste);

  return 0;
}
