/***************************************************************************
*       navigator.cpp
*       Summary: Implementation file for the Spring 2020 end-of-semester
*       navigator, which incorporates the trajectory evaluation and 
*       arc-to-motor-comands components.
*       @author Pat Callaghan
*       @editor <NAME>
*       <General Structure>
*       Key References:
*       <Key References>

*
* <DESCRIPTION>
* @param
* @retval
* PRE:
* POST:
* SD EF:
*
*
*
* References:
****************************************************************************/
// BEGIN: C++ headers:
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <queue>
#include <string>
#include <time.h>
#include <tuple>
#include <utility>
#include <vector>

// BEGIN: ROS headers:
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "navigator/getNextLocalGoal.h" //TODO: Ensure transform node is appropriately adjusted
#include "navigator/Arc.h"
#include "roverModel.h"
#include "trajectoryEvaluator.h"

using namespace trajectoryEvaluatorNamespace;
using namespace roverModelNamespace;

//TODO:Change 'state' to pose
class navigator {
public:
  navigator(double startXCoord, double startYCoord) 
  	: localStart.pose.position.x(startXCoord), localStart.pose.position.y(startYCoord)
  {
		this->evaluator = new trajectoryEvaluator();
  }
  geometry_msgs::PoseStamped& getLocalGoal() { return localGoal; }
  void setLocalGoal(const double xCoord, const double yCoord) {
    localGoal.pose.position.x = xCoord;
    localGoal.pose.position.y = yCoord;
  }

  geometry_msgs::PoseStamped& getLocalStart() { return localStart; }
  void setLocalStart(double xCoord, double yCoord) {
    localStart.pose.position.x = xCoord;
    localStart.pose.position.y = yCoord;
  }

  void localMapCallback(std_msgs::Float64MultiArray map); //TODO: Implement
  void pathPublishingCreator(const std_msgs::Float64MultiArray &mappingInput,
                             const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal);
  void publishHeartbeat(); 

private:  
  trajectoryEvaluator evaluator;
  geometry_msgs::PoseStamped localGoal;
  geometry_msgs::PoseStamped localStart;
  ros::NodeHandle n;
  ros::Publisher requestLocalMap;
  ros::Publisher arcSelection;
  ros::Publisher heartbeat;
  ros::Subscriber mapSubscriber;
  ros::ServiceClient requestNewLocalGoal;

//Messages:
  navigator::getNextLocalGoal srv;
  navigator::Arc arcMessage;
};

//Initialize messages to be sent over the wire:
void navigator::publishSubscribeRequest() {
  ros::Rate pollRate(100);
  requestNewLocalGoal =
      n.serviceClient<ans::getNextLocalGoal>("tf_next_local_goal");
  arcChooser = n.advertise<navigator::Arc>("navigator/arc_commands", 1);
  mapSubscriber = n.subscribe("terrain_mapper/map_for_planning", 1, &navigator::localMapCallback, this);
  //mappingSub = n.subscribe("CSpaceTopic", 1, &localPlannerNode::localMapCallback, this); TODO: Uncertain if using this at present; make that determination
  return;
}

void navigator::publishHeartbeat() {
        heartbeat.publish(this->heartbeatMessage);
        return;
}

void navigator::localMapCallback(std_msgs::Float64MultiArray mappingInput) {
  geometry_msgs::PoseStamped roverPose; //odomReplicator should be replaced with the rover's actual coordinates in the local frame when calling for a new goal. 
  odomReplicator.x = 49;
  odomReplicator.y = 99;
  //Send the latest actual coordinate for transforming into the global frame and request a new local goal:
  srv.request.latestCoord = odomReplicator;  //currently, nothing is done with the odomReplicator point being sent as the request message; it is only a placeholder
  std::cout << "localMapTopic callback\n";
  if (this->requestNewLocalGoal.call(srv)) {
    localGoal.x = srv.response.nextLocalGoal.x;
    localGoal.y = srv.response.nextLocalGoal.y;
    std::cout << "localGoal: " << localGoal.x << " " << localGoal.y << std::endl;
    ROS_INFO("Planning a path to goal\n");
  }
  return;
}

/**
 * Function which calls a new @param map and evaluates the precomputed @param arcSet
 * for the best choice to drive. 
 *
 */
void navigator::evaluateNewMap(const std_msgs::Float64MultiArray &map, const geometry_msgs::PoseStamped &start, const geometry_msgs::PoseStamped &goal) {

  ROS_INFO("Start coords: %d , %d", start.x, start.y);
  ROS_INFO("Goal coords: %d , %d", goal.x, goal.y);



  ROS_INFO("Getting motorMsgs:");
  std::vector<std::pair<std_msgs::Float64, std_msgs::Float64>> motorMsgs =
      this->makeMotorMsgs(localPath); // get motor commands for publishing

  motorParamsPub.publish(motorParamMsg);
  return;
}

//////////////MAIN///METHOD//////////////////
int main(int argc, char **argv) {
  ros::init(argc, argv, "navigator");
  localPlannerNode LP(49, 0);
  while (ros::ok()) {
    LP.publishSubscribeRequest();
    LP.publishHeartbeat();
    ros::spin();
  }
  return 0;
}
///////////////////////////////////////////

