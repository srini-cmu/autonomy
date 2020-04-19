// BEGIN: ROS headers:
#include "Dstar.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Char.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
// END: ROS headers

// BEGIN: C++ headers:
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <queue>
#include <string>
#include <time.h>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
// END: C++ headers

namespace std {
template <>
struct hash<state> { // template specialization; struct hash which handles
                     // 'state' datatype
  typedef state argument_type;     // member type
  typedef std::size_t result_type; // member type
  std::size_t operator()(const state &id) const
      noexcept { // operator() is a member function
    return std::hash<int>()(id.x ^ (id.y << 4));
  }
};
} // namespace std

struct SquareGrid {

  int width, height;
  std::unordered_set<state> walls; // I believe this is a reference to
                                   // obstacles--not the grid's dimensions

  SquareGrid(int width_, int height_) : width(width_), height(height_) {}

  bool in_bounds(state id) const {
    return 0 <= id.x && id.x < width && 0 <= id.y && id.y < height;
  }

  bool passable(state id) const { return walls.find(id) == walls.end(); }
};

struct GridWithWeights : SquareGrid {
  std::vector<state> landscape;
  GridWithWeights(int w, int h) : SquareGrid(w, h) {}
};
class planner {
public:
  planner();
  planner(int xCoord, int yCoord) {
    globalStart.x = xCoord;
    globalStart.y = yCoord;
    heartbeatMessage.data = "Planner is alive.";
    response.data = "Global planner: Request received.";
  }
  // Manipulators:
  void setGlobalStart(int xCoord, int yCoord) {
    globalStart.x = xCoord;
    globalStart.y = yCoord;
  }
  state &getGlobalStart() { return globalStart; }
  void setGlobalGoal(int xCoord, int yCoord) {
    globalGoal.x = xCoord;
    globalGoal.y = yCoord;
  }
  state &getGlobalGoal() { return globalGoal; }
  void setGlobalGrid(int _width, int _height) {
    globalGrid.width = _width;
    globalGrid.height = _height;
  }
  GridWithWeights &getGlobalGrid() { return globalGrid; }
  // End manipulators

  std::vector<geometry_msgs::Point>
  reconstructDstarPath(std::list<state> &path);
  void pathPublishingCreator(Dstar *dstarLiteObject, GridWithWeights &planOnMe,
                             nav_msgs::OccupancyGrid &mappingInput,
                             state &start, state &goal);
  void globalMapCallback(nav_msgs::OccupancyGrid mappingInput);
  void roverExecutiveCallback(std_msgs::String executiveCommand);
  void publishHeartbeat();
  void subscribeAndPublish();

private:
  GridWithWeights globalGrid{500, 500};
  nav_msgs::OccupancyGrid globalMap;
  state globalStart{0, 0};
  state globalGoal{0, 0};
  ros::NodeHandle n;
  ros::Publisher globalPathVectorPub;
  //ros::Publisher globalPathVisualPub;
  ros::Publisher acknowledgeExecutive;
  ros::Publisher heartbeat;
  ros::Subscriber roverExecutiveSub;
  ros::Subscriber mappingSub;

  // Messages:
  nav_msgs::Path pathVector;
  std_msgs::String heartbeatMessage;

  std_msgs::String response;
 // visualization_msgs::Marker pathVisual;
  geometry_msgs::PoseStamped pushToPath;
};

void planner::subscribeAndPublish() {
  ros::Rate poll_rate(100);
  mappingSub =
      n.subscribe("map", 1, &planner::globalMapCallback, this);
  roverExecutiveSub = n.subscribe("/rover_executive/control_nodes", 1, &planner::roverExecutiveCallback, this);
  globalPathVectorPub = n.advertise<nav_msgs::Path>(
      "globalPathVectorTopic", 1, true); //"true" for latching the topic
  acknowledgeExecutive = n.advertise<std_msgs::String>("planner/ack", 1, true);
  heartbeat = n.advertise<std_msgs::String>("internal_heartbeat/planner", 1, true);
  //globalPathVisualPub =
   //   n.advertise<visualization_msgs::Marker>("pathVisualTopic", 1, true);
  // The following while-loop makes sure that if the requisite subscribers
  // haven't yet connected, the global path won't be published.
 // while (globalPathVectorPub.getNumSubscribers() == 0 || // < 2
 //        globalPathVisualPub.getNumSubscribers() == 0) {
 //   poll_rate.sleep();
 // }
  return;
}

GridWithWeights weightsGrid(nav_msgs::OccupancyGrid *mappingInput,
                            Dstar *dstarLite) {
  int gridH = mappingInput->info.height;
  int gridW = mappingInput->info.width;
  GridWithWeights grid(gridW, gridH);

  int counter = 0;
  int setter = 0;

  for (int height = 0; height < gridH; ++height) { //initial set to 2 to prevent planner from planning off of the map
    for (int width = 0; width < gridW; ++width) {
      state current{width, height};
      if (mappingInput->data[counter] >=   // <---for map_server scale 
		     62 ){ //<---for map_server scale
        dstarLite->updateCell(width, height, -1);
      } else {
//	      dstarLite->updateCell(width, height, 1);
        dstarLite->updateCell(width, height, 0.05 * std::abs( static_cast<int>((mappingInput)->data[counter])));// 1); // Minimum must be >0   &&  must multiply by 0.1 for planner to function in reasonable time. Larger costs seem to slow it down substantially.
      }
      ++counter;
    }
  }
  return grid;
}

std::vector<geometry_msgs::Point>
planner::reconstructDstarPath(std::list<state> &path) {
  std::vector<geometry_msgs::Point> convertedPath;
  auto pathFront = path.begin();

  for (int i = 0; i < path.size(); ++i) {
    geometry_msgs::Point poseFromState;
    poseFromState.x = (*pathFront).x;
    poseFromState.y = (*pathFront).y;

    convertedPath.push_back(poseFromState);

   // std::cout << "path coord: x: " << pathFront->x
   //           << ", and the y: " << pathFront->y << std::endl;
    std::advance(pathFront, 1);
  }
  return convertedPath;
}

void planner::pathPublishingCreator(
    Dstar *dstarLiteObject, GridWithWeights &planOnMe,
    nav_msgs::OccupancyGrid &mappingInput, state &start, state &goal) {
  // The following block is so global path can be represented on rviz
 // pathVisual.header.frame_id = "map";
 // pathVisual.header.stamp = ros::Time::now();
 // pathVisual.id = 0;
 // pathVisual.type = visualization_msgs::Marker::LINE_STRIP;
 // pathVisual.action = visualization_msgs::Marker::ADD;
 // pathVisual.pose.orientation.w = 1.0;
 // pathVisual.scale.x = 0.5; //10;
 // pathVisual.color.g = 1.0f;
 // pathVisual.color.a = 1.0;
 // pathVisual.ns = "line_strip_path";
 // pathVisual.lifetime = ros::Duration();
  // End rviz block

  dstarLiteObject->init(start.x, start.y, goal.x, goal.y);

  planOnMe = weightsGrid(
      &mappingInput,
      dstarLiteObject); // create a grid with weighted nodes (that is
                        // interpretable by the Dstar algorithm) in accordance
                        // with the mappingInput occupancy grid.
  std::cout << "About to replan()\n";
  dstarLiteObject->replan();

  std::cout << "Replan finished\n";
  ROS_INFO("Start coords: %d , %d", start.x, start.y);
  ROS_INFO("Goal coords: %d , %d", goal.x, goal.y);

  list<state> path =
      dstarLiteObject->getPath(); // get the path found by the algorithm

  ROS_INFO("Getting GLPath:");
  vector<geometry_msgs::Point> GLPath = reconstructDstarPath(path);

  // Making the path vector for globalPathVisual and globalPathVector:
  for (int i = 0; i < GLPath.size(); ++i) {
    pushToPath.pose.position = GLPath[i];
    pathVector.poses.push_back(pushToPath);

   // pathVisual.points.push_back(GLPath[i]);
    //    std::cout<<"GLPath coord: x: " << GLPath[i].position.x << ", and the
    //    y: "<<GLPath[i].position.y<<std::endl;
  }
  //globalPathVisualPub.publish(pathVisual);
  globalPathVectorPub.publish(pathVector);
  return;
}

void planner::globalMapCallback(nav_msgs::OccupancyGrid mappingInput) {
  std::cout << "In globalMapCallback\n";
  this->setGlobalGrid(500, 500); 
  this->globalMap = mappingInput;

  return;
}

//TODO: Make sure the string parsing is accurate.
void planner::roverExecutiveCallback(std_msgs::String executiveCommand) {
	char char_array[executiveCommand.data.length() + 1];
	std::strcpy(char_array, executiveCommand.data.c_str());	
	if (executiveCommand.data.find("Global planner") == true) {

		acknowledgeExecutive.publish(this->response);

		//Set planner's coordinates:
		string cppString = executiveCommand.data;
		int startX, startY, goalX, goalY;
		startX = std::stoi(executiveCommand.data.substr(18, cppString.find(",") - 1));
		startY = std::stoi(executiveCommand.data.substr(cppString.find(",") + 1, cppString.find(")") - 1));
		cppString = cppString.substr(cppString.find(")") + 1, cppString.length() - 2);

		goalX = std::stoi(executiveCommand.data.substr(1, cppString.find(",") - 1));
		goalY = std::stoi(executiveCommand.data.substr(cppString.find(",") + 1, cppString.length() - 1));
		setGlobalStart(startX, startY);
		setGlobalGoal(goalX, goalY);

		Dstar *dstarLiteObject = new Dstar();
		ROS_INFO("Planning a path to goal\n");
		pathPublishingCreator( dstarLiteObject, this->getGlobalGrid(), this->globalMap,
		    this->getGlobalStart(),
		    this->getGlobalGoal());
		ROS_INFO("Path Planned\n");
	}

	return;
}

void planner::publishHeartbeat() {
	heartbeat.publish(this->heartbeatMessage);
	return;
}

//////////////////////MAIN///METHOD///////////////////////
int main(int argc, char **argv) {
  ros::init(argc, argv, "planner");
  //Initialize to start coords (0, 0):
  planner gpn(0, 0);
  while (ros::ok()) {
    gpn.subscribeAndPublish();
    gpn.publishHeartbeat();
    ros::spin();
  }
  return 0;
}
//////////////////////////////////////////////////////////
