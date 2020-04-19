// BEGIN: ROS headers:
#include "navigator/getNextLocalGoal.h"
#include "navigator/transformedPoints.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
// END: ROS headers

// BEGIN: C++ headers:
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
// END: C++ headers

using std::vector;

/*
 * This node retrieves two global-map coordinates and transforms
 * them into the local frame. More specifically, the two retrieved global
 * coordinates are used to determine the next traversal goal coordinate in the
 * local frame.
 */

// Class which contains all pertinent functions and variables:
class globalToLocalTransformer {
public:
  globalToLocalTransformer(double localRes, double globalRes) {
    localMapResolution = localRes;
    globalMapResolution = globalRes;
  }
  // Euclidean distance manipulators:
  double getMaxED() const { return maxED; }
  void setMaxED() { maxED = localMapResolution * euclidDist(localOrigin, furthestLocalGoal); }

  void setLocalOrigin(int _x, int _y, int _z) {
    localOrigin.x = _x;
    localOrigin.y = _y;
    localOrigin.z = _z;
  }

  // Used to set a furthest point for calculating maximum Euclidean distance:
  void setFurthestLocalGoal(int _x, int _y, int _z) {
    furthestLocalGoal.x = _x;
    furthestLocalGoal.y = _y;
    furthestLocalGoal.z = _z;
  }
  geometry_msgs::Point getLocalOrigin() const { return localOrigin; }
  geometry_msgs::Point getFurthestLocalGoal() const {
    return furthestLocalGoal;
  }
  void setLocalGoalToSend(geometry_msgs::Point newLocalGoal) {
    localGoalToSend = newLocalGoal;
  }
  geometry_msgs::Point getLocalGoalToSend() const { return localGoalToSend; }
  // Used for scaling:
  double getLocalResolution() const { return localMapResolution; }
  double getGlobalResolution() const { return globalMapResolution; }
  void setNextGlobalStart(vector<geometry_msgs::PoseStamped>::iterator _start) {
    globalPathIt = _start;
  }
  void setGlobalGoal(
      const vector<geometry_msgs::PoseStamped>::iterator _globalGoal) {
    globalGoal = _globalGoal;
  }
  // Iterator used to iterate through the global path and select the next
  // furthest acceptable point on the global path for transforming:
  vector<geometry_msgs::PoseStamped>::iterator getNextGlobalGoal() const {
    return globalGoal;
  }

  vector<geometry_msgs::PoseStamped>::iterator getNextGlobalStart() const {
    return globalPathIt;
  }
  double euclidDist(geometry_msgs::Point start, geometry_msgs::Point next);
  geometry_msgs::Point transformToLocalGoal(geometry_msgs::Point start,
                                            geometry_msgs::Point goal);

  // Prototypes for ros node callback functions:
  void globalPathRetrieverCallback(nav_msgs::Path globalPathInput);
  bool sendNextLocalGoal(ans::getNextLocalGoal::Request &req,
                         ans::getNextLocalGoal::Response &res);

  // Function for initiating subscribers and services:
  void subscribeAndServe() {

    thetaPub = n.advertise<std_msgs::Float64>("localFrameHeading", 1, true);
    globalPathPointPub = n.advertise<geometry_msgs::PoseStamped>("globalPathPoint", 1, true);
  //  globalPathPointVisualPub = n.advertise<visualization_msgs::Marker>("globalPathPointVisual", 1, true);
    globalPathSubscriber = n.subscribe(
        "globalPathVectorTopic", 1,
        &globalToLocalTransformer::globalPathRetrieverCallback, this);

    sendLocalGoalService = n.advertiseService(
        "tf_next_local_goal", &globalToLocalTransformer::sendNextLocalGoal,
        this);
  }

private:
  vector<geometry_msgs::PoseStamped> globalPath;
  vector<geometry_msgs::PoseStamped>::iterator globalPathIt;
  double localMapResolution = 0.05; // meters/pixel
  double globalMapResolution = 1;  // meters/pixel

  geometry_msgs::Point localOrigin;
  geometry_msgs::Point furthestLocalGoal;
  geometry_msgs::Point localGoalToSend;
  vector<geometry_msgs::PoseStamped>::iterator globalGoal;
  visualization_msgs::Marker globalPathPointVisual;
  double maxED = 0.0;
  double lastTheta = 0.0;
  double roverHeading;

  ros::NodeHandle n;
  ros::Subscriber globalPathSubscriber;
  ros::Subscriber pathStatusSubscriber;
  ros::ServiceServer sendLocalGoalService;
  ros::Publisher thetaPub;
  ros::Publisher globalPathPointPub;
  ros::Publisher globalPathPointVisualPub;
};

// Subscriber callback to retrieve and store a global trajectory:
void globalToLocalTransformer::globalPathRetrieverCallback(
    nav_msgs::Path globalPathInput) {

  this->globalPath = globalPathInput.poses;
  this->setNextGlobalStart(this->globalPath.begin());
  //Set first global goal in compliance with the local frame:
  vector<geometry_msgs::PoseStamped>::iterator start =
      this->getNextGlobalStart();
  vector<geometry_msgs::PoseStamped>::iterator goal = start + 1;
  while ((globalMapResolution * euclidDist(start->pose.position, (goal + 1)->pose.position)) <
     maxED && (goal + 1) != this->globalPath.end()) {
       goal += 1;
  }
  this->setGlobalGoal(goal);
  return;
}

// Service for getting next global coordinates along the retrieved trajectory
// and transforming them into a new local goal:
bool globalToLocalTransformer::sendNextLocalGoal(
    ans::getNextLocalGoal::Request &req, ans::getNextLocalGoal::Response &res) {
  if (this->getNextGlobalStart() == this->globalPath.end()) {
    std::cout << "Global goal reached\n";
    return false;
  } else {
    vector<geometry_msgs::PoseStamped>::iterator start =
        this->getNextGlobalStart();
    
    //Publish global starting point for localMapMaker's bilinear interpolation:
    globalPathPointPub.publish(*start);
    //Publish visual of the local start and goal on the global map:
    globalPathPointVisual.type = visualization_msgs::Marker::POINTS;
    globalPathPointVisual.header.frame_id = "map";
    globalPathPointVisual.header.stamp = ros::Time::now();
    globalPathPointVisual.action = visualization_msgs::Marker::ADD;
    globalPathPointVisual.pose.orientation.x = globalPathPointVisual.pose.orientation.y = globalPathPointVisual.pose.orientation.z = 0; 
    globalPathPointVisual.pose.orientation.w = 1; 
    globalPathPointVisual.scale.x = 1;
    globalPathPointVisual.scale.y = 1;
    globalPathPointVisual.color.b = 1;
    globalPathPointVisual.color.a = 1;
    globalPathPointVisual.lifetime = ros::Duration(2.0);
    globalPathPointVisual.points.push_back(start->pose.position);
    globalPathPointVisual.points.push_back(getNextGlobalGoal()->pose.position);
    globalPathPointVisualPub.publish(globalPathPointVisual);
    globalPathPointVisual.points.clear();

    //set iterator for next call to sendNextLocalGoal:
    this->setLocalGoalToSend(
        this->transformToLocalGoal(start->pose.position, getNextGlobalGoal()->pose.position));
    std::cout << "globalStart: \n"
              << start->pose.position << "\nglobalGoal: \n"
              << getNextGlobalGoal()->pose.position
              << "\nLocal Goal: " << this->getLocalGoalToSend() << std::endl;
    this->setNextGlobalStart(start + 1);
    this->setGlobalGoal(getNextGlobalGoal() + 1);

    // Fill service's response message:
    res.nextLocalGoal = this->getLocalGoalToSend();
    ROS_INFO("Sending res back\n");
    }
  return true;
}

// TODO void pathStatusCallback (for requesting new start, goal coords)

geometry_msgs::Point
globalToLocalTransformer::transformToLocalGoal(geometry_msgs::Point start,
                                                    geometry_msgs::Point goal) {
  // NOTE: Needn't transform 'start' because the local map's start-point will be
  // constant.

  geometry_msgs::Point resultant;
  geometry_msgs::Point holder;
  // Calculate vector between goal and start:
    resultant.x = goal.x - start.x;
    resultant.y = goal.y - start.y;
  // need line perpendicular to the resultant vector ie. the cross-product of
  // k-hat (0, 0, 1) and the resultant. This perpendicular line represents the
  // local frame's x-axis:
    holder.x = resultant.x;
    resultant.x = -1 * resultant.y;
    resultant.y = holder.x;

  std::cout << "Resultant vector: " << resultant << std::endl;
  geometry_msgs::Point comparisonAxis;
  comparisonAxis.x = 1.0;
  comparisonAxis.y = comparisonAxis.z = 0.0;

  // Calcuate the angle theta between global map's x-axis and the line
  // perpendicular to the previously-calculated resultant.
  double theta = acos(
      ((resultant.x * comparisonAxis.x) + (resultant.y * comparisonAxis.y)) /
      (sqrt((resultant.x * resultant.x) + (resultant.y * resultant.y)) *
       sqrt((comparisonAxis.x * comparisonAxis.x) +
            (comparisonAxis.y * comparisonAxis.y))));
 
//Perhaps not necessary, but keeping here for safe-keeping:
  if (resultant.x == -0) { 
    resultant.x = 0;
  }
  if (resultant.y == -0) {
    resultant.y = 0;
  }
  

// determine if x & y have opposite signs; calculate theta accordingly:
  if ((static_cast<int>(resultant.x) ^ static_cast<int>(resultant.y)) <
      0) { 
    if (resultant.x < 0) {
      theta = M_PI - theta;
    } else {
      theta = theta - M_PI;
    }
  }
  else {
    if (resultant.x < 0) {
      theta = theta - M_PI;
    } else {
      theta = M_PI - theta;
    }
  }

  if (start.x == globalPath.begin()->pose.position.x && start.y == globalPath.begin()->pose.position.y) {
    lastTheta = theta;
  }

  roverHeading = theta;
  std_msgs::Float64 sendTheta;
  //sending the rotation angle to the localMapMaker:
  sendTheta.data = roverHeading; 
  thetaPub.publish(sendTheta);

  geometry_msgs::Point transformedPoint = goal;

  //Apply translation:
  transformedPoint.x += -(start.x);
  transformedPoint.y += -(start.y);
  // Apply rotation:
  geometry_msgs::Point newTransformedPoint = transformedPoint;
  newTransformedPoint.x =
      (transformedPoint.x * cos(lastTheta)) - (sin(lastTheta) * transformedPoint.y); 
  newTransformedPoint.y =
      (transformedPoint.x * sin(lastTheta)) + (cos(lastTheta) * transformedPoint.y);
  this->lastTheta = theta;

  // Apply scaling:

   newTransformedPoint.x =
      ( newTransformedPoint.x / localMapResolution );// / globalMapResolution;

   newTransformedPoint.y =
       ( newTransformedPoint.y / localMapResolution );// / localMapResolution;

  newTransformedPoint.x += 49;

  return newTransformedPoint; // Calculations up to this point are accurate
}

// For calculating the Euclidean distance between two points:
double globalToLocalTransformer::euclidDist(geometry_msgs::Point start,
                                                 geometry_msgs::Point next) {
  double xDist = abs(next.x - start.x);
  double yDist = abs(next.y - start.y);
  double distance = sqrt((xDist * xDist) + (yDist * yDist));

  return distance;
}

///////MAIN METHOD/////////

int main(int argc, char **argv) {
  ros::init(argc, argv, "globalToLocalTransformer");
  globalToLocalTransformer g2l(
      0.05, 2.0); // Initialize with (local resolution, global resolution)    <----real local should be 0.05
  g2l.setLocalOrigin(49, 0, 0); //Represents the rover's frame of reference
  g2l.setFurthestLocalGoal(49, 99, 0);
  g2l.setMaxED();
  g2l.subscribeAndServe();
  ros::spin();
  return 0;
}
