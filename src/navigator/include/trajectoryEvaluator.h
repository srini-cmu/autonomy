/***************************************************************************
*       trajectoryEvaluator.h
*	Summary: Header file for the Spring 2020 end-of-semester trajectory
*       evaluator--a component of the navigator's forward simulation.
*	@author Pat Callaghan
* 	@editor <NAME>
*	<General Structure>
*	Key References:
*	<Key References>
****************************************************************************/


/*
* <DESCRIPTION>
* @param
* @retval
* PRE:
* POST:
* SD EF:
*
*/


#ifndef TRAJECTORYEVALUATOR_H
#define TRAJECTORYEVALUATOR_H

//BEGIN C++ HEADERS:
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <tuple>

//BEGIN ROS HEADERS:
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"
#include "roverModel.h"

namespace trajectoryEvaluatorNamespace {
using std::vector;
using std::tuple;
using std::pair;

class trajectoryEvaluator {

public:
    trajectoryEvaluator(vector<tuple<double, double, double>> newSetOfArcs, double _stepSize, double _slopeThreshold);

    const tuple<double, double, double> getArc(int toGet) {
        return this->setOfArcs[toGet];
    }
    const double getArcCost(int arcToGet) {
        return this->arcCosts[arcToGet];
    }
    const int getCurrentArc() {
        return this->currentArc;
    }
    const int getGoalX() {
        return this->goalX;
    }
    const int getGoalY() {
        return this->goalY;
    }
    const int getStepSize() {
        return this->stepSize;
    }
    const double getSlopeThreshold() {
        return this->slopeThreshold;
    }
    void setGoalX(int toSet) {
        this->goalX = toSet;
        return;
    }
    void setGoalY(int toSet) {
        this->goalY = toSet;
        return;
    }
    void setStepSize(double toSet) {
        this->stepSize = toSet;
        return;
    }
    void setSlopeThreshold(double toSet) {
        this->slopeThreshold = toSet;
        return;
    }

    void fillRoverFootprint(std_msgs::Float64MultiArray nextMap, roverNamespace::rover& roverObject);
    void computeStepSize(double duration, double velocity);
    const double computeArcCost(int arcToCheck, std_msgs::Float64MultiArray nextMap, roverNamespace::rover roverObject);
    const pair<double, double> computeSlopeAndRoughnessCost(std_msgs::Float64MultiArray nextMap, roverNamespace::rover roverObject);
    const double computeDistanceFromGoal(double arcToCheck, int currentGoalX, int currentGoalY, roverNamespace::rover roverObject);
    const void transformFootprint(tuple<double, double, double> arcBeingConsidered, double currentStep, roverNamespace::rover& roverObject,std_msgs::Float64MultiArray nextMap);
    const double computeTheta(double rad, double dist);

private:
    vector<tuple<double, double, double>> setOfArcs; //A set of arcs defined by (radius, distance, speed)
    int    currentArc;
    bool   objectCollisionFlag;
    double slopeThreshold;
    int    goalX;
    int    goalY;
    double stepSize;
    double arcCosts[7];
};

}
#endif // TRAJECTORYEVALUATOR_H
