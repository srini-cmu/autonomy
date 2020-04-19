/***************************************************************************
*       trajectoryEvaluator.cpp
*       Summary: Implementation file for the Spring 2020 end-of-semester trajectory
*       evaluator--a component of the navigator's forward simulation.
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
* Standard deviation as reasonable terrain roughness metric: https://www.pnas.org/content/pnas/suppl/2005/04/20/0501207102.DC1/01207SuppText.pdf
****************************************************************************/

//BEGIN C++ HEADERS:
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <vector>
#include <tuple>
#include <cmath>

//BEGIN ROS HEADERS:
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectoryEvaluator.h"
#include "std_msgs/Float64MultiArray.h"
#include "roverModel.h"

namespace trajectoryEval{
using std::vector;
using std::tuple;
using namespace roverModelNamespace;
using namespace trajectoryEvaluatorNamespace;

//Constructors:
trajectoryEvaluator(double _slopeThreshold)
    :
{
    this->objectCollisionFlag = false;
}

trajectoryEvaluator(vector<tuple<double, double, double>> newSetOfArcs, double _stepSize, double _slopeThreshold)
    : setOfArcs(newSetOfArcs), stepSize(_stepSize), slopeThreshold(_slopeThreshold)
{
    this->objectCollisionFlag = false;
}


const double computeArcCost(int arcToCheck, std_msgs::Float64MultiArray nextMap, roverModel roverObject) {
    double arcCostTotal = 1;
    pair<double, double> slopeAndRoughness;
    double distanceFromGoalCost;
    //Compute the number of frames along the arc we'll need to check. Note: Dividing both terms by the map's resolution effectively cancels out those divisors (ie. (a/b) / (c/b) == a/c)
    double scaledSteps = std::get<1>(this->setOfArcs.at(arcToCheck)) / this->stepSize;
    std::cout << "scaled steps: " << scaledSteps << std::endl;
    for (int i = 0; i < scaledSteps; ++i) {
        roverObject.resetFootprint();
        this->transformFootprint(setOfArcs.at(arcToCheck), i, roverObject, nextMap);
        slopeAndRoughness = this->computeSlopeAndRoughnessCost(nextMap, roverObject);
        //If an obstacle was found in the computeSlopeAndRoughnessCost evaluation, assign this arc a max-value cost:
        if (this->objectCollisionFlag == true) {
            this->objectCollisionFlag = false;
            this->arcCosts[arcToCheck] = std::numeric_limits<double>::max();
            return std::numeric_limits<double>::max();
        }
        if (i == (scaledSteps - 1)) distanceFromGoalCost = this->computeDistanceFromGoal(arcToCheck, this->goalX, this->goalY, roverObject);
        arcCostTotal += arcCostTotal * distanceFromGoalCost + arcCostTotal * slopeAndRoughness.first + arcCostTotal * slopeAndRoughness.second;
    }
    std::cout << "arcCostTotal: " << arcCostTotal << std::endl;
    this->arcCosts[arcToCheck] = arcCostTotal;
    return arcCostTotal;

}


const pair<double, double> computeSlopeAndRoughnessCost(std_msgs::Float64MultiArray nextMap, roverModel roverObject) {
    //Need to halve the array as such: all x coords from (0 to mapWidth / 2), then (mapWidth / 2 to mapWidth). All y coords from (0 to mapLength):
    double rollCost;
    //Need to halve the array as such: all x coords from (0 to mapWidth). All y coords from (0 to mapLength / 2), then (mapLength / 2 to mapLength):
    double pitchCost;
    //Return the standard deviation of lengths across the map array as some indicator of terrain roughness:
    double overallMeanLength;
    double variance;
    double stdDev;

    double firstMeanLength; //For roll: left half. For pitch: top half
    double secondMeanLength; //For roll: right half. For pitch: bottom half

    //ROLL:
    for (int i = 0; i < (roverObject.getScaledWidth() / 2); ++i) {
        for (int j = 0; j < roverObject.getScaledLength(); ++j) {
            firstMeanLength   += nextMap.getValue(roverObject.getFootprintCoord(i, j, 1) + nextMap.getWidth() * roverObject.getFootprintCoord(i, j, 2));
            //While iterating, calculate the map's overall mean length:
            overallMeanLength += nextMap.getValue(roverObject.getFootprintCoord(i, j, 1) + nextMap.getWidth() * roverObject.getFootprintCoord(i, j, 2));
        }
    }
    for (int i = (roverObject.getScaledWidth() / 2); i < roverObject.getScaledWidth(); ++i) {
        for (int j = 0; j < roverObject.getScaledLength(); ++j) {
            secondMeanLength  += nextMap.getValue(roverObject.getFootprintCoord(i, j, 1) + nextMap.getWidth() * roverObject.getFootprintCoord(i, j, 2));
            //While iterating, calculate the map's overall mean length
            overallMeanLength += nextMap.getValue(roverObject.getFootprintCoord(i, j, 1) + nextMap.getWidth() * roverObject.getFootprintCoord(i, j, 2));
        }
    }

    overallMeanLength /= roverObject.getScaledLength() * roverObject.getScaledWidth();
    firstMeanLength    = firstMeanLength /  ((roverObject.getScaledWidth() / 2) * roverObject.getScaledLength());
    secondMeanLength   = secondMeanLength / ((roverObject.getScaledWidth() / 2) * roverObject.getScaledLength());

    rollCost = abs(firstMeanLength - secondMeanLength) / roverObject.getScaledWidth(); //CAUSING ISSUE AT RANDOM
    firstMeanLength = 0;
    secondMeanLength = 0;

    //PITCH:
    for (int i = 0; i < roverObject.getScaledWidth(); ++i) {
        for (int j = 0; j < (roverObject.getScaledLength() / 2); ++j) {
            secondMeanLength +=  nextMap.getValue(roverObject.getFootprintCoord(i, j, 1) + nextMap.getWidth() * roverObject.getFootprintCoord(i, j, 2));
            //While iterating, calculate the map's total length variance
            variance += std::pow(nextMap.getValue(roverObject.getFootprintCoord(i, j, 1) + nextMap.getWidth() * roverObject.getFootprintCoord(i, j, 2)) - overallMeanLength, 2);
        }
    }
    for (int i = 0; i < roverObject.getScaledWidth(); ++i) {
        for (int j = (roverObject.getScaledLength() / 2); j < roverObject.getScaledLength(); ++j) {
            firstMeanLength +=   nextMap.getValue(roverObject.getFootprintCoord(i, j, 1) + nextMap.getWidth() * roverObject.getFootprintCoord(i, j, 2));
            //While iterating, calculate the map's total length variance
            variance += std::pow(nextMap.getValue(roverObject.getFootprintCoord(i, j, 1) + nextMap.getWidth() * roverObject.getFootprintCoord(i, j, 2)) - overallMeanLength, 2);
        }
    }
    stdDev = std::sqrt((variance / (roverObject.getScaledLength() * roverObject.getScaledWidth()))); //@param stdDev == roughness
    firstMeanLength  = firstMeanLength  / (roverObject.getScaledWidth() * (roverObject.getScaledLength() / 2));
    secondMeanLength = secondMeanLength / (roverObject.getScaledWidth() * (roverObject.getScaledLength() / 2));
    pitchCost = abs(firstMeanLength - secondMeanLength) / roverObject.getScaledLength();

    //If either of the calculated slopes is greater than we'd prefer, indicate there is an obstacle collision:
    if (pitchCost >= this->slopeThreshold || rollCost >= this->slopeThreshold) {
        std::cout << "Raising flag with pitch and roll values: " << pitchCost << " " << rollCost << std::endl;
        this->objectCollisionFlag = true;
        return pair<double, double>(-1.0, -1.0);
    }
    std::cout << "roll cost and pitch cost : " << rollCost << " " << pitchCost << std::endl;
    double slopeCost = rollCost + pitchCost; //CAUSING ISSUE AT RANDOM
    slopeCost /= slopeThreshold;
    return pair<double, double>(slopeCost, stdDev);
}

const double computeDistanceFromGoal(double arcToCheck, int currentGoalX, int currentGoalY, roverModel roverObject) {
    double xDistance = roverObject.getFootprintCoord(roverObject.getScaledWidth() / 2, 0, 1); //Get rover origin's global x coordinate
    double yDistance = roverObject.getFootprintCoord(roverObject.getScaledWidth() / 2, 0, 2); //Get rover origin's global y coordinate
    std::cout << "x and y coords retrieved: " << xDistance << " " << yDistance << std::endl;

    //Compute x and y vectors between rover and goal positions
    xDistance = abs(xDistance - currentGoalX);
    yDistance = abs(yDistance - currentGoalY);

    //Compute the displacement magnitude:
    double displacement = std::sqrt( std::pow(xDistance, 2) + std::pow(yDistance, 2));
    std::cout << "displacement from goal: " << displacement << std::endl;
    return displacement;
}


//Returns a 2D vector in which each represented cell contains the (x,y) coordinate of the mapArray cell from which we will draw length data
        //*****I don't think we want to pass by reference b/c we want to keep roverObject footprint coords in their original state for repeating transformation checking
const void transformFootprint(tuple<double, double, double> arcBeingConsidered, double currentStep, roverModel& roverObject, std_msgs::Float64MultiArray mapObject) {
    //arc tuple: (radius, distance, speed)
    double scalar         = mapObject.getResolution();
    double scaledRadius   = std::get<0>(arcBeingConsidered) / scalar;
    double scaledDistance = currentStep * this->stepSize / scalar;
    double currentTheta   = computeTheta(scaledRadius, scaledDistance);

    //Get the difference of the scaled radius and the x-coordinate of the drive arc at this particular step:
    double translateX     = scaledRadius * cos(currentTheta);
           translateX     = scaledRadius - translateX;
    double translateY     = scaledRadius * sin(currentTheta);

    //Move the arc origin to the rover's perspective ie. the middle of the local map frame:
    translateX += mapObject.getWidth() / 2;

    //transform those initialized coordinates by translateX, translateY translation and currentTheta rotation:
    for (int i = 0; i < roverObject.getScaledWidth(); ++i) {
        for (int j = 0; j < roverObject.getScaledLength(); ++j) {
            //Transform: Rotate, then translate:
            double newX = roverObject.getFootprintCoord(j, i, 1) * cos(currentTheta) - roverObject.getFootprintCoord(j, i, 2) * sin(currentTheta) + translateX;
            double newY = roverObject.getFootprintCoord(j, i, 1) * sin(currentTheta) + roverObject.getFootprintCoord(j, i, 2) * cos(currentTheta) + translateY;
            newY = round(newY);
            newX = round(newX);
            //If newX or newY are out of the local frame, set them to the frame's max dimension: OR say this arc is a no-go
            if (newX >= mapObject.getWidth()) { newX = mapObject.getWidth() - 1; }
            if (newY >= mapObject.getLength()) { newY = mapObject.getLength() - 1; }
            //Assign coordinates:
            roverObject.setFootprintCoord(j, i, 1, newX);
            roverObject.setFootprintCoord(j, i, 2, newY);
        }
    }
    return;
}

//********* MAY NOT ACTUALLY NEED: *********************
//Fetch the data in the mapArray cells and put them in the roverFootprintData cells:
void fillRoverFootprint(std_msgs::Float64MultiArray nextMap, roverModel& roverObject) {
    int indexToPlace;
    for (int i = 0; i < roverObject.getScaledWidth(); ++i) {
        for (int j = 0; j < roverObject.getScaledLength(); ++j) {
           // std::cout<<"testing"<<std::endl;
            indexToPlace = roverObject.getFootprintCoord(i, j, 1) + roverObject.getScaledWidth() * roverObject.getFootprintCoord(i, j, 2);
            //std::cout<<"nextMap data:"<< nextMap.getValue(indexToPlace) << std::endl;
            roverObject.setFootprintLengthData(i, j, nextMap.data.getValue(indexToPlace));
        }
    }
    return;
}

//Compute the rover's angle after traveling a certain distance along an arc with a certain radius:
const double computeTheta(double rad, double dist) {
    if (rad == 0) return 0.0;
    std::cout<<"compute theta: " << dist / rad << std::endl;
    return dist / rad;
}

}
