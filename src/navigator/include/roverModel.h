/***************************************************************************
*       roverModel.h
*       Summary: Header file for the Spring 2020 end-of-semester navigator--
*       this roverModel contains rover dimensions for easy reference and 
*       manipulation as they pertain to the navigator's functionality. 
*       This is a component of the navigator's forward simulation.
*       @author Pat Callaghan
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

#ifndef ROVER_H
#define ROVER_H

//BEGIN C++ HEADERS:
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <tuple>

//BEGIN ROS HEADERS:
#include "ros/ros.h"
#include "std_msgs/MultiArrayFloat64.h"
#include "std_msgs/String.h"
#include "roverModel.h"

namespace roverModelNamespace {
using std::vector;
using std::pair;

class rover
{
public:
    rover() {}
    rover(double _width, double _length, double _localResolution);// (m, m, m/cell)

    const double getWidth() { return this->width; }
    const double getLength() { return this->length; }
    const double getScaledWidth() { return this->scaledWidth; }
    const double getScaledLength() { return this->scaledLength; }
    const double getResolution() { return this->localResolution; }
    const vector<pair<double, double>> getFootprint() { return this->footprint; }
    const vector<double> getFootprintLengthData() { return this->footprintLengthData; }
    const double getFootprintCoord(int x, int y, int pairChoice);

    void setWidth(double toSet) {
        this->width = toSet;
        return;
    }
    void setLength(double toSet) {
        this->length = toSet;
        return;
    }
    void setScaledWidth(double toSet) {
        this->scaledWidth = toSet;
        return;
    }
    void setScaledLength(double toSet) {
        this->scaledLength = toSet;
        return;
    }
    void setLocalResolution(double toSet) {
        this->localResolution = toSet;
        return;
    }
    void setFootprintCoord(int x, int y, int pairChoice, double newCoord); //For setting coordinate data
    void setFootprintLengthData(int x, int y, double dataToSet); //For setting length data
    void resetFootprint(); //Resetting for another transform

private:
    double width;
    double length;
    double localResolution;
    double scaledWidth;
    double scaledLength;
    vector<double> footprintLengthData;
    vector<pair<double, double>> footprint;
};

} //end namespace roverNamespace

#endif // ROVER_H
