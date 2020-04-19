#include <vector>
#include <cstdlib>
#include <iostream>
#include "rover.h"

namespace roverModelNamespace {
using std::vector;
using std::pair;

rover::rover(double _width, double _length, double _localResoluton)
    :width(_width), length(_length), localResolution(_localResoluton)
{
    this->scaledWidth = width / localResolution;
    this->scaledLength = length / localResolution;

    //Adjust so that double -> int rounding errors are avoided:
    this->scaledWidth = static_cast<int>(this->scaledWidth < 0 ? this->scaledWidth -0.5 : this->scaledWidth + 0.5);
    this->scaledLength = static_cast<int>(this->scaledLength < 0 ? this->scaledLength -0.5 : this->scaledLength + 0.5);
    double placeMe = this->scaledLength * this->scaledWidth;
    int noPlaceME = static_cast<int>(placeMe < 0 ? placeMe -0.5 : placeMe + 0.5); //to account for rounding errors when casting from double to int
    //End adjustment

    //Allocate space for footprint size:
    this->footprint = vector<pair<double, double>>(noPlaceME);
    this->footprintLengthData = vector<double>(noPlaceME);

    //initialize coordinates of rover footprint:
    for (int i = 0; i < scaledWidth; ++i) {
        for (int j = 0; j < scaledLength; ++j) {
            this->setFootprintCoord(i, j, 1, i);
            this->setFootprintCoord(i, j, 2, j);
        }
    }
}

const double rover::getFootprintCoord(int x, int y, int pairChoice) {
    int index = x + scaledWidth * y;
    if (pairChoice == 1) return this->footprint.at(index).first;
    else if (pairChoice ==2) return this->footprint.at(index).second;
    else return -1.0;
}

void rover::setFootprintCoord(int x, int y, int pairChoice, double newCoord) {
    int index = x + scaledWidth * y;
    if (pairChoice == 1) this->footprint.at(index).first = newCoord;
    else if (pairChoice == 2) this->footprint.at(index).second = newCoord;
    return;
}

void rover::setFootprintLengthData(int x, int y, double dataToSet) {
        int index = x + scaledWidth * y;
        this->footprintLengthData.at(index) = dataToSet;
        return;
}

void rover::resetFootprint() {
    //Reset rover footprint's coords:
    for (int i = 0; i < scaledWidth; ++i) {
        for (int j = 0; j < scaledLength; ++j) {
            setFootprintCoord(i, j, 1, i);
            setFootprintCoord(i, j, 2, j);
        }
    }
    return;
}
}
