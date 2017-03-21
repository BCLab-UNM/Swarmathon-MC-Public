#include "Robot.h"

Robot::Robot(std::string name, int id){
    this->name = name;
    this->id = id;
    this->calibrated = false;
    this->posAdjustX = 0;
    this->posAdjustY = 0;
}

