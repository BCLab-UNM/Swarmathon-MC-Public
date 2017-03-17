#include "Robot.h"

Robot::Robot(std::string name, int id){
    this->name = name;
    this->id = id;
    calibrated = false;
}

