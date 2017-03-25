#include "ObstacleStack.h"

void ObstacleStack::addToStack(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D interrupt, geometry_msgs::Pose2D restore, geometry_msgs::Pose2D goalOfInterruption){
    //if stack is not empty eddit top element in stack to account for the new change in position before push
    if(!(this->currentStack.empty())){
        //set the new current location for the top element in the stack.
        geometry_msgs::Pose2D currentOfTop = this->currentStack.top();
        currentOfTop.x = restore.x;
        currentOfTop.y = restore.y;
        this->currentStack.pop();
        this->currentStack.push(currentOfTop);
    }

    this->currentStack.push(currentLocation);
    this->interruptStack.push(interrupt);
    this->restoreStack.push(restore);
    this->goalOfInterruption.push(goalOfInterruption);
    this->atRestore.push(false);
    this->atOldGoal.push(false);
    this->leveled.push(false);
}

void ObstacleStack::popStack(){
    this->currentStack.pop();
    this->interruptStack.pop();
    this->restoreStack.pop();
    this->goalOfInterruption.pop();

    this->atRestore.pop();
    this->atOldGoal.pop();
    this->leveled.pop();
}

