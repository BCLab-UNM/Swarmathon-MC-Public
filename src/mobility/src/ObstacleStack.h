#ifndef __OBSTACLESTACK_H_INCLUDED__   // if x.h hasn't been included yet...
#define __OBSTACLESTACK_H_INCLUDED__   //   #define this so the compiler knows it has been included

#include<string>
#include <geometry_msgs/Pose2D.h>
#include <stack>


class ObstacleStack{
    std::stack<geometry_msgs::Pose2D> currentStack;
    std::stack<geometry_msgs::Pose2D> interruptStack;
    std::stack<geometry_msgs::Pose2D> restoreStack;
    std::stack<geometry_msgs::Pose2D> goalOfInterruption;
    std::stack<bool> atRestore;
    std::stack<bool> atOldGoal;
    std::stack<bool> leveled;

public:
    void addToStack(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D interrupt, geometry_msgs::Pose2D restore, geometry_msgs::Pose2D goalOfInterruption);
    void popStack(); //pop the stack
    //get the lcations that are being worked on
    geometry_msgs::Pose2D getCurrLocation() {return (geometry_msgs::Pose2D)currentStack.top();}
    geometry_msgs::Pose2D getInterruptedLocation(){return (geometry_msgs::Pose2D)interruptStack.top();}
    geometry_msgs::Pose2D getRestoreLocation(){return (geometry_msgs::Pose2D)restoreStack.top();}
    geometry_msgs::Pose2D getGoalOfInterruption(){return (geometry_msgs::Pose2D)goalOfInterruption.top();}

    void setRestore(){
        this->atRestore.pop();
        this->atRestore.push(true);
    }
    void setOldGoal(){
        this->atOldGoal.pop();
        this->atOldGoal.push(true);
    }
    void setLeveled(){
        this->leveled.pop();
        this->leveled.push(true);
    }

    void unsetRestore(){
        this->atRestore.pop();
        this->atRestore.push(false);
    }
    void unsetOldGoal(){
        this->atOldGoal.pop();
        this->atOldGoal.push(false);
    }
    void unsetLeveled(){
        this->leveled.pop();
        this->leveled.push(false);
    }

    bool isAtOldGoal(){return (bool)atOldGoal.top();}
    bool isLeveled(){return (bool)leveled.top();}
    bool isAtRestore(){return (bool)atRestore.top();}
    bool isEmpty(){return (bool)currentStack.empty();}

    void resetInterruptedLocation(geometry_msgs::Pose2D location){
        this->interruptStack.pop();
        this->interruptStack.push(location);
    }

    void resetGoalOfInterruption(geometry_msgs::Pose2D location){
        this->goalOfInterruption.pop();
        this->goalOfInterruption.push(location);
    }



};

#endif
