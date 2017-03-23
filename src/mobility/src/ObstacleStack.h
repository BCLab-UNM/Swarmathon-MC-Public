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
    geometry_msgs::Pose2D getCurrLocation() {return currentStack.top();}
    geometry_msgs::Pose2D getInterruptedLocation(){return interruptStack.top();}
    geometry_msgs::Pose2D getRestoreLocation(){return restoreStack.top();}
    geometry_msgs::Pose2D getGoalOfInterruption(){return goalOfInterruption.top();}

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

    bool isAtOldGoal(){return atOldGoal.top();}
    bool isLeveled(){return leveled.top();}
    bool isAtRestore(){return atRestore.top();}
    bool isEmpty(){return currentStack.empty();}


};

#endif
