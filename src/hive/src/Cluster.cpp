#include "Cluster.h"


Cluster::Cluster(std::string robotName, float clusterX, float clusterY, int numberOfRobots){
    this->robotThatFoundCluster = robotName;

    assignedRobots.push_back(robotName);
    this->clusterX = clusterX;
    this->clusterY = clusterY;

    this->confirmCount = 0;
    this->notConfirmCount = 0;
    this->maxRobotsAssigned = 1;

    this->numberOfRobots = numberOfRobots;
}

//this method adds a confirm and calculates how many robots can work on a cluster
void Cluster::addConfirm(){
    this->confirmCount++;

    //if the amound of confirms can be divided by the amound of robots and we get a whole number
    if(confirmCount % numberOfRobots == 0){
        //add more robots if we can
        if((int)(confirmCount/numberOfRobots) - numberOfRobots <= notConfirmCount){
            if(maxRobotsAssigned < numberOfRobots)
                maxRobotsAssigned++;
            ROS_INFO("Adding confirm. Confirm count is: %d, numbers of robots is %d:", confirmCount, maxRobotsAssigned);
        }

    }
}

//if cluster has not been confirmed
void Cluster::addUnconfirm(){
    this->notConfirmCount++; //increase the not confirmed cound
    if(this->maxRobotsAssigned > 0){  //if the mas robot value is greater that 0
        this->maxRobotsAssigned--; //reduce the robot count
        ROS_INFO("Adding unconfirm. Unconfirm count is %d, Confirm count is: %d, numbers of robots is %d:", notConfirmCount, confirmCount, maxRobotsAssigned);
    }

    //when max robot count hits 0 the hive will remove the cluster from its list
}

void Cluster::assignRobot(std::string robotName){
    if(assignedRobots.size() < maxRobotsAssigned){
        this->assignedRobots.push_back(robotName);
        ROS_INFO("Assigning robot %s:",robotName.c_str());
    }
}

void Cluster::unassignRobot(std::string robotName){
    if(this->robotThatFoundCluster != robotName){ //if it is not the robot that found the cluster
        for(int i = 0; i<assignedRobots.size(); i++){ //iterate through the robot list
            if((std::string)(assignedRobots[i]) == robotName){ //if robot name matches
                assignedRobots.erase(assignedRobots.begin() + i); //remove the robot
                ROS_INFO("Unassigning robot %s:",robotName.c_str());
            }
        }
    }
}


