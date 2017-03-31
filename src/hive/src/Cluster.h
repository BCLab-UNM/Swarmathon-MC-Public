#ifndef __CLUSTER_H_INCLUDED__   // if x.h hasn't been included yet...
#define __CLUSTER_H_INCLUDED__   //   #define this so the compiler knows it has been included

#include<string>
#include<vector>
#include<ros/ros.h>

class Cluster{


public:
    int maxRobotsAssigned; //max robots allowed to clear the cluster
    int confirmCount; //number of confirms of the cluster
    int notConfirmCount; //number of not confirmes each time we visit a cluster
    std::vector <std::string> assignedRobots; //the robots that are assigned for the job
    float clusterX; //cluster pos
    float clusterY;

    int numberOfRobots; //number of robots in hive

    std::string robotThatFoundCluster;

    Cluster(std::string robotName, float clusterX, float clusterY, int numberOfRobots);
    void addConfirm();
    void addUnconfirm();
    void assignRobot(std::string robotName);
    void unassignRobot(std::string robotName);


};

#endif
