/* TO USE SERVICE IN ANY CLASS*/
/* FIRST: import the server (#import "hive/HiveSrv.h")
 * SECOND: declaire a service client (ros::ServiceClient client;)
 * THIRD: init the client where the node handler is (client = nodeHandler.serviceClient<hive::hiveSrv>("hive_service_add");) <---name is used(important)
 * FOURTH: Service is initialized and ready to use
 * FIFTH: USe the service
 *              //create a service
 *              hive::hiveSrv srv;
 *              //put numbers in request
 *              srv.request.numA = atoll("1"); <--atoll() converts string to int
 *              srv.request.numB = atoll("2");
 *
 *              if(client.call(srv)){   <--make a call using client created in step 2 and 3
 *                  ROS_INFO("Sum: %ld", srv.response.sum); //if sucess get responce values
 *              } else {
 *                  ROS_INFO("Failed to call service add_two_ints");
 *                  //return;
 *              }
 *
 *
 * manual manfffffual manul
 *
 */
//ros includes
#include "ros/ros.h"

//operation necessary includes
#include<vector>
#include<deque>
#include<string>
#include <angles/angles.h>
#include"Robot.h"
#include"Cluster.h"

using namespace std;

//this is an import for the .srv file that has to be written to create a new service
#include "hive_srv/hiveSrv.h"
#include "hive_srv/hiveAddRobot.h"
#include "hive_srv/calibrate.h"
#include "hive_srv/getPosAdjust.h"
#include "hive_srv/setArena.h"
#include "hive_srv/foundCluster.h"
#include "hive_srv/getHeading.h"
#include "hive_srv/askReturnPermission.h"

//variables declaration
vector<Robot> robotList;
int robotCounter = 0;

//position adjusts
bool positionAdjusted = false;

//return que
deque <string> returnQ;
ros::Time timeInFront;

//cluster list
vector<Cluster> clusterList;

/*
 * call back method for the service. req contains the request fields that
 * were specified in the .srv file. res contains fields that were specified
 * in the .srv file
*/
bool notifyAboutCluster(hive_srv::foundCluster::Request &req, hive_srv::foundCluster::Response &res){
    //find position adjust for the robot that is requesting
    float x = 0; //x adjust
    float y = 0; //y adjust
    for(int j = 0; j<robotList.size(); j++){
        //if the name is equal
        if(((Robot)robotList[j]).name == ((string)req.robotName)){
            x = ((Robot)robotList[j]).posAdjustX;
            y = ((Robot)robotList[j]).posAdjustY;
            break;
        }
    }
    //go through the list of clusters
    for(int i = 0; i<clusterList.size(); i++){
        float existingClusterMinX = ((Cluster)clusterList[i]).clusterX - 1.5;
        float existingClusterMaxX = ((Cluster)clusterList[i]).clusterX + 1.5;

        float existingClusterMinY = ((Cluster)clusterList[i]).clusterY - 1.5;
        float existingClusterMaxY = ((Cluster)clusterList[i]).clusterY + 1.5;



        //check if x is one meter away from the cluster
        if(((float)req.posX)-x >= existingClusterMinX && ((float)req.posX)-x <= existingClusterMaxX){
            //if it is one meter away
            //check of y is one away
            if(((float)req.posY)-y >= existingClusterMinY && ((float)req.posY)-y <= existingClusterMaxY){
                //if it is then cluster is the same as in the list.
                Cluster & c = clusterList[i];
                if(req.targetCount>=6){
                    //increase confirm count in list
                    ROS_INFO("Added confirm to cluster");
                    c.addConfirm();
                    return true;
                } else { //if didnt see enough targets while pick up and we are close to cluster
                    //increase not confirm.
                    c.addUnconfirm();
                    if(c.maxRobotsAssigned == 0){ //if cluster is useless
                        //kick it out of the house
                        clusterList.erase(clusterList.begin() + i);
                        ROS_INFO("Erasing useless cluster");
                        return true;
                    }
                }
            }
        }
    }

    //if for loop didnt have any clusters or the cluster found is a diferent location
    //create a new cluster
    //fid the robot in the robot list to get his offset
    x = 0; //x adjust
    y = 0; //y adjust
    for(int i = 0; i<robotList.size(); i++){
        //if the name is equal
        if(((Robot)robotList[i]).name == ((string)req.robotName)){
            x = ((Robot)robotList[i]).posAdjustX;
            y = ((Robot)robotList[i]).posAdjustY;
            break;
        }
    }
    //put an adjusted position cluster in list
    Cluster c((string)req.robotName, ((float)req.posX)-x, ((float)req.posY)-y, robotList.size());
    clusterList.push_back(c);
    ROS_INFO("#############Added new cluster pos X:%f, Y:%f", ((float)req.posX)-x, ((float)req.posY)-y);
    return true;
}

bool addRobot(hive_srv::hiveAddRobot::Request &req, hive_srv::hiveAddRobot::Response &res){
    Robot r(req.robotName, robotCounter);
    res.robotIdInHive = robotCounter;
    robotCounter++;


    robotList.push_back(r);
    ROS_INFO("Heading for robot=%s, heading=%f", ((std::string)req.robotName).c_str(), (double)req.currTheta);
    ROS_INFO("request: Name=%s, ID=%ld", ((std::string)req.robotName).c_str(), (long int)res.robotIdInHive);
    return true;

}

bool calibration(hive_srv::calibrate::Request &req, hive_srv::calibrate::Response &res){
    string name = (string)req.robotName;
    for(int i = 0; i<robotList.size(); i++){
        if(name == ((Robot)robotList.at(i)).name){ //find the robot in list
            if(req.calibratedOnStart == false){ //if calibration is not finished
                if(i == 0){
                    res.calibrate = true;
                    ROS_INFO("Calibrating robot named: %s", ((std::string)req.robotName).c_str());
                    return true;
                } else if(((Robot)robotList.at(i-1)).calibrated == true) {
                    ROS_INFO("Calibrating robot named: %s", ((std::string)req.robotName).c_str());
                    res.calibrate = true;
                    res.positionAdjusted = positionAdjusted;
                    return true;
                } else {
                    //ROS_INFO("Not calibrating robot named: %s", ((std::string)req.robotName).c_str());
                    res.calibrate = false;
                    return true;
                }
            } else {
                Robot &r = robotList.at(i);
                if(!r.calibrated){
                    r.posAdjustX = req.currLocationX; //set the adjusts if in the center
                    r.posAdjustY = req.currLocationY;
                    r.calibrated = true;
                    ROS_INFO("Setting adjustment for: %s, x: %f. y: %f", ((std::string)req.robotName).c_str(), (float)req.currLocationX, (float)req.currLocationY);
                }
                return true;
            }
        }
    }
    return false;
}

bool setArena(hive_srv::setArena::Request &req, hive_srv::setArena::Response &res){
    string name = req.robotName;
    float startSearchWidth;
    int id = 0;
    float arenaSize = 15;
    for(int i = 0; i<robotList.size(); i++){
        if(name == ((Robot)robotList.at(i)).name){ //find the robot in list
            Robot r = robotList[i];
            id = i;
            //ROS_INFO("Robot ID found in serArena: %d, in robot: %d", i, r.id);
        }
    }

    //find out arena size
    if(robotList.size() <= 4){ //if arena has 3 robots
        arenaSize = 15;
        //res.prelim = true;
    } else if(robotList.size() >= 5){ //if arena has 6 robots
        arenaSize = 22;
        //res.prelim = false;
    }

    //find half the arena
    arenaSize = arenaSize - 3; // Shrinking the arena size because no cubes are lined up along the edge
    if(robotList.size() <= 4){

        //split arena in 3

        float split = (arenaSize/2)/robotList.size(); // find out how to split
        startSearchWidth = split * (robotList.size() - id);
        float endSearchWidth = split * (robotList.size() - (id+1));
        res.searchStartWidth = startSearchWidth;
        res.searchEndWidth = endSearchWidth;
        res.prelim = true;
        int robots = robotList.size();
        ROS_INFO("Start %f:, End: %f, ID: %d, RObots: %d ", startSearchWidth, endSearchWidth, id, robots);
    } else {
        float split = (arenaSize/2)/robotList.size(); // find out how to split
        float startSearchWidth = split * (robotList.size() - id);
        float endSearchWidth = split * (robotList.size() - (id+1));
        res.searchStartWidth = startSearchWidth;
        res.searchEndWidth = endSearchWidth;
        res.prelim = false;
        int robots = robotList.size();
        ROS_INFO("Start %f:, End: %f, ID: %d, RObots: %d ", startSearchWidth, endSearchWidth, id, robots);
    }

    return true;
}


bool getPosAdjust(hive_srv::getPosAdjust::Request &req, hive_srv::getPosAdjust::Response &res){
    string name = req.robotName;
    for(int i = 0; i<robotList.size(); i++){
        if(name == ((Robot)robotList.at(i)).name){ //find the robot in list
            Robot &r = robotList.at(i);
            res.posAdjustX = r.posAdjustX;
            res.posAdjustY = r.posAdjustY;
        }

    }

    return true;
}

bool askReturnPermission(hive_srv::askReturnPermission::Request &req, hive_srv::askReturnPermission::Response &res){
    //first check if all the robots are calibrated. If not nobody can return
    for(int i = 0; i<robotList.size(); i++){
        if(!((Robot)robotList[i]).calibrated){ //if we found at least one robot not calibrated
            res.goDropOff = false;
            res.goToReadyPos = false;
            return true;
        }
    }

    //if Q is empty tell the first rover to just go to ready
    if(returnQ.empty()){
        //if robot is at readyPOS
        if(req.atReady){
            //put it in que
            timeInFront = ros::Time::now();
            returnQ.push_back((string)(req.robotName));
        } else {
            //tell the robot to go to ready
            res.goToReadyPos = true;
            res.goDropOff = false;
        }
    } else {
        if(ros::Time::now().toSec() - timeInFront.toSec() >=15){
            returnQ.pop_front();
            timeInFront = ros::Time::now();
        }

        //check if requested name matches the next robot in que
        if((string)(returnQ.front()) == (string)(req.robotName)){
            ROS_INFO("Robot is first. Is ready?");
            //check if robot is at ready pos
            res.goDropOff = true;
            if((bool)(req.droppedOff)) {//if robot dropped
                returnQ.pop_front(); //pop it
                timeInFront = ros::Time::now();
            }
        } else {
            //check if requested name matches the first robot in Q
            if((string)(returnQ.front()) == (string)(req.robotName)){
                ROS_INFO("Robot is first. Go Drop");
                res.goDropOff = true;
                if((bool)(req.droppedOff)){
                    returnQ.pop_front();
                    timeInFront = ros::Time::now();
                }
            } else {
                //check if robot is in the que
                for(int i = 0; i< returnQ.size(); i++){
                    if((string)(returnQ[i]) == (string)(req.robotName)){
                        //if in que just tell it to go to ready
                        //ROS_INFO("Robot is not first but in que. Dont drop but be ready");
                        res.goDropOff = false;
                        res.goToReadyPos = true;
                        return true;
                    }
                }

                //if robot is not in the queue
                //check if it is in a readyPOS
                if(req.atReady){
                    //put it in Q
                    returnQ.push_back((string)(req.robotName));
                    return true; //return
                }
                //if robot is not at ready tell it to go to ready
                res.goDropOff = false;
                res.goToReadyPos = true;

            }
        }
    }


    /*//if all robots are calibrated then see if return que is emty
    if(returnQ.empty()){
        ROS_INFO("Putting in que");
        //tell the robot to go drop off
        res.goToReadyPos = true;
        res.goDropOff = false;
        returnQ.push_back((string)(req.robotName));
    } else {
        //check if requested name matches the next robot in que
        if((string)(returnQ.front()) == (string)(req.robotName)){
            ROS_INFO("Robot is first. Go Drop");
            res.goDropOff = true;
            if((bool)(req.droppedOff))
                returnQ.pop_front();
        } else {
            //check if robot is in the que
            for(int i = 0; i< returnQ.size(); i++){
                if((string)(returnQ[i]) == (string)(req.robotName)){
                    //if in que just tell it to go to ready
                    //ROS_INFO("Robot is not first but in que. Dont drop but be ready");
                    if(!req.droppedOff){
                        res.goDropOff = false;
                        res.goToReadyPos = true;
                        return true;
                    } else {
                        returnQ.clear();
                    }
                }
            }
            res.goDropOff = false;
            res.goToReadyPos = true;
            returnQ.push_back((string)(req.robotName));
        }
    }*/
    return true;
}

bool getHeading(hive_srv::getHeading::Request &req, hive_srv::getHeading::Response &res){
    if(clusterList.empty()){
        res.headingIsSet = false;
        ROS_INFO("There is no heading yet. Go do what you were doing looser");
        return true;
    } else {
        int shortesIndex = 0;
        float shortestDistance = 1000000000; //set big distance so that first element will deff have shorter distance
        //go through the list
        for(int i = 0; i<clusterList.size(); i++){
            //find shortest distance heading
            double distance = fabs(hypot(((Cluster)clusterList[i]).clusterX - ((float)req.posX), ((Cluster)clusterList[i]).clusterY - ((float)req.posY)));
            if(((Cluster)clusterList[i]).robotThatFoundCluster == ""){
                //there is no robot assigned. Can send
                Cluster & c = clusterList[i];
                c.robotThatFoundCluster = ((string)req.robotName);
                shortesIndex = i;
                break;
            } else {
                //check if distance is good
                if(distance < shortestDistance){
                    shortesIndex = i;
                }
            }
        }

        shortestDistance = 1000000000; //set big distance so that first element will deff have shorter distance
        for(int i = 0; i<clusterList.size(); i++){
            if(((Cluster)clusterList[i]).robotThatFoundCluster == ((string)req.robotName)){
                double distance = fabs(hypot(((Cluster)clusterList[i]).clusterX - ((float)req.posX), ((Cluster)clusterList[i]).clusterY - ((float)req.posY)));
                if(distance < shortestDistance){
                    shortesIndex = i;
                }
            }
        }


        //find position adjust for the robot that is requesting
        float x = 0; //x adjust
        float y = 0; //y adjust
        for(int i = 0; i<robotList.size(); i++){
            //if the name is equal
            if(((Robot)robotList[i]).name == ((string)req.robotName)){
                x = ((Robot)robotList[i]).posAdjustX;
                y = ((Robot)robotList[i]).posAdjustY;
                break;
            }
        }



        res.headingX = ((Cluster)clusterList[shortesIndex]).clusterX + x;
        res.headingY = ((Cluster)clusterList[shortesIndex]).clusterY + y;
        res.headingIsSet = true;
        ROS_INFO("#############Sending to cluster at X:%f, Y:%f", ((float)res.headingX), ((float)res.headingY));
        Cluster &c = clusterList[shortesIndex];
        c.assignRobot(((string)req.robotName));
    }


    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "hive_server"); //initialize the server (not really important to know what it does)
    ros::NodeHandle n; //create a node handle

    //using the node handle add a new service. The parameters are the name(important) and callback method
    //Use this name to call the service

    //tell the herver that there is a cluster
    ros::ServiceServer s1 = n.advertiseService("notify_about_cluster", notifyAboutCluster);
    //have to have a ServiceServer even though never used
    ros::ServiceServer s2 = n.advertiseService("hive_add_robot", addRobot);
    //calibration service
    ros::ServiceServer s3 = n.advertiseService("calibration", calibration);

    ros::ServiceServer s4 = n.advertiseService("set_arena", setArena);

    ros::ServiceServer s5 = n.advertiseService("get_pos_adjust", getPosAdjust);

    ros::ServiceServer s6 = n.advertiseService("ask_return_permission", askReturnPermission);

    ros::ServiceServer s7 = n.advertiseService("get_heading", getHeading);


    ROS_INFO("Ready to hive bitchez.");

    ros::spin(); //spin the service as fast as you can
    return 0;
}


























