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
 */
//ros includes
#include "ros/ros.h"

//operation necessary includes
#include<vector>
#include<deque>
#include<string>
#include <angles/angles.h>
#include"Robot.h"

using namespace std;

//this is an import for the .srv file that has to be written to create a new service
#include "hive_srv/hiveSrv.h"
#include "hive_srv/hiveAddRobot.h"
#include "hive_srv/calibrate.h"
#include "hive_srv/getPosAdjust.h"
#include "hive_srv/setArena.h"
#include "hive_srv/askReturnPermission.h"

//variables declaration
vector<Robot> robotList;
int robotCounter = 0;

//position adjusts
bool positionAdjusted = false;

//return que
deque <string> returnQ;


/*
 * call back method for the service. req contains the request fields that
 * were specified in the .srv file. res contains fields that were specified
 * in the .srv file
*/
bool add(hive_srv::hiveSrv::Request &req, hive_srv::hiveSrv::Response &res){
    res.sum = req.numA + req.numB; //pulls two numbers from request and adds them to sum field in responce
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.numA, (long int)req.numB);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    ROS_INFO("Vector capacity: [%d]", (int)robotList.size());
    return true; //return true if service had a success
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
        int robots = robotList.size();
        ROS_INFO("Start %f:, End: %f, ID: %d, RObots: %d ", startSearchWidth, endSearchWidth, id, robots);
    } else {
        float split = (arenaSize/2)/robotList.size(); // find out how to split
        float startSearchWidth = split * (robotList.size() - id);
        float endSearchWidth = split * (robotList.size() - (id+1));
        res.searchStartWidth = startSearchWidth;
        res.searchEndWidth = endSearchWidth;
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

    //if all robots are calibrated then see if return que is emty
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
                    ROS_INFO("Robot is not first but in que. Dont drop but be ready");
                    res.goDropOff = false;
                    res.goToReadyPos = true;
                    return true;
                }
            }
            res.goDropOff = false;
            res.goToReadyPos = true;
            returnQ.push_back((string)(req.robotName));
        }
    }
    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "hive_server"); //initialize the server (not really important to know what it does)
    ros::NodeHandle n; //create a node handle

    //using the node handle add a new service. The parameters are the name(important) and callback method
    //Use this name to call the service "hive_service_add" (basically will call the add mnethod)
    ros::ServiceServer s1 = n.advertiseService("hive_service_add", add);
    //have to have a ServiceServer even though never used
    ros::ServiceServer s2 = n.advertiseService("hive_add_robot", addRobot);
    //calibration service
    ros::ServiceServer s3 = n.advertiseService("calibration", calibration);

    ros::ServiceServer s4 = n.advertiseService("set_arena", setArena);

    ros::ServiceServer s5 = n.advertiseService("get_pos_adjust", getPosAdjust);

    ros::ServiceServer s6 = n.advertiseService("ask_return_permission", askReturnPermission);

    ROS_INFO("Ready to hive bitchez.");

    ros::spin(); //spin the service as fast as you can
    return 0;
}


























