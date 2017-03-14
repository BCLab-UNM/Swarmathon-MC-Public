#include "ros/ros.h"
#include "hive/hiveSrv.h" //this is an import for the .srv file that has to be written to create a new service

/**
 * call back method for the service. req contains the request fields that
 * were specified in the .srv file. res contains fields that were specified
 * in the .srv file
*/
bool add(hive::hiveSrv::Request &req, hive::hiveSrv::Response &res){
    res.sum = req.numA + req.numB; //pulls two numbers from request and adds them to sum field in responce
    ROS_INFO("request: x=%ld, y=%ld", (long int)req.numA, (long int)req.numB);
    ROS_INFO("sending back response: [%ld]", (long int)res.sum);
    return true; //return true if service had a success
}

int main(int argc, char **argv){
    ros::init(argc, argv, "hive_server"); //initialize the server (not really important to know what it does)
    ros::NodeHandle n; //create a node handle

    //using the node handle add a new service. The parameters are the name(important) and callback method
    //Use this name to call the service "hive_service_add" (basically will call the add mnethod)
    n.advertiseService("hive_service_add", add);
    ROS_INFO("Ready to add two ints.");

    ros::spin(); //spin the service as fast as you can
    return 0;
}


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















