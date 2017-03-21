#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <string>
#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>
#include "hive_srv/setArena.h"
#include "hive_srv/getPosAdjust.h"

using namespace std;


/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController {

  public:

    SearchController();

    // performs search pattern
    geometry_msgs::Pose2D search(string robotName, geometry_msgs::Pose2D centerLocation, geometry_msgs::Pose2D currentLocation);

    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);

    //set the next step size for the robot
    void setStepSize();


  private:
    bool init;

    double ratio;
    int numOfItr;
    random_numbers::RandomNumberGenerator* rng;
    bool started;

    float startSearchWidth;
    float endSearchWidth;
    bool startingLocation;

    float posAdjustX;
    float posAdjustY;

};
#endif /* SEARCH_CONTROLLER */
