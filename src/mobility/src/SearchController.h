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
 * here should be modified and enhanced to improve search performance.2
 */
class SearchController {

  public:

    SearchController();

    // performs search pattern
    geometry_msgs::Pose2D search(string robotName, geometry_msgs::Pose2D centerLocation,
                                 geometry_msgs::Pose2D currentLocation, double oTheta);

    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);

    //set the next step size for the robot
    void setStepSize();


  private:
    bool init; // keeps track if this is the first iteration of the search method
    bool prelim; // Boolean that detects if there are 3 or 6 rovers
    bool started;

    double ratio;
    double offShift; // the offshift a rover recieves based off their own 0,0. This shift will be applied to x, y, or both
    double shiftY; // the y shift a rover recieves based off their own (0,0)
    double shiftX; // the x shift a rover recieves based off their own (0,0)

    int numOfItr; // keeps track of how many iterations the rovers have done
    int checkpoint; // regulates which way the rovers turn based off which checkpoint they're at (e.g, at y = -x or x = 0)
    int rover; // ID number given to a specific rover

    random_numbers::RandomNumberGenerator* rng;


    float startSearchWidth;
    float endSearchWidth;
    bool startingLocation;

    float posAdjustX;
    float posAdjustY;

};
#endif /* SEARCH_CONTROLLER */
