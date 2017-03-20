#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <geometry_msgs/Pose2D.h>
#include <random_numbers/random_numbers.h>
#include <ros/ros.h>


/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController {

  public:

    SearchController();

    // performs search pattern
    geometry_msgs::Pose2D search(int id, geometry_msgs::Pose2D centerLocation, geometry_msgs::Pose2D currentLocation);

    // continues search pattern after interruption
    geometry_msgs::Pose2D continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation);


  private:
    double ratio;
    int numOfItr;
    random_numbers::RandomNumberGenerator* rng;
    bool first;
    bool stop;
    int direction;


};
#endif /* SEARCH_CONTROLLER */
