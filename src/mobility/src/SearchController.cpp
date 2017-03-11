#include "SearchController.h"

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  this->first = true;
  this->numOfItr = 0;
  this->ratio = 0.5;
}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(geometry_msgs::Pose2D currentLocation) {
  geometry_msgs::Pose2D goalLocation;
    if(this->first)
    {
      //currentLocation.theta = 0;
       goalLocation.theta = 0;
      this->first = false;
    }
    else
    {
        //select new heading from Gaussian distribution around current heading
        goalLocation.theta = currentLocation.theta + (3.14/2);//rng->gaussianussian(currentLocation.theta, 0.25);
    }

    if(this->numOfItr == 2){
        this->ratio += 0.5;
        this->numOfItr = 0;
    }

    //select new position 50 cm from current location
    goalLocation.x = currentLocation.x + (ratio * cos(goalLocation.theta));
    goalLocation.y = currentLocation.y + (ratio* sin(goalLocation.theta));
    this->numOfItr++;

    return goalLocation;
}

/**
 * Continues search pattern after interruption. For example, avoiding the
 * center or collisions.
 */
geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
  geometry_msgs::Pose2D newGoalLocation;

  //remainingGoalDist avoids magic numbers by calculating the dist
  double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

  //this of course assumes random walk continuation. Change for diffrent search methods.
  newGoalLocation.theta = oldGoalLocation.theta;
  newGoalLocation.x = currentLocation.x + (0.50 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (0.50 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

  return newGoalLocation;
}
