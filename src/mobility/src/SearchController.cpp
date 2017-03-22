#include "SearchController.h"



SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  this->init = false;
  this->numOfItr = 0;
  this->ratio = 0;
  this->started = false;
  this->startingLocation = false;

}

/**
 * This code implements a basic random walk search.
 */
geometry_msgs::Pose2D SearchController::search(string robotName, geometry_msgs::Pose2D centerLocation, geometry_msgs::Pose2D currentLocation) {
  geometry_msgs::Pose2D goalLocation;

   ROS_INFO("heySearch");

    /*if(this->first)
    {
      //currentLocation.theta = 0;
      goalLocation.theta = (3.14/2)*counter;
      this->first = false;
      this->stop = false;
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

        if(this->numOfItr == 2){
            this->ratio -= 0.5;
            this->numOfItr = 0;
        }

        /*if(this->ratio >= 2){
            if(!stop){
                //code to go back to center
                //set angle to center as goal heading
                goalLocation.theta = atan2(0 - currentLocation.y, 0 - currentLocation.x);

                //set center as goal position
                goalLocation.x = 0;
                goalLocation.y = 0;
                //spinWasTrue = true; only turn on for random walk to center
                stop = true;
            }
        } else {
        if(ratio <= endSearchWidth){
            stop = true;
            return currentLocation;
        } else {
            if(first){
                this->first = false;
                //select new position 50 cm from current location
                goalLocation.x = currentLocation.x + (ratio/2 * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + (ratio/2 * sin(goalLocation.theta));
            } else {
                goalLocation.x = currentLocation.x + (ratio * cos(goalLocation.theta));
                goalLocation.y = currentLocation.y + (ratio * sin(goalLocation.theta));
            }

            this->numOfItr++;
        }
    } else {
        //select new position 50 cm from current location
        goalLocation.x = currentLocation.x + (ratio * cos(goalLocation.theta));
        goalLocation.y = currentLocation.y + (ratio* sin(goalLocation.theta));
        this->numOfItr++;
*/
    /*if(!started){
        goalLocation.x = 0;
        goalLocation.y = 4;
        goalLocation.theta = goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
        started = true;
    } else {
        goalLocation.theta = currentLocation.theta + M_PI/2; //turn around
        goalLocation.x = (currentLocation.x) + (4 * cos(goalLocation.theta)); //go to start
        goalLocation.y = (currentLocation.y) + (4 * sin(goalLocation.theta));
    }*/





   //set next step size
   //if the distance is too far it is probably better to go from the end to start.

   if(!init){
       //make robots move to their starting position
       //pull the starting width from the server
       ros::NodeHandle n;
       ros::ServiceClient setArenaClient = n.serviceClient<hive_srv::setArena>("set_arena");
       hive_srv::setArena srv;
       srv.request.robotName = robotName;
       if(setArenaClient.call(srv)){
           startSearchWidth = srv.response.searchStartWidth;
           endSearchWidth = srv.response.searchEndWidth;
           startSearchWidth = 2;
           endSearchWidth = 1;
       } else {
           ROS_INFO("Could not call set arena client");
       }

       //set the adjusts
       ros::ServiceClient getPosAdjust = n.serviceClient<hive_srv::getPosAdjust>("get_pos_adjust");
       hive_srv::getPosAdjust psrv;
       psrv.request.robotName = robotName;
       if(getPosAdjust.call(psrv)){
           this->posAdjustX = psrv.response.posAdjustX;
           this->posAdjustY = psrv.response.posAdjustY;

           ROS_INFO("Adjusts set for: %s, X: %f, Y: %f", robotName.c_str(), posAdjustX, posAdjustY);
       } else {
           ROS_INFO("Could not call position adjust client");
       }

       ratio = startSearchWidth * 2;


       init = true;
   } else {
       if(numOfItr == 2){
           ratio -= 0.5;
           numOfItr = 0;
       }


   }

   //check if we are at the starting location
   if(!startingLocation){
       goalLocation.theta = currentLocation.theta + M_PI; //turn 180
       goalLocation.x = currentLocation.x + (ratio/2 * cos(goalLocation.theta)); //go to start
       goalLocation.y = currentLocation.y + (ratio/2 * sin(goalLocation.theta));
       startingLocation = true;
   } else { //in not at starting location
       if(!started){ //if we just started we need to do half a ratio again
           goalLocation.theta = currentLocation.theta + M_PI/2; //turn 90
           goalLocation.x = currentLocation.x + (ratio/2 * cos(goalLocation.theta)); //go to direction
           goalLocation.y = currentLocation.y + (ratio/2 * sin(goalLocation.theta));
           started = true;
       } else {
           goalLocation.theta = currentLocation.theta + M_PI/2; //turn 90
           goalLocation.x = currentLocation.x + ((ratio) * cos(goalLocation.theta)); //go to direction
           goalLocation.y = currentLocation.y + ((ratio) * sin(goalLocation.theta));
           numOfItr++;
       }

   }

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
  newGoalLocation.x = currentLocation.x + (1 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
  newGoalLocation.y = currentLocation.y + (1 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));

  return newGoalLocation;
}
