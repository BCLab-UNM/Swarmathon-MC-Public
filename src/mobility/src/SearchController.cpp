#include "SearchController.h"
#include <math.h> /*fabs*/

SearchController::SearchController() {
    rng = new random_numbers::RandomNumberGenerator();
    this->init = false;
    this->prelim = false;
    this->numOfItr = 0;
    this->ratio = 0;
    this->started = false;
    this->startingLocation = false;
    this->checkpoint = 0;
    this->shiftX = 0;
    this->shiftY=0;
    this->rover;
    this->offShift = 1.15;

}

geometry_msgs::Pose2D SearchController::search(string robotName, geometry_msgs::Pose2D centerLocation,
                                               geometry_msgs::Pose2D currentLocation, double oTheta) {
    geometry_msgs::Pose2D goalLocation;

    ROS_INFO("heySearch");

    if(!init){
        //make robots move to their starting position
        //pulling information (starting width) from the server
        ros::NodeHandle n;
        ros::ServiceClient setArenaClient = n.serviceClient<hive_srv::setArena>("set_arena");
        hive_srv::setArena srv;
        srv.request.robotName = robotName;
        // this wont twerk :(
        // prelim = srv.response.prelim; // figuring out if we're in prelim or not


        if(setArenaClient.call(srv)){
            startSearchWidth = srv.response.searchStartWidth;
            endSearchWidth = srv.response.searchEndWidth;
            this->prelim = srv.response.prelim;
            ROS_INFO("Values are set");
        }
        else {
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


        // this is assigning IDs to the rovers for non prelim rounds
        if((10* oTheta) >27)
            rover=3; // pi
        else if((10* oTheta) >11)
            rover=4; // pi/2 NOT 3*pi/4 ***show amit
        else if((10 * oTheta) > 4)
            rover=5; // pi/4
        else if((10 * oTheta) >-5)
            rover=6; // 0
        else if((10 * oTheta) >-20)
            rover=1; // -pi/2
        else if((10 * oTheta) >-27)
            rover=2; // -3*pi/4
        else
            rover=3; // -pi

        ROS_INFO("*********just set rover number to %d, ***** looking at %f, *****",rover,oTheta);

        init = true; // set initialization to true because we don't need to go through it again
    } // end of init

    // This block of code executes when there are only three rovers
        // this needs to be set back to true after testing ***
        if (prelim) {

                if(numOfItr == 2){

                    ratio += 0.5;

                    numOfItr = 0;

                }

                //check i = falsef we are at the starting location

                if(!startingLocation){

                    switch(rover){

                    case 3:

                        ratio=12;

                        shiftX= -offShift; shiftY= 0;

                        goalLocation.x = 6+shiftX; //go to start

                        goalLocation.y = 0;

                        goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                        break;

                    case 4:

                        ratio=8;

                        shiftX= 0; shiftY= offShift;

                        goalLocation.x = -4; //go to start

                        goalLocation.y = -3+offShift;

                        goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                        break;

                    case 6:

                        shiftX= offShift; shiftY= 0;

                        ratio=4;

                        goalLocation.x = -2+ shiftX; //go to start

                        goalLocation.y = 1;

                        goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                        break;

                    }

                    startingLocation=true;

                } else { //if not at starting location

                    if(!started){ //if we just started we need to do half a ratio again

                        switch(rover){

                        case 3:

                            goalLocation.x = 6+shiftX; //go to start

                            goalLocation.y = 6;

                            goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                            break;

                        case 4:

                            goalLocation.x = -4; //go to start neg neg

                            goalLocation.y = -4+offShift;;

                            goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                            break;

                        case 6:

                            goalLocation.x = -2+ shiftX; //go to start neg pos

                            goalLocation.y = 2;

                            goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);

                            break;

                        }

                        started = true;

                    } else {

                        switch(rover){

                        case 3:

                            if(currentLocation.theta>2) //looking west

                                goalLocation.theta=-M_PI/2;

                            else if(currentLocation.theta>1) //lookin north

                                goalLocation.theta=M_PI;

                            else if(currentLocation.theta>-1) //looking east

                                goalLocation.theta=M_PI/2;

                            else if(currentLocation.theta>-2) //looking south

                                goalLocation.theta=0;

                            else    //looking west

                                goalLocation.theta=-M_PI/2;

                            break;

                        case 6:

                            if(currentLocation.theta>2) //looking west

                                goalLocation.theta=M_PI/2;

                            else if(currentLocation.theta>1) //lookin north

                                goalLocation.theta=0;

                            else if(currentLocation.theta>-1) //looking east

                                goalLocation.theta=-M_PI/2;

                            else if(currentLocation.theta>-2) //looking south

                                goalLocation.theta=M_PI;

                            else    //looking west

                                goalLocation.theta=M_PI/2;

                            break;

                        case 4:

                            if(currentLocation.theta>2) //looking west

                                goalLocation.theta=-M_PI/2;

                            else if(currentLocation.theta>1) //lookin north

                                goalLocation.theta=M_PI;

                            else if(currentLocation.theta>-1) //looking east

                                goalLocation.theta=M_PI/2;

                            else if(currentLocation.theta>-2) //looking south

                                goalLocation.theta=0;

                            else    //looking west

                                goalLocation.theta=-M_PI/2;

                            break;

                        }

                        goalLocation.x = currentLocation.x + ((ratio) * cos(goalLocation.theta)); //go to direction

                        goalLocation.y = currentLocation.y + ((ratio) * sin(goalLocation.theta));

                        numOfItr++;

                    }

                }

            }

        // for testing, i'm making this execute if prelim is TRUE cause my laptop cant handle 6 rovers ***
        // this algorithm executes when there are 6 rovers
        if (!prelim) {

            //check if we are at the starting location
            if(!startingLocation)
            {
                ROS_INFO("BEFORE STARTING LOCATION IF: ******rover # %d, ****** theta %f , **** x: %f , ******* y: %f ,",
                         rover, (double)(goalLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));
                switch(rover)
                {
                case 1: shiftX= 0; shiftY= -offShift; // (0,1)
                    goalLocation.x = 0 + shiftX;
                    goalLocation.y = 3 + shiftY;
                    break;
                case 2: shiftX= -sqrt(2)*offShift; shiftY= -sqrt(2)*offShift; // (1,1)
                    goalLocation.x = 3 + shiftX;
                    goalLocation.y = 3 + shiftY;
                    break;
                case 3: shiftX= -offShift; shiftY= 0; // (1, -1)
                    goalLocation.x = 3 + shiftX;
                    goalLocation.y = -3 + shiftY;
                    break;
                case 4: shiftX= 0; shiftY= offShift; // (0, -1)
                    goalLocation.x = 0 + shiftX;
                    goalLocation.y = -3 + shiftY;
                    break;
                case 5: shiftX= sqrt(2)*offShift; shiftY= sqrt(2)*offShift; // (-1,-1)
                    goalLocation.x = -3 + shiftX;
                    goalLocation.y = -3 + shiftY;
                    break;
                case 6: shiftX= offShift; shiftY= 0; // (-1,1)
                    goalLocation.x = -3 + shiftX;
                    goalLocation.y = 3 + shiftY ;
                    break;
                }
                goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                ROS_INFO("STARTING LOCATION IF: ******rover # %d, ****** theta %f , **** x: %f , ******* y: %f ,",
                         rover, (double)(goalLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));
                startingLocation = true;

            }
            else
            { //if not at starting location
    //            if(!started)
    //            {
    //                // ROS_INFO("BEFORE STARTING POSITION: ******rover # %d, ****** theta %f , **** x: %f , ******* y: %f ,",
    //                //    rover, (double)(goalLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));

    //                switch(rover)
    //                {
    //                case 1: shiftX= 0; shiftY= -offShift; // (0,1)
    //                    goalLocation.x = 0 + shiftX;
    //                    goalLocation.y = 2 + shiftY;
    //                    break;
    //                case 2: shiftX= -sqrt(2)*offShift; shiftY= -sqrt(2)*offShift; // (1,1)
    //                    goalLocation.x = 2 + shiftX;
    //                    goalLocation.y = 2 + shiftY;
    //                    break;
    //                case 3: shiftX= -offShift; shiftY= 0; // (1, -1)
    //                    goalLocation.x = 2 + shiftX;
    //                    goalLocation.y = -2 + shiftY;
    //                    break;
    //                case 4: shiftX= 0; shiftY= offShift; // (0, -1)
    //                    goalLocation.x = 0 + shiftX;
    //                    goalLocation.y = -2 + shiftY;
    //                    break;
    //                case 5: shiftX= sqrt(2)*offShift; shiftY= sqrt(2)*offShift; // (-1,-1)
    //                    goalLocation.x = -2 + shiftX;
    //                    goalLocation.y = -2 + shiftY;
    //                    break;
    //                case 6: shiftX= offShift; shiftY= 0; // (-1,1)
    //                    goalLocation.x = -2 + shiftX;
    //                    goalLocation.y = 2 + shiftY ;
    //                    break;
    //                }

    //                goalLocation.theta = atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
    //                // ROS_INFO("AFTER STARTING POSITION: ******rover # %d, ****** theta %f , **** x: %f , ******* y: %f ,",
    //                //rover, (double)(goalLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));
    //                started = true;
    //            }
    //            // this used to be just an else statement but it no longer works...
    //            else {

                    ROS_INFO("Start OF CODE: ******rover # %d, ****** theta %f , **** x: %f , ******* y: %f ,",
                    rover, (double)(goalLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));

                    switch(rover) {
                    case 1:
                        checkpoint=numOfItr%2;
                        switch(checkpoint){
                        case 0:
                            goalLocation.y = (currentLocation.y + 1);
                            goalLocation.x = (-goalLocation.y+shiftY);
                            goalLocation.theta =atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                            break;
                        case 1:
                            goalLocation.y = (currentLocation.y + shiftY);
                            goalLocation.x=(0);
                            goalLocation.theta =0;
                            break;
                        }
                        break;

                    case 2:  checkpoint=numOfItr%2;
                        switch(checkpoint){
                        case 0:
                            goalLocation.y = (currentLocation.y + 1);
                            goalLocation.x = (0);
                            goalLocation.theta =atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                            break;
                        case 1:
                            goalLocation.y = (currentLocation.y);
                            goalLocation.x = (currentLocation.y);
                            goalLocation.theta =0;

                            break;
                        }
                        break;
                    case 3:  checkpoint=numOfItr%2;
                        ROS_INFO(":D***********ROVER 3 OUTPUT ONLINE LETS FUCKING GO: checkpoint %d, **********rover %d, ", checkpoint, rover);
                        switch(checkpoint){
                        case 0:
                            goalLocation.x = currentLocation.x;
                            goalLocation.y = fabs(currentLocation.y);
                            goalLocation.theta =M_PI/2;
                            break;
                        case 1:
                            goalLocation.x = (currentLocation.x + 1);
                            goalLocation.y = -(currentLocation.y+1);
                            goalLocation.theta =atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                            break;
                        }
                        break;
                    case 4: checkpoint=numOfItr%2;
                        ROS_INFO(":D***********ROVER 4 OUTPUT ONLINE LETS FUCKING GO: checkpoint %d, **********rover %d, ", checkpoint, rover);

                        switch(checkpoint){
                        case 0:
                            ROS_INFO("C0B4 CURRENT****** theta %f , **** x: %f , ******* y: %f ,",
                                     (double)(currentLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y ));
                            ROS_INFO("C0B4 GOAL****** theta %f , **** x: %f , ******* y: %f ,",
                                     (double)(goalLocation.theta), (double)(goalLocation.x), (double)(goalLocation.y));
                            goalLocation.y = (currentLocation.y);
                            goalLocation.x = (-currentLocation.y+shiftY);
                            goalLocation.theta = 0;
                            ROS_INFO("C0AF CURRENT****** theta %f , **** x: %f , ******* y: %f ,",
                                     (double)(currentLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));
                            ROS_INFO("C0AF GOAL****** theta %f , **** x: %f , ******* y: %f ,",
                                     (double)(goalLocation.theta), (double)(goalLocation.x), (double)(goalLocation.y));

                            break;
                        case 1:
                            ROS_INFO("C1B4 CURRENT****** theta %f , **** x: %f , ******* y: %f ,",
                                     (double)(currentLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y ));
                            ROS_INFO("C1B4 GOAL****** theta %f , **** x: %f , ******* y: %f ,",
                                     (double)(goalLocation.theta), (double)(goalLocation.x), (double)(goalLocation.y));
                            goalLocation.y = (currentLocation.y-1);
                            goalLocation.x= (0);
                            goalLocation.theta =atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                            ROS_INFO("C1AF CURRENT****** theta %f , **** x: %f , ******* y: %f ,",
                                     (double)(currentLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));
                            ROS_INFO("C1AF GOAL****** theta %f , **** x: %f , ******* y: %f ,",
                                     (double)(goalLocation.theta), (double)(goalLocation.x), (double)(goalLocation.y));

                            break;
                        }
                        break;
                    case 5: checkpoint=numOfItr%2;
                        switch(checkpoint){
                        case 0:
                            goalLocation.x = (0);
                            goalLocation.y = (currentLocation.y - 1);
                            goalLocation.theta =atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                            break;
                        case 1:
                            goalLocation.y = (currentLocation.y);
                            goalLocation.x = (goalLocation.y);
                            goalLocation.theta =M_PI;
                            break;
                        }
                        break;
                    case 6:
                        ROS_INFO("*********************************************************************************");
                        checkpoint=numOfItr%2;
                        switch(checkpoint){
                        case 0:
                            //                        ROS_INFO("C0B4 CURRENT****** theta %f , **** x: %f , ******* y: %f ,",
                            //                                 (double)(currentLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y ));
                            //                        ROS_INFO("C0B4 GOAL****** theta %f , **** x: %f , ******* y: %f ,",
                            //                                 (double)(goalLocation.theta), (double)(goalLocation.x), (double)(goalLocation.y));
                            goalLocation.x = currentLocation.x;
                            goalLocation.y = -(currentLocation.y);
                            goalLocation.theta = -M_PI/2;
                            //                        ROS_INFO("C0AF CURRENT****** theta %f , **** x: %f , ******* y: %f ,",
                            //                                 (double)(currentLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));
                            //                        ROS_INFO("C0AF GOAL****** theta %f , **** x: %f , ******* y: %f ,",
                            //                                 (double)(goalLocation.theta), (double)(goalLocation.x), (double)(goalLocation.y));

                            break;
                        case 1:
                            //                        ROS_INFO("C1B4 CURRENT****** theta %f , **** x: %f , ******* y: %f ,",
                            //                                 (double)(currentLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));
                            //                        ROS_INFO("C1B4 GOAL****** theta %f , **** x: %f , ******* y: %f ,",
                            //                                 (double)(goalLocation.theta), (double)(goalLocation.x), (double)(goalLocation.y));
                            goalLocation.x = (currentLocation.x - 1);
                            goalLocation.y =  (fabs(currentLocation.y-1));
                            goalLocation.theta =atan2(goalLocation.y - currentLocation.y, goalLocation.x - currentLocation.x);
                            //                        ROS_INFO("C1AF CURRENT****** theta %f , **** x: %f , ******* y: %f ,",
                            //                                 (double)(currentLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));
                            //                        ROS_INFO("C1AF GOAL****** theta %f , **** x: %f , ******* y: %f ,",
                            //                                 (double)(goalLocation.theta), (double)(goalLocation.x), (double)(goalLocation.y));
                            break;
                        }

                        break;
                  //  }

                }
                    numOfItr++; // increasing number of iterations the rovers went through

            }

            //ROS_INFO("END OF CODE: ******rover # %d, ****** theta %f , **** x: %f , ******* y: %f ,",
            //rover, (double)(goalLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));


            //ROS_INFO("END OF CODE: ******rover # %d, ****** theta %f , **** x: %f , ******* y: %f ,",
            //rover, (double)(goalLocation.theta), (double)(currentLocation.x), (double)(currentLocation.y));

        } // end of non-prelim search

        //ROS_INFO("Returning to transform from searchController");
        return goalLocation;
    }


    /* Continues search pattern after interruption. For example, avoiding the
     * center or collisions.
     */
    geometry_msgs::Pose2D SearchController::continueInterruptedSearch(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D oldGoalLocation) {
        geometry_msgs::Pose2D newGoalLocation;

        //remainingGoalDist avoids magic numbers by calculating the dist
        double remainingGoalDist = hypot(oldGoalLocation.x - currentLocation.x, oldGoalLocation.y - currentLocation.y);

      //this of course assumes random walk continuation. Change for diffrent search methods.
      newGoalLocation.theta = oldGoalLocation.theta;
      newGoalLocation.x = currentLocation.x + (0.8 * cos(oldGoalLocation.theta)); //(remainingGoalDist * cos(oldGoalLocation.theta));
      newGoalLocation.y = currentLocation.y + (0.8 * sin(oldGoalLocation.theta)); //(remainingGoalDist * sin(oldGoalLocation.theta));


        return newGoalLocation;
    }
