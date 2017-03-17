#ifndef __ROBOT_H_INCLUDED__   // if x.h hasn't been included yet...
#define __ROBOT_H_INCLUDED__   //   #define this so the compiler knows it has been included

#include<string>

class Robot{


public: //bad practive to make everyhting publick but works faster
    std::string name;
    int id;
    bool calibrated;

    Robot(std::string name, int id);


};

#endif
