#ifndef __ROBOT_H_INCLUDED__   // if x.h hasn't been included yet...
#define __ROBOT_H_INCLUDED__   //   #define this so the compiler knows it has been included

#include<string>

class Robot{
    std::string name;
    int id;

public:
    Robot(std::string name, int id);
    int getID(){return id;}
    std::string getName(){return name;}
};

#endif
