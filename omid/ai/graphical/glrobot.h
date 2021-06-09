#ifndef GLROBOT_H
#define GLROBOT_H
#include <stdio.h>
#include "math.h"
#include "../geometry.h"
#include "../GL/glut.h"
//--------------------------------------------------------------------------------------------------------------------------

class GLRobot
{
public:
    VecPosition position ;
    int my_ID ;
    double angle;
    double Radius ;
    int Is_my_color_yellow ;
	int n;
	int Size_Of_RRT;
	VecPosition* RRT;
    GLRobot(VecPosition _pos = VecPosition(0,0),double angle = 0, int _my_id = 0, int _Is_my_color_yellow = true, VecPosition* rrt=0, int size_of_rrt = 0);
    void set_Robot(VecPosition _pos = VecPosition(0,0),double angle = 0, int _my_id = 0, int _Is_my_color_yellow = true, VecPosition* rrt=0, int size_of_rrt=0);
    void Draw(World &world) ;
};
//--------------------------------------------------------------------------------------------------------------------------
#endif // GLROBOT_H
