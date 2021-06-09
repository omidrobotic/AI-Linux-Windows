#ifndef GLBALL_H
#define GLBALL_H
#include <math.h>
#include "../geometry.h"
#include "/usr/include/GL/glut.h"
//--------------------------------------------------------------------------------------------------------------------------

class GLBall
{
    VecPosition ball_Pos;
    double Ball_Radius ;
public:
    GLBall();
    void set_Position(VecPosition _pos);
    void Draw(World &world) ;
};
//--------------------------------------------------------------------------------------------------------------------------
#endif // GLBALL_H
