#include "glball.h"
class World;
//--------------------------------------------------------------------------------------------------------------------------
GLBall::GLBall()
{
    ball_Pos = VecPosition(0,0);
    Ball_Radius = 0.025 ;
}
void GLBall::set_Position(VecPosition _pos)
{
   ball_Pos = _pos ;
}
void GLBall::Draw(World &world)
{
    glColor3d(1,0.3,0);
    glBegin(GL_TRIANGLE_FAN);
    for(double angle = 0.0f ; angle <= 2 * M_PI ; angle += (M_PI)/20.0f)
    {
        double x = Ball_Radius * sin(angle);
        double y = Ball_Radius * cos(angle);
        glVertex3f( ball_Pos.getX()/1000.0f +x , ball_Pos.getY()/1000.0f + y , -7.99999 );
    }
    glEnd();
}
//--------------------------------------------------------------------------------------------------------------------------
