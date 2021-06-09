#include "glrobot.h"
#include "../Switches.h"
class GLFrame;
//--------------------------------------------------------------------------------------------------------------------------
GLRobot::GLRobot(VecPosition _pos, double _angle, int _my_id, int _Is_my_color_yellow, VecPosition* rrt, int size_of_rrt)
{

    Radius = 0.09 ;
    set_Robot(_pos,_angle,_my_id,_Is_my_color_yellow,rrt, size_of_rrt);
}
void GLRobot::set_Robot(VecPosition _pos, double _angle, int _my_id, int _Is_my_color_yellow, VecPosition* rrt,int size_of_rrt)
{
    position = _pos;
    angle = _angle ;
    my_ID = _my_id;
    Is_my_color_yellow = _Is_my_color_yellow ;
#if DRAW_RRT_PATHS == 1
	RRT = rrt;
	Size_Of_RRT = size_of_rrt;
#endif
}
void GLRobot::Draw(World &world)
{
    double ang ;
    if(Is_my_color_yellow==1)
        glColor3d(1,0.8,0) ;
    else if (Is_my_color_yellow == 0)
        glColor3d(0,0,0.80);
	else
		glColor3d(1, 0, 0);

    glBegin(GL_LINE_LOOP);
    for(ang = angle + (3 * M_PI) / 4 + M_PI ; ang <= 9 *( M_PI / 4 ) + angle + M_PI; ang += (M_PI)/25)
    {
        double x = Radius * sin(-ang);
        double y = Radius * cos(-ang);
        glVertex3f(x+position.getX() / 1000.0f , y+position.getY() / 1000.0f , -8);
    }
    glEnd();

    if(Is_my_color_yellow==1)
        glColor3d(1,1,0.3) ;
	else if (Is_my_color_yellow == 0)
        glColor3d(0.28,0.46,1);
	else
		glColor3d(0.8, 0, 0);

    glBegin(GL_TRIANGLE_FAN);
    for(ang = angle + (3 * M_PI) / 4 + M_PI ; ang <= 9 *( M_PI / 4 ) + angle + M_PI; ang += (M_PI)/25)
    {
        double x = Radius * sin(-ang);
        double y = Radius * cos(-ang);
        glVertex3f( x+position.getX() / 1000.0f , y+position.getY() / 1000.0f , -8);
    }
    glEnd();

	///drawing rrt path
#if DRAW_RRT_PATHS == 1
	if (Size_Of_RRT > 0)
	{
		glColor3d(1, 0, 0);
		glBegin(GL_LINES);
		for (n = 0;n < Size_Of_RRT;n++)
		{
			glVertex3f(RRT[n].getX() / 1000.0f, RRT[n].getY() / 1000.0f, -7.99999);
			glVertex3f(RRT[n+1].getX() / 1000.0f, RRT[n+1].getY() / 1000.0f, -7.99999);
		}
		glEnd();
	}
#endif

}
//--------------------------------------------------------------------------------------------------------------------------
