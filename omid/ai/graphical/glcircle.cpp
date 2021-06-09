#include "glcircle.h"


void GLCircle::set_circle(Circle _circle, GLfloat _red, GLfloat _green, GLfloat _blue, bool _is_filled)
{
    circle = _circle ;
	red = _red;
	green =_green;
	blue =_blue;
    is_filled = _is_filled ;
}



void GLCircle::Draw(World &world)
{
    double angle ;

    glColor3d( red / 255.0f, green/ 255.0f, blue / 255.0f );

    if(is_filled)
    {
        glBegin(GL_TRIANGLE_FAN);
        for(angle = 0.0f; angle <= 2.0 * M_PI; angle += (M_PI/20.0f))
        {
            double x = circle.getRadius() / 1000.0f * sin(angle);
            double y = circle.getRadius() / 1000.0f * cos(angle);
            glVertex3f(x + circle.getCenter().getX() / 1000.0f, y + circle.getCenter().getY() / 1000.0f, -8);
        }
        glEnd();
    }
    else
    {
        glBegin(GL_LINE_LOOP);
        for(angle = 0.0f; angle <= 2.0 * M_PI; angle += (M_PI/20.0f))
        {
            double x = circle.getRadius() / 1000.0f * sin(angle);
            double y = circle.getRadius() / 1000.0f * cos(angle);
            glVertex3f(x + circle.getCenter().getX() / 1000.0f, y + circle.getCenter().getY() / 1000.0f, -8);
        }
        glEnd();
    }
}
