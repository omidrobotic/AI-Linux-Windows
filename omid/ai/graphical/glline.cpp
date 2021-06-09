#include "glline.h"


GLLine::GLLine()
{

}

void GLLine::set_Line(VecPosition _start, VecPosition _end, GLfloat _red, GLfloat _green, GLfloat _blue)
{
    start = _start ;
    end = _end ;
	red = _red;
	green = _green;
	blue = _blue;
}

void GLLine::Draw(World &world)
{
	glColor3d(red / 255.0f, green / 255.0f, blue / 255.0f);

    glBegin(GL_LINES);
    glVertex3d(start.getX()/1000.0f ,start.getY()/1000.0f, -8);
    glVertex3d(end.getX()/1000.0f ,end.getY()/1000.0f, -8);
    glEnd();
}

