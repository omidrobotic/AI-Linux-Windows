#ifndef GLLINE_H
#define GLLINE_H

#include "../geometry.h"
#include "../GL/glut.h"

class GLLine
{
public:
    VecPosition start,end ;
	GLfloat red; GLfloat green; GLfloat blue;
    GLLine();
    void set_Line(VecPosition _start, VecPosition _end , GLfloat _red = 255, GLfloat _green = 0, GLfloat _blue = 0) ;
    void Draw(World &world) ;
};

#endif // GLLINE_H
