#ifndef GLCIRCLE_H
#define GLCIRCLE_H

#include "../geometry.h"
#include "../GL/glut.h"

class GLCircle
{
public:
    Circle circle ;
	GLfloat red; GLfloat green; GLfloat blue;
    bool is_filled;
    GLCircle()
    {
    }

    void set_circle(Circle _circle = Circle( VecPosition( 0, 0 ) , 500 ), GLfloat _red=255, GLfloat _green=0, GLfloat _blue=0, bool _is_filled = true );
    void Draw(World &world) ;
};

#endif // GLCIRCLE_H
