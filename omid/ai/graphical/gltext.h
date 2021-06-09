#ifndef GLTEXT_H
#define GLTEXT_H
#include "../geometry.h"
#include <string>
#include "../GL/glut.h"

class GLText
{
public:
    VecPosition position;
    string text;
	GLfloat red; GLfloat green; GLfloat blue;
    GLText()
    {

    }
    void setText(VecPosition _position = VecPosition(0,0) , string _text = "" , GLfloat _red = 255, GLfloat _green = 0, GLfloat _blue = 0)
    {
        position = _position;
        text = _text;
		red = _red;
		green = _green;
		blue = _blue;
    }

    string Draw(World &world) ;
};

#endif // GLTEXT_H
