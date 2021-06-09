#ifndef GLFRAME_H
#define GLFRAME_H

//--------------------------------------------------------------------------------------------------------------------------
#include "glball.h"
#include "glfield.h"
#include "glrobot.h"
#include "gltext.h"
#include "glcircle.h"
#include "glline.h"
//--------------------------------------------------------------------------------------------------------------------------
#include "../world.h"
#include "../GL/glut.h"
#include "../Switches.h"
class GLFrame 
{
public:
   
    GLField Field;
    GLBall Ball;
    GLRobot RobotT[MAX_ROBOTS_IN_THE_FIELD] ;
    GLRobot RobotO[MAX_ROBOTS_IN_THE_FIELD] ;
	GLRobot mouse;
	GLLine RRT[World::num_of_vertices];


    int CurrentWidth;
    int CurrentHeight;



    static GLText texts[1000];
    static int num_of_texts;

    static GLCircle circles[1000] ;
    static int num_of_circle ;

    static GLLine lines[1000];
    static int num_of_line;

    static void addCirlceToPainting ( Circle circle, GLfloat _red = 255, GLfloat _green = 0, GLfloat _blue = 0,bool isFilled = true );
    static void addLineToPainting ( VecPosition start, VecPosition end, GLfloat _red = 255, GLfloat _green = 0, GLfloat _blue = 0);
    static void addTextToPainting ( VecPosition pos, string text, GLfloat _red = 255, GLfloat _green = 0, GLfloat _blue = 0);
    void resetPainting();
	static void resetCirclePaintings();
	static void resetLinePaintings();
    void paintPaitingStuff(World &world);


    static VecPosition curser_pos;
    static VecPosition last_curser_click_pos;




    GLFrame() ;
    ~GLFrame();

    
	void mouseCB(World &world,int button, int state, int x, int y);
	void Keyboard(World &world,unsigned char key, int x, int y);
    void initializeGL() ;
    void paintGL(World &world) ;
    void resizeGL(int w ,int h) ;
	void displayText(float x, float y, float z,const string string, float _red = 255.0f, float _green = 0.0f, float _blue = 0.0f);


};

#endif // GLFRAME_H
