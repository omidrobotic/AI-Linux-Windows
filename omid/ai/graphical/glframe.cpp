

#include "glframe.h"
#include<stdio.h>
#include<string.h>
#include "../Switches.h"

VecPosition GLFrame::curser_pos;
VecPosition GLFrame::last_curser_click_pos;

GLText GLFrame::texts[1000];
int GLFrame::num_of_texts;

GLCircle GLFrame::circles[1000] ;
int GLFrame::num_of_circle ;

GLLine GLFrame::lines[1000];
int GLFrame::num_of_line;


GLFrame::GLFrame()
{

	//paintGL();
    num_of_texts = 0;
    num_of_circle = 0;
    num_of_line = 0;

}

GLFrame::~GLFrame()
{
    // killTimer(1);
}

void GLFrame::addCirlceToPainting (Circle circle, GLfloat _red, GLfloat _green, GLfloat _blue, bool isFilled)
{
	if (num_of_circle >= 2500)
		num_of_circle = 0;
    circles[ num_of_circle++ ].set_circle( circle,  _red,  _green,  _blue, isFilled );
}

void GLFrame::addLineToPainting (VecPosition start, VecPosition end, GLfloat _red, GLfloat _green, GLfloat _blue)
{
	if (num_of_line > 1000)
		num_of_line = 0;
    lines[ num_of_line++ ].set_Line( start, end,  _red,  _green,  _blue);
}

void GLFrame::addTextToPainting (VecPosition pos, string text, GLfloat _red, GLfloat _green, GLfloat _blue)
{
    texts[ num_of_texts++ ].setText( pos, text,  _red,  _green,  _blue);
}

void GLFrame::resetPainting()
{
    num_of_circle = 0;
    num_of_line = 0;
    num_of_texts = 0;
}
void GLFrame::resetCirclePaintings()
{
	num_of_circle = 0;
}
void GLFrame::resetLinePaintings()
{
	num_of_line = 0;
}
void GLFrame::paintPaitingStuff(World &world)
{
    if( num_of_circle > 2000 || num_of_line > 1000 || num_of_texts > 1000 )
    {
       cout<< "There are too many painting to paint!!!";
        return;
    }
    for( int i = 0; i < num_of_circle; i++ )
        circles[i].Draw(world);

    for( int i = 0; i < num_of_line; i++ )
        lines[i].Draw(world);

    for( int i = 0; i < num_of_texts; i++ )
		displayText( texts[i].position.getX() / 1000.0f, texts[i].position.getY() / 1000.0f, -8,texts[i].Draw(world));
	//IMPORTANT:change made
	//{
    //resetPainting();
	//}

    glColor3b(1,1,1);
    // renderText( -3.475 , 2.475 , -7.99999, curser_pos.toString() );
}






//void GLFrame::set_world(World &_world)
//{
//    world = _world;
//}

void GLFrame::initializeGL()
{
    glClearColor(.2,.3,.1,.5);
    glEnable( GL_DEPTH_TEST );
}
void GLFrame::paintGL(World &world)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glPushMatrix();
	
	//********************************************** Set Scale ***********************************
#if DRAW_CHART == 0
	///normal field
	#if DIVISION == 1
		glScalef(0.147, 0.147, 0.14);
		glTranslatef(0.0, 0.0, 8.0);
	#else
		glScalef(0.2, 0.2, 0.2);
		glTranslatef(0.0, 0.0, 8.0);
	#endif
#else
	///field with chart
	glScalef(0.094, 0.169, 0.2);
	glTranslatef(-WholeFieldLengthG / 1.93, 0.0, 8.0);
#endif

	// glLoadIdentity();

	//********************************************** Draw Field ***********************************
    Field.Draw(world);

    //********************************************** Set and Draw Ball pos ***********************************
    Ball.set_Position(world.ball.getCurrentBallPosition());
    Ball.Draw(world);

//	//********************************************** Set and Draw RRT Paths ***********************************
//#if DRAW_RRT_PATHS == 1
//	RobotT[1].set_Robot(VecPosition(0, 0), 0, 0, world.team_color == TC_Yellow, world.r,2*world.num_of_vertices);
//	RobotT[1].Draw(world);
//	for (int i = 0; i < World::num_of_vertices; i++)
//	{
//		//RRT[i].set_Line()
//	}
//#endif

	//********************************************** Set and Draw RRT Balks ***********************************
#if DRAW_BALKS == 1
	
	GLCircle glc;
	for (int i = 0; i < world.numT; i++)
	{
		glc.set_circle(world.robotT_circle_balk[i], 0.0F, 0.0F, 255.0F, false);
		glc.Draw(world);
	}
	for (int i = 0; i < world.numO; i++)
	{
		glc.set_circle(world.robotO_circle_balk[i], 0.0F, 0.0F, 255.0F, false);
		glc.Draw(world);
	}
	
#endif


	//GLLine gll;
	//for (int i = 0; i < World::num_of_active_line_balks; i++)
	//{
	//	//Circle c = World::circle_balk[i];
	//	gll.set_Line(World::line_balk[i].start_point, World::line_balk[i].end_point, 0.0F, 0.0F, 255.0F);
	//	gll.Draw(world);
	//}

	//********************************************** Set and Draw Mouse ***********************************
#if DRAW_MOUSE == 1
	mouse.set_Robot(VecPosition(world.mouseX, world.mouseY), world.mouseAngle, world.mouseID, world.team_color == TC_Yellow);
	mouse.Draw(world);
	if (world.mouseID<10)
		displayText(world.mouseX / 1000.0f - 0.025, world.mouseY / 1000.0f - 0.020, -7.99999, to_string(world.mouseID), 255, 0, 0);
	else
	{
		/*const char s = char(world.mouseID - 10 + toascii('A'));
		string str(&s);
		displayText(world.mouseX / 1000.0f - 0.025, world.mouseY / 1000.0f - 0.020, -7.99999, str, 255, 0, 0);*/
	}
#endif

#if DRAW_PENALTY_AREA_ATTACKER_LIMIT == 1
	GLLine gll;

	gll.set_Line(Field::getUpLeft_LeftPenaltyArea(), Field::getUpRight_LeftPenaltyArea());
	gll.Draw(world);
	gll.set_Line(Field::getUpRight_LeftPenaltyArea(), Field::getDownRight_LeftPenaltyArea());
	gll.Draw(world);
	gll.set_Line(Field::getDownLeft_LeftPenaltyArea(), Field::getDownRight_LeftPenaltyArea());
	gll.Draw(world);

	gll.set_Line(Field::getUpLeft_RightPenaltyArea(), Field::getUpRight_RightPenaltyArea());
	gll.Draw(world);
	gll.set_Line(Field::getUpLeft_RightPenaltyArea(), Field::getDownLeft_RightPenaltyArea());
	gll.Draw(world);
	gll.set_Line(Field::getDownLeft_RightPenaltyArea(), Field::getDownRight_RightPenaltyArea());
	gll.Draw(world);
#endif

    for(int i = 0 ; i < world.numT; i++)
    {

#pragma region "FeedForward Red Robot"
#if USE_FEEDFORWARD == 1
		//*************** set Robot position ********************
		//***************  set Angle robot	 ***********************
		//***************      set ID		 ***********************
		//***************  set Robot color	 ***********************
		RobotT[i].set_Robot(world.robotT[i].uncorrected_position, world.robotT[i].angle, world.robotT[i].id,2);
		//********************  Draw Robot  *********************
		RobotT[i].Draw(world);
		//********************  Draw ID  ************************
		if (world.robotT[i].id<10)
			displayText(RobotT[i].position.getX() / 1000.0f - 0.025, RobotT[i].position.getY() / 1000.0f - 0.020, -7.99999, to_string(world.robotT[i].id), 0, 0, 0);
		else
		{
			const char s = char(world.robotT[i].id - 10 + toascii('A'));
			string str(&s);
			displayText(RobotT[i].position.getX() / 1000.0f - 0.025, RobotT[i].position.getY() / 1000.0f - 0.020, -7.99999, str, 0, 0, 0);
		}
#endif
#pragma endregion


        //***************       set Robot position        ********************
        //***************       set Angle robot       	  ***********************
        //***************           set ID		          ***********************
        //***************       set Robot color	          ***********************
		//***************  set Robot path to destination  ***********************
        RobotT[i].set_Robot(world.robotT[i].position,world.robotT[i].angle,world.robotT[i].id, world.team_color == TC_Yellow,/*CWorld*/world.robotT[i].pathToDestination,/*CWorld*/world.robotT[i].sizeOfPathToDestination);
        //********************  Draw Robot  *********************
        RobotT[i].Draw(world);
        //********************  Draw ID  ************************
		if(world.robotT[i].id<10)
			displayText(RobotT[i].position.getX() / 1000.0f - 0.025, RobotT[i].position.getY() / 1000.0f - 0.020, -7.99999, to_string(world.robotT[i].id),0,0,0);
		else
		{
			const char s = char(world.robotT[i].id-10 + toascii('A'));
			string str(&s);
			displayText(RobotT[i].position.getX() / 1000.0f - 0.025, RobotT[i].position.getY() / 1000.0f - 0.020, -7.99999,str, 0, 0, 0);
		}

	}
	for (int i = 0; i < world.numO; i++)
	{
		//*************** set Robot position ********************
		//*************** set Angle robot ***********************
		//***************       se ID     ***********************
		//*************** set Robot color ***********************
		RobotO[i].set_Robot(world.robotO[i].position, world.robotO[i].angle, world.robotO[i].id, world.team_color == TC_Blue);
		//********************  Draw Robot  *********************
		RobotO[i].Draw(world);
		//********************  Draw ID  ************************
		if (world.robotO[i].id < 10)
			displayText(RobotO[i].position.getX() / 1000.0f - 0.025, RobotO[i].position.getY() / 1000.0f - 0.020, -7.99999, to_string(world.robotO[i].id), 0, 0, 0);
		else
		{
			const char s = char(world.robotO[i].id-10 + toascii('A'));
			string str(&s);
			displayText(RobotO[i].position.getX() / 1000.0f - 0.025, RobotO[i].position.getY() / 1000.0f - 0.020, -7.99999, str, 0, 0, 0);
		}
	}
	
	

    paintPaitingStuff(world);
	glPopMatrix();
	glutSwapBuffers();



}








void GLFrame::resizeGL(int w, int h)
{
    w = fmax(1,w);
    h = fmax(1,h);
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective(GlframeScale,1.0 ,1.0,100.0 );
    glMatrixMode( GL_MODELVIEW );
    glViewport( 0 , 0 - h * 0.18 , w , h * 1.36 );
    CurrentWidth = w;
    CurrentHeight = h;
}
void GLFrame::displayText(float x, float y,float z,  string string, float _red , float _green , float _blue)
{
	int j = string.length();
	
	char *cstr = new char[string.length() + 1];
	strcpy(cstr, string.c_str());
	// do stuff
	
	glColor3d(_red / 255.0f, _green / 255.0f, _blue / 255.0f);
	glRasterPos3f(x, y, z);
	for (int i = 0; i < j; i++) {
		glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_10, cstr[i]);
	}
	delete[] cstr;
}
void GLFrame::mouseCB(World &world,int button, int state, int x, int y)
{
#if DIVISION == 1
	world.mouseX = 6800 * (2 * (float)x / (float)glutGet(GLUT_WINDOW_WIDTH) - 1);	//5000
	world.mouseY = -6800 * (2 * (float)y / (float)glutGet(GLUT_WINDOW_HEIGHT) - 1);
#elif DIVISION == 2
	world.mouseX = 5000 * (2 * (float)x / (float)glutGet(GLUT_WINDOW_WIDTH) - 1);	//5000
	world.mouseY = -5000 * (2 * (float)y / (float)glutGet(GLUT_WINDOW_HEIGHT) - 1);
#endif
	world.mouseAngle = 0;
	world.mouseID = 1;
	//cout << "\nx:"<< world.mouseX;
	//cout << "y:"<< world.mouseY;
	if (button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			world.mouseLeftDown = true;

		}
		else if (state == GLUT_UP)
			world.mouseLeftDown = false;
	}

	else if (button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			world.mouseRightDown = true;

		}
		else if (state == GLUT_UP)
			world.mouseRightDown = false;
	}

	//glutPostRedisplay();
}
void GLFrame::Keyboard(World &world,unsigned char key, int x, int y)
{
	//Sleep(1);
	switch (key)
	{
	case 27:             // ESCAPE key
		exit(0);
		break;

	case 'a':

		break;

	case 'd':

		break;

	case 's':

		break;
	case'w':

		break;
	case 'A':

		break;

	case 'D':

		break;

	case 'S':

		break;
	case'W':

		break;

	case'-':

	case'_':

		break;
	case'+':

		break;
	case'=':

		break;
	case'r':
	{

		break;
	}
	case'R':
	{

		break;
	}

	}
	//glutPostRedisplay();
}

