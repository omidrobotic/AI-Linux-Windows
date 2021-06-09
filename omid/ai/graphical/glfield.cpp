// In The Name of God

#include "glfield.h"
#include "../world.h"
#include "glframe.h"
#include "../Switches.h"


// variable of Graphic Field
// why ( /1000 ) ?  ---->  Converting miliMeter to Meter


//--------------------------------------------------------------------------------------------------------------------------
GLField::GLField()
{
}

void GLField::Draw(World &world)
{
	//
	////************************************* chart ***********************
#if DRAW_CHART == 1
	


	////************************************* white background ***********************
	glColor3f(1, 1, 1);
	glBegin(GL_POLYGON);

	glVertex3f((WholeFieldLengthG / 2), (WholeFieldWidthG / 1.25), -8.00003);
	glVertex3f(3 * (WholeFieldLengthG / 1.95), (WholeFieldWidthG / 1.25), -8.00003);
	glVertex3f(3 * (WholeFieldLengthG / 1.95), -(WholeFieldWidthG / 1.25), -8.00003);
	glVertex3f((WholeFieldLengthG / 2), -(WholeFieldWidthG / 1.25), -8.00003);
	glEnd();

	/*glVertex3f((WholeFieldLengthG / 2), (WholeFieldWidthG / 2), -8.00003);
	glVertex3f(0, (WholeFieldWidthG / 2), -8.00003);
	glVertex3f(0, -(WholeFieldWidthG / 2), -8.00003);
	glVertex3f((WholeFieldLengthG / 2), -(WholeFieldWidthG / 2), -8.00003);
	glEnd();*/
	////************************************* axes ***********************
	//(3*(WholeFieldWidthG)/3.75)/2
	glColor3f(0, 0, 0);
	glBegin(GL_LINES);

	glVertex3f((WholeFieldLengthG / 2), 0, -8.00003);
	glVertex3f(3 * (WholeFieldLengthG / 1.95), 0, -8.00003);

	/*glColor3f(1, 0, 0);

	glVertex3f((WholeFieldLengthG / 2), ((3 * WholeFieldWidthG) / 7.5), -8.00003);
	glVertex3f(3 * (WholeFieldLengthG / 1.95), ((3 * WholeFieldWidthG) / 7.5), -8.00003);

	glVertex3f((WholeFieldLengthG / 2), -((3 * WholeFieldWidthG) / 7.5), -8.00003);
	glVertex3f(3 * (WholeFieldLengthG / 1.95), -((3 * WholeFieldWidthG) / 7.5), -8.00003);*/



	////************************************* grids ***********************
	glColor3f(0.7, 0.7, 0.7);

	//horizontal lines
	for (double i = 0; i < 3 * (WholeFieldLengthG / 1.95); i += 0.265)
	{
		glVertex3f((WholeFieldLengthG / 2) + i, 3 * (WholeFieldWidthG) / 3.75, -8.00003);
		glVertex3f((WholeFieldLengthG / 2) + i, -3 * (WholeFieldWidthG) / 3.75, -8.00003);
	}

	//vertical lines
	for (double i = 0; i <= 2 * (WholeFieldWidthG + 0.265); i += 0.265)
	{
		glVertex3f((WholeFieldLengthG / 2), (WholeFieldWidthG)-i, -8.00003);
		glVertex3f(3 * (WholeFieldLengthG / 1.95), (WholeFieldWidthG)-i, -8.00003);

	}


	////************************************* AI data ***********************
	glColor3f(0, 0, 1);
	double scale = 30.0;
	for (int i = 0; i < World::number_of_speeds - 1; i++)
	{
		glVertex3f((i / scale) + (WholeFieldLengthG / 2), sqrt(pow(World::AI_speeds[i][0], 2) + pow(World::AI_speeds[i][1], 2)) /*+ ((3 * WholeFieldWidthG) / 7.5)*/, -8.00003);
		glVertex3f((i / scale) + (1.0 / scale) + (WholeFieldLengthG / 2), sqrt(pow(World::AI_speeds[i + 1][0], 2) + pow(World::AI_speeds[i + 1][1], 2)) /*+ ((3 * WholeFieldWidthG) / 7.5)*/, -8.00003);
	}

	////************************************* ROBOT data ***********************
	glColor3f(1, 0, 0);
	/*for (int i = 0; i < World::number_of_speeds - 1; i++)
	{
		glVertex3f((i / scale) + (WholeFieldLengthG / 2)				   , (3*sqrt(pow(World::previous_robot_speed.getX(),2)+ pow(World::previous_robot_speed.getY() ,2))/700.000) - (WholeFieldWidthG / 1.25) , -8.00003);
		glVertex3f((i / scale) + (1.0 / scale) + (WholeFieldLengthG / 2), (3*sqrt(pow(World::robot_speed.getX(), 2)		  + pow(World::robot_speed.getY(), 2))         /700.000) - (WholeFieldWidthG / 1.25)	, -8.00003);
	}*/
	//cout << sqrt(pow(World::previous_robot_speed.getX(), 2) + pow(World::previous_robot_speed.getY(), 2))<<endl;

	for (int i = 0; i < World::number_of_speeds - 1; i++)
	{
		glVertex3f((i / scale) + (WholeFieldLengthG / 2), sqrt(pow(World::ROBOT_speeds[i][0], 2) + pow(World::ROBOT_speeds[i][1], 2)) - (WholeFieldWidthG / 1.25), -8.00003);
		glVertex3f((i / scale) + (1.0 / scale) + (WholeFieldLengthG / 2), sqrt(pow(World::ROBOT_speeds[i + 1][0], 2) + pow(World::ROBOT_speeds[i + 1][1], 2)) - (WholeFieldWidthG / 1.25), -8.00003);
	}
	glEnd();
#endif
	////************************************* end chart ***********************

    //************************************* suround rectangle ***********************
    // glVertex3f( WholeFieldLengthG, WholeFieldWidthG , ***Z)
    // *** why ( Z ) ? ---->

    glColor3f(0,0.7,0);
    glBegin(GL_POLYGON);

	//glVertex3f(0, 8, -8.00003);
	//glVertex3f(3, 8, -8.00003);

    glVertex3f( -(WholeFieldLengthG/2) , (WholeFieldWidthG/2) , -8.00003);
    glVertex3f( (WholeFieldLengthG/2), (WholeFieldWidthG/2) , -8.00003);
    glVertex3f( (WholeFieldLengthG/2), -(WholeFieldWidthG/2) , -8.00003);
    glVertex3f( - (WholeFieldLengthG/2), -(WholeFieldWidthG/2) , -8.00003);
    glEnd();
    //************************************* main rectangle ***********************
    // glVertex3f( WholeFieldLengthG, WholeFieldWidthG , ***Z)

    glColor3f(0,0.9,0);
    glBegin(GL_POLYGON);
    glVertex3f( -((FieldLengthG/2)+SurroundFieldMarginG) , ((FieldWidthG/2)+SurroundFieldMarginG) , -8.00002);
    glVertex3f( ((FieldLengthG/2)+SurroundFieldMarginG) , ((FieldWidthG/2)+SurroundFieldMarginG) , -8.00002);
    glVertex3f( ((FieldLengthG/2)+SurroundFieldMarginG) , -((FieldWidthG/2)+SurroundFieldMarginG) , -8.00002);
    glVertex3f( -((FieldLengthG/2)+SurroundFieldMarginG) , -((FieldWidthG/2)+SurroundFieldMarginG) , -8.00002 );
    glEnd();
    //************************************* main field lines ***********************
    glColor3f(1,1,1);
    glBegin(GL_LINES);
    glVertex3f( -(FieldLengthG/2) , (FieldWidthG/2) , -8.00001) ;
    glVertex3f( (FieldLengthG/2) , (FieldWidthG/2) , -8.00001) ;
    glVertex3f( (FieldLengthG/2) , (FieldWidthG/2) , -8.00001) ;
    glVertex3f( (FieldLengthG/2) , -(FieldWidthG/2) , -8.00001) ;
    glVertex3f( (FieldLengthG/2) , -(FieldWidthG/2) , -8.00001) ;
    glVertex3f( -(FieldLengthG/2) , -(FieldWidthG/2) , -8.00001) ;
    glVertex3f( -(FieldLengthG/2) , -(FieldWidthG/2) , -8.00001) ;
    glVertex3f( -(FieldLengthG/2) , (FieldWidthG/2) , -8.00001) ;

    //************************************ Mid Line **************************
    glVertex3f( 0 , (FieldWidthG/2) , -8.00001) ;
    glVertex3f( 0 , -(FieldWidthG/2) , -8.00001) ;
    //************************************ Line of Penalty Area  ***************
    /*Circle Penalty Area
    glVertex3f( -((FieldLengthG/2)-PenaltyAreaRadiusG)  , (FreeBoundG/2) , -8.00001) ;
    glVertex3f( -((FieldLengthG/2)-PenaltyAreaRadiusG)  , -(FreeBoundG/2) , -8.00001) ;
    glVertex3f( ((FieldLengthG/2)-PenaltyAreaRadiusG) , (FreeBoundG/2) , -8.00001) ;
    glVertex3f( ((FieldLengthG/2)-PenaltyAreaRadiusG) , -(FreeBoundG/2) , -8.00001) ;*/
	
    //************************************ Corner points **************************
    glVertex3f( ((FieldLengthG/2)-0.1)  , (FieldWidthG/2) , -8.00001) ;
    glVertex3f(  ((FieldLengthG/2)-0.1) , ((FieldWidthG/2)-.025) , -8.00001) ;
    glVertex3f( -((FieldLengthG/2)-0.1) , (FieldWidthG/2) , -8.00001) ;
    glVertex3f( -((FieldLengthG/2)-0.1) , ((FieldWidthG/2)-.025) , -8.00001) ;
    glVertex3f( ((FieldLengthG/2)-0.1)  , -(FieldWidthG/2) , -8.00001) ;
    glVertex3f( ((FieldLengthG/2)-0.1)  , -((FieldWidthG/2)-.025) , -8.00001) ;
    glVertex3f( -((FieldLengthG/2)-0.1) , -(FieldWidthG/2) , -8.00001) ;
    glVertex3f( -((FieldLengthG/2)-0.1) , -((FieldWidthG/2)-.025) , -8.00001) ;
    glEnd();
    //************************************ Mid point **************************
    glBegin(GL_TRIANGLE_FAN);

    for(double angle = 0.0f; angle <= 2.0 * M_PI; angle += (M_PI/20.0f))
    {
        double x = 0.03 * sin(angle);
        double y = 0.03 * cos(angle);
        glVertex3f(x , y, -8.00001);
    }
    glEnd();

	//************************************ Penalty Point **************************
  /*  glBegin(GL_TRIANGLE_FAN);

    for(double angle = 0.0f; angle <= 2.0 * M_PI; angle += (M_PI/20.0f))
    {
        double x = 0.03 * sin(angle);
        double y = 0.03 * cos(angle);
        glVertex3f(x + ((FieldLengthG/2)-PenaltyPointG), y, -8.00001);
    }
    glEnd();

    glBegin(GL_TRIANGLE_FAN);

    for(double angle = 0.0f; angle <= 2.0 * M_PI; angle += (M_PI/20.0f))
    {
        double x = 0.03 * sin(angle);
        double y = 0.03 * cos(angle);
        glVertex3f(x -((FieldLengthG/2)-PenaltyPointG), y, -8.00001);
    }
    glEnd();*/

	//************************************ Main Circle **************************
    double angle;
	double CenterAreaRadius = CenterCircleRadiusG;// , PenaltyAreaRadius = PenaltyAreaRadiusG;
    double x,y;

    glBegin(GL_LINE_LOOP);
    for(angle = 0.0f; angle <= 2.0 * M_PI; angle += (M_PI/20.0f))
    {
        x = CenterAreaRadius * sin(angle);
        y = CenterAreaRadius * cos(angle);
        glVertex3f(x, y, -8.00001);
    }
    glEnd();

	//uncomment the    // , PenaltyAreaRadius = PenaltyAreaRadiusG;   in the above , then uncomment below code
   /* glBegin(GL_POINTS);
//    r = 0.8;
    for(angle = 0.0f ; angle <= M_PI/2.0 ; angle += (M_PI/20.0f)/20)
    {
        x = PenaltyAreaRadius * sin(angle) - (FieldLengthG/2);
        y = PenaltyAreaRadius * cos(angle) + (FreeBoundG/2);
        glVertex3f(x, y, -8.00001);
    }
    glEnd();
    glBegin(GL_POINTS);
    for(angle = M_PI/2 ; angle <= M_PI ; angle += (M_PI/20.0f)/20)
    {
        x = PenaltyAreaRadius * sin(angle) - (FieldLengthG/2);
        y = PenaltyAreaRadius * cos(angle) - (FreeBoundG/2);
        glVertex3f(x, y, -8.00001);
    }
    glEnd();
    glBegin(GL_POINTS);
    for(angle = (3*M_PI)/2 ; angle <= 2.0 * M_PI; angle += (M_PI/20.0f)/20)
    {
        x = PenaltyAreaRadius * sin(angle) + (FieldLengthG/2);
        y = PenaltyAreaRadius * cos(angle) + (FreeBoundG/2);
        glVertex3f(x, y, -8.00001);
    }
    glEnd();
    glBegin(GL_POINTS);
    for(angle = M_PI ; angle <=  (3*M_PI)/2 ; angle += (M_PI/20.0f)/20)
    {
        x = PenaltyAreaRadius * sin(angle) + (FieldLengthG/2);
        y = PenaltyAreaRadius * cos(angle) - (FreeBoundG/2);
        glVertex3f(x, y, -8.00001);
    }
    glEnd();*/

	//************************************* Penalty Area ***********************

	glColor3f(1, 1, 1);
	glBegin(GL_LINES);
	glVertex3f(-(FieldLengthG / 2),(PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f(-(FieldLengthG / 2) + PenaltyAreaWidthG , (PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f(-(FieldLengthG / 2), -(PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f(-(FieldLengthG / 2) + PenaltyAreaWidthG, -(PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f(-(FieldLengthG / 2) + PenaltyAreaWidthG, (PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f(-(FieldLengthG / 2) + PenaltyAreaWidthG, -(PenaltyAreaLengthG / 2), -8.00001);
	
	glVertex3f((FieldLengthG / 2), (PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f((FieldLengthG / 2) - PenaltyAreaWidthG, (PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f((FieldLengthG / 2), -(PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f((FieldLengthG / 2) - PenaltyAreaWidthG, -(PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f((FieldLengthG / 2) - PenaltyAreaWidthG, (PenaltyAreaLengthG / 2), -8.00001);
	glVertex3f((FieldLengthG / 2) - PenaltyAreaWidthG, -(PenaltyAreaLengthG / 2), -8.00001);

	glEnd();

    //************************************* Goal ***********************
	if(world.team_color==TC_Blue)
		glColor3f(0, 0, 1);
	else
		glColor3f(1, 1, 0);
    glBegin(GL_LINES);
    glVertex3f( -((FieldLengthG/2)+0.180), (GoalLengthG/2) , -8.00001) ;
    glVertex3f( -(FieldLengthG/2) , (GoalLengthG/2) , -8.00001) ;
    glVertex3f( -((FieldLengthG/2)+0.180), -(GoalLengthG/2) , -8.00001) ;
    glVertex3f( -(FieldLengthG/2), -(GoalLengthG/2) , -8.00001) ;
    glVertex3f( -((FieldLengthG/2)+0.180), (GoalLengthG/2) , -8.00001) ;
    glVertex3f( -((FieldLengthG/2)+0.180), -(GoalLengthG/2) , -8.00001) ;
	if (world.team_color == TC_Blue)
		glColor3f(1, 1, 0);
	else
		glColor3f(0, 0, 1);
    glVertex3f( ((FieldLengthG/2)+0.180), (GoalLengthG/2) , -8.00001) ;
    glVertex3f( (FieldLengthG/2) , (GoalLengthG/2) , -8.00001) ;
    glVertex3f( ((FieldLengthG/2)+0.180), -(GoalLengthG/2) , -8.00001) ;
    glVertex3f( (FieldLengthG/2), -(GoalLengthG/2) , -8.00001) ;
    glVertex3f( ((FieldLengthG/2)+0.180), (GoalLengthG/2) , -8.00001) ;
    glVertex3f( ((FieldLengthG/2)+0.180), -(GoalLengthG/2) , -8.00001) ;
    glEnd();
}
//--------------------------------------------------------------------------------------------------------------------------
