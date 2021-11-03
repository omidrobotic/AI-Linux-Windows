#ifndef WORLD_H
#define WORLD_H

#include "FieldGeometry.h"
#include "geometry.h"
#include "GameState.h"
#include "timer.h"
#include "Switches.h"

//#define CAM_COUNT 4
//#define DIVISION B
//#define DRAW_CHART 0
//#define DRAW_MATLAB_DIAGRAM 0
//#define DRAW_BALKS 1
//#define DRAW_MOUSE 0
//#define DRAW_RRT_PATHS 1
//#define MAX_BALLS 40
//#define MAX_ROBOTS_IN_THE_FIELD 12
//#define MAX_ROBOTS_PER_TEAM_IN_THE_FIELD 8
//#define MERGE_DISTANCE 500
//#define ROBOT_RADIUS 85	//millimeter
//#define SEND_COMMANDS_TO_ROBOTS 1	/// 1 : FOR COMMAND TO REAL ROBOTS   0 : FOR COMMAND TO GRSIM ROBOTS
//#define USE_FEEDFORWARD 0

////----- Defines Refree -------------------------------------------------------------
//#define PORT_NUM_Refree        10004	//10003          
//#define GROUP_ADDR_Refree  "224.5.23.1" 
////----- Defines Vision------------------------------------------------------------- 
//#define PORT_NUM_Vision        10008
//#define GROUP_ADDR_Vision  "224.5.23.2"
////------------------------------------------------------------------------------

#define MOUSE_AS_VECPOSITION VecPosition(world.mouseX,world.mouseY)

//class Setpoint
//{
//public:
//	Setpoint()
//	{
//	}
//
//	unsigned int id;
//	VecPosition position;
//	AngRad angle;
//
//};


/*! ENUMS */
enum TeamColorData
{
	TC_Yellow = 0,
	TC_Blue = 1
};
enum TeamSide
{
	TS_RightSide = 0,
	TS_LeftSide = 1
};
/*! ENUMS */

class Ball
{
public:
	Ball()
	{
	}

	enum BalkMode {
		notBalk = 0,
		balk = 1,
		stopMode = 2
	};

	VecPosition velocity;
	VecPosition oldPositions[MAX_BALLS + 1];
	double oldTime[MAX_BALLS + 1];
	double radius;

	bool setCurrentBallPosition(VecPosition _pos, double time_captured);
	bool setVelocity(VecPosition _vel);
	VecPosition getCurrentBallPosition();
	VecPosition getVelocity();
	VecPosition getOldBallPosition(int _oldFrame);
	double getOldBallTime(int _oldFrame);
};
class Robot
{
public:
	unsigned int id;
	VecPosition position;
	VecPosition uncorrected_position;
	Circle* robot_balk;
	VecPosition velocity;
	double w;
	AngRad angle;
	double wheelAngleForward;
	double wheelAngleBack;
	double radius;
	long double timeCaptured;
	double timeSent;
	double timeRecived;

	void operator = (const Robot &r);

	///Strategy
	//StrategyDuty duty = d_NoDuty;
	VecPosition destination_position = NULL;
	AngRad destination_angle = NULL;
	bool destination_set = false;

	///RRT
	VecPosition pathToDestination[59 /*World::num_of_vertices*/];
	int sizeOfPathToDestination;	///number of all nodes of the path,including current position and original destinition
	bool isRobotTBalk = true;
	bool isRobotOBalk = true;
	bool isPenaltyAreaBalk = true;
	Ball::BalkMode ballBalkMode = Ball::BalkMode::notBalk;
	
	///Velocitis
	bool send_command = true;
	VecPosition velocityToGo{ NULL,NULL };
	double wToGo;

	///shoot and chip
	short int kick_power; ///between 0 to 7
	bool shoot_or_chip;	///true(1) for shoot; false(0) for chip
	bool spinBack;

    // Farhan
    string role; // "None", "Golie", "Attack", "Recieve", "Defend"
    int coveredBy = -1;
};
class TeamData {
public:
    VecPosition Set_Refree_Ball_Position;
    int sendDataPort;
    int Goalie;
	int Yellow_Cards;
	uint32_t Yellow_Card_Times[12];
	int Yellow_Cards_Times_Size;
	int Red_Cards;
	int Timeouts;
	uint32_t Timeout_Time;
	int Score;
	TeamColorData color;
};
class Field
{
public:
	static   VecPosition getGoalMidO();
	static   VecPosition getGoalMidP();
	static   VecPosition getUpLeftPoint();
	static   VecPosition getUpRightPoint();
	static   VecPosition getDownLeftPoint();
	static   VecPosition getDownRightPoint();
	static   VecPosition getSurroundFieldUpLeftPoint();
	static   VecPosition getSurroundFieldUpRightPoint();
	static   VecPosition getSurroundFieldDownLeftPoint();
	static   VecPosition getSurroundFieldDownRightPoint();
	static   VecPosition getUpMidPoint();
	static   VecPosition getDownMidPoint();
	static   VecPosition getMidPoint();
	static   VecPosition getUpBarP();
	static   VecPosition getDownBarP();
	static   VecPosition getUpBarO();
	static   VecPosition getDownBarO();
	static   VecPosition getUpLeftCornel();
	static   VecPosition getDownLeftCornel();
	static   VecPosition getUpRightCornel();
	static   VecPosition getDownRightCornel();
	static   VecPosition getUpLeft_LeftPenaltyArea();
	static   VecPosition getDownLeft_LeftPenaltyArea();
	static   VecPosition getUpRight_LeftPenaltyArea();
	static   VecPosition getDownRight_LeftPenaltyArea();
	static   VecPosition getUpLeft_RightPenaltyArea();
	static   VecPosition getDownLeft_RightPenaltyArea();
	static   VecPosition getUpRight_RightPenaltyArea();
	static   VecPosition getDownRight_RightPenaltyArea();
	static   VecPosition getPenaltyPointP();
	static   VecPosition getPenaltyPointO();
	static	 VecPosition getRightGoal_DownGoalPost_RightPoint();
	static	 VecPosition getRightGoal_DownGoalPost_LeftPoint();
	static	 VecPosition getRightGoal_UpGoalPost_RightPoint();
	static	 VecPosition getRightGoal_UpGoalPost_LeftPoint();
	static	 VecPosition getLeftGoal_DownGoalPost_RightPoint();
	static	 VecPosition getLeftGoal_DownGoalPost_LeftPoint();
	static	 VecPosition getLeftGoal_UpGoalPost_RightPoint();
	static	 VecPosition getLeftGoal_UpGoalPost_LeftPoint();
	static   Line getGoalLineP();
	static   Line getGoalLineO();
	static	 Paraline getUpParaline_LeftPenaltyArea();
	static	 Paraline getRightParaline_LeftPenaltyArea();
	static	 Paraline getDownParaline_LeftPenaltyArea();
	static	 Paraline getLeftParaline_LeftPenaltyArea();
	static	 Paraline getUpParaline_RightPenaltyArea();
	static	 Paraline getRightParaline_RightPenaltyArea();
	static	 Paraline getDownParaline_RightPenaltyArea();
	static	 Paraline getLeftParaline_RightPenaltyArea();
	static	 Paraline getRightGoal_UpGoalPost_CountinuedToEndOfField();
	static	 Paraline getRightGoal_DownGoalPost_CountinuedToEndOfField();
	static	 Paraline getLeftGoal_DownGoalPost_CountinuedToEndOfField();
	static	 Paraline getLeftGoal_UpGoalPost_CountinuedToEndOfField();
	static   Rect getOppAreaonetouchRectup();
	static   Rect getOppAreaonetouchRectdown();
	static   Rect getFieldRectangle();
	static   Rect getSurroundFieldRectangle();
	static   Rect getMidAreaRect();
	static   Rect getTeamAreaRect();	///
	static   Rect getOppAreaRect();
#if RECTANGULAR_PENALTY_AREA != 0
	static	 Rect getLeftPenaltyArea();
	static   Rect getRightPenaltyArea();
#endif
	static   bool isInField(const VecPosition &v);
	static   bool isInSurroundField(const VecPosition &v);
	static	 bool isInSurroundField_notInBehindGoal(const VecPosition &v);
	static	 bool isInBehindGoal(const VecPosition &v);
	static	 short int detect_airt(VecPosition position);
};
class World
{
public:
	Ball ball;
	Robot robotT[MAX_ROBOTS_IN_THE_FIELD];
	Robot robotO[MAX_ROBOTS_IN_THE_FIELD];
	//Setpoint Setpoint_desired[MAX_ROBOTS_IN_THE_FIELD];

	int numT;
	int last_numT;
	int numO;
	int last_numO;

#pragma region "RRT"
	/*!RRT VARIABLES*/
	static const int num_of_vertices = 60;		//60
	static const int num_of_additional_circle_balks = 1;
	static const int num_of_additional_paraline_balks = 6;
	Circle robotT_circle_balk[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	Circle robotO_circle_balk[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	Circle additional_circle_balks[num_of_additional_circle_balks];
	const Paraline additional_paraline_balks[num_of_additional_paraline_balks];
	//VecPosition r[2 * num_of_vertices];
	//static const int num_of_allowed_circle_balks = 24;
	//static int num_of_active_line_balks;
	//static Circle circle_balk[num_of_allowed_circle_balks];
	//static const int num_of_allowed_line_balks = 10;
	//static const int num_of_allowed_robotT_circle_balks = 12;
	//static const int num_of_allowed_robotO_circle_balks = 12;
	//int num_of_robotT_circle_bulks;
	//int num_of_robotO_circle_bulks;
#if RECTANGULAR_PENALTY_AREA == 0
	static const Circle penalty_area_circle_balk[4];
	static Paraline penalty_area_line_balk[2];
#endif
	/*! RRT VARIABLES*/

#pragma endregion 

#pragma region "Chart VARIABLES"
#if DRAW_CHART == 1
	/*! Chart VARIABLES*/
	static const int number_of_speeds = 1000;	//164
	static double AI_speeds[number_of_speeds][2];
	static double ROBOT_speeds[number_of_speeds][2];
	/*static double ROBOT_avarage_location_x;
	static double ROBOT_avarage_location_y;*/
	static VecPosition previous_robot_speed;
	static VecPosition previous_robot_location;
	static double previous_vision_time;
	static VecPosition robot_speed;
	/*! Chart VARIABLES*/
#endif
#pragma endregion

#pragma region "feed_forward"
#if USE_FEEDFORWARD == 1
	static const int feedforward_number_of_speeds_to_consider = 25;	//50
	VecPosition robot_movement_seen_by_vision[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD][feedforward_number_of_speeds_to_consider];
	VecPosition robot_movement_calculated_by_commands[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD][feedforward_number_of_speeds_to_consider];

	void shift_robot_positions_seen_by_vision();
	void shift_robot_positions_calculated_by_commands();

	static VecPosition summation_of_movement_seen_by_vision_0to15[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	static VecPosition summation_of_movement_seen_by_vision_mines15to0[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	static VecPosition summation_of_movement_calculated_by_commands_15to30[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	static VecPosition summation_of_movement_calculated_by_commands_0to15[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
#endif
#pragma endregion

    static TeamColorData  team_color;  // blue yellow
	static TeamSide   team_side;  // right  left
	static mode_State::StageMode stageMode;
	static mode_State::PlayMode playMode;
	static mode_State::KickMode kickMode;
	static VecPosition ballPlacementPosition;

	void setStageMode(mode_State::StageMode _pm);
	void setPlayMode(mode_State::PlayMode _pm);
	void setKickMode(mode_State::KickMode _pm);
	void setTeamColor(TeamColorData _cl);
	void setTeamSide(TeamSide _s);

	VecPosition get_robotT_position(const int &robot_number) const;
	int getIndexForRobotTNumber(const int &robot_number) const;
	int getRobotTNumberForIndex(const int &robot_index) const;
	int getIndexForRobotONumber(const int &robot_number) const;
	int getRobotONumberForIndex(const int &robot_index) const;
	void exactSleep(double milliseconds);

	double mouseX;
	double mouseY;
	bool mouseLeftDown;
	bool mouseRightDown;
	double mouseAngle = 0;
	unsigned int mouseID = 23;
	static World instance;
	static World &getInstance();
	//double time_remaining;
	static Timer glTimer;
	TeamData team_T;
	TeamData team_O;
	World();



};

class DrawShape
{
public:
	static void DrawDot(VecPosition point, double radius = 50, double red = 255, double green = 0, double blue = 0);
	static void DrawCircle(Circle circle, double red = 255, double green = 0, double blue = 0,bool isfill = false);
	static void DrawParaline(VecPosition start, VecPosition end, double red = 255, double green = 0, double blue = 0);
	static void DrawParaline(Paraline pl, double red = 255, double green = 0, double blue = 0);
	static void ClearCircles();
	static void ClearLines();
};
class Tools
{
public:
	static int u(double t);
	static int inPeriodGenerator();
};

extern World world;
//extern World CWorld;

#endif // WORLD_H
