#ifndef SWITCHES_H
#define SWITCHES_H

#define GAME_MODE_ROBOCUP_2021 1 /// 1 : Run in Simulator 2 : Technical Challenge 3 : Hardware Challenge
#define CAM_COUNT 4
#define DIVISION 2	/// 1 : for division A		2 : for division B      3 : for OmidReal
#define MAX_BALLS 40
#define MAX_ROBOTS_IN_THE_FIELD 2*MAX_ROBOTS_PER_TEAM_IN_THE_FIELD
#define MAX_ROBOTS_PER_TEAM_IN_THE_FIELD 11
#define MERGE_DISTANCE 500
#define ROBOT_RADIUS 90	  ///millimeter  //ERforce 80 GRSIM 85 Real 85
#define	BALL_RADIUS 21
#define DISTANCE_TO_BALL_IN_STOP_MODE 500
#define ATTACKER_DISTANCE_FROM_PENALTY_AREA_LIMIT 200
#define DRAW_CHART 0
#define DRAW_MATLAB_DIAGRAM 0
#define DRAW_BALKS 0
#define DRAW_MOUSE 0
#define DRAW_RRT_PATHS 0
#define DRAW_PENALTY_AREA_ATTACKER_LIMIT 0
#define SEND_COMMANDS_TO_ROBOTS 2	/// 2: FOR COMMAND ER-force ROBOT 1 : FOR COMMAND TO REAL ROBOTS   0 : FOR COMMAND TO GRSIM ROBOTS
#define USE_FEEDFORWARD 0
#define RECTANGULAR_PENALTY_AREA 1	/// 1 (non zero) : FOR RECTANGULAR PENALTY AREA   0 : FOR CIRCULAR PENALTY AREA
#define REACH_DESTINATION_APPROXIMATION 25
#define REACH_ANGLE_APPROXIMATION 0.4

//////////////////mhz
#define DISTANCE_ROBOT_HAVE_BALL 50
#define DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE 20
#define CORRECTION_FACTOR 0.1
#define DISTANCE_TO_BALL_IN_STOPMODE 500
#define MAX_BALL_SPEED 6//meter/second
#define SET_PASS_DISTANCE_BEST_PASS sqrt(pow(FieldLength,2)+ pow(FieldWidth, 2))*(1/3) 
#define SAFE_PASS_DISTANCE sqrt(pow(FieldLength,2)+ pow(FieldWidth, 2))*(1/10)
#define RESOULOTION_OF_FIND_PASS 50
#define PROBLITY_GOAL_FREE_SPACE 1.25 //   2.25/3 present   //of tree
#define PROBLITY_GOAL_DISTANCE sqrt(pow(FieldLength,2)+ pow(FieldWidth, 2))*(3.0/10) //   1.20/2 present   //of two
#define PLAN_SCORE_CPU 20
#define PLAN_SCORE_POINT_X ((FieldLength / 2) - ROBOT_RADIUS) - (i*ROBOT_RADIUS * PLAN_SCORE_CPU)
#define PLAN_SCORE_POINT_Y ((FieldWidth / 2) - ROBOT_RADIUS) - (j*ROBOT_RADIUS * PLAN_SCORE_CPU)
#define	PLAN_SCORE_MAXIMOM_X (((-FieldLength / 2) + PenaltyAreaWidth)+ 0.5*ROBOT_RADIUS)////penalty area
#define PLAN_SCORE_MINIMOM_X ((-FieldLength / 2)- 3.3 * ROBOT_RADIUS)////penalty area
#define PLAN_SCORE_MAXIMOM_Y ((PenaltyAreaLength / 2)+ 3.3 * ROBOT_RADIUS)////penalty area
#define PLAN_SCORE_MINIMOM_Y ((-PenaltyAreaLength / 2)- 3.3 * ROBOT_RADIUS)////penalty area
#define MAX_ROBOT_SPEED 900;



#define PRESENT_OF_ATTACKER 0.80 //PRESENT OF ATTACKER ROBOT T
//////////////////mhz



#define UDP_CLIENT_INTERFACE "eth1"
//----- Defines Refree -------------------------------------------------------------
#define TEAM_NAME "OMID"
#define PORT_NUM_Refree        10003	//10003          
#define GROUP_ADDR_Refree  "224.5.23.1" 
//----- Defines Vision-------------------------------------------------------------- 
#define PORT_NUM_Vision        10006
#define GROUP_ADDR_Vision  "224.5.23.2"
//----- Defines Send Grsim Commands-------------------------------------------------
#define PORT_NUM_SEND_GRSIM_COMMAND		20011
#define GROUP_ADDR_SEND_GRSIM_COMMAND	"127.0.0.1"//"192.168.0.2"
//----- Defines Send ER-force Commands-------------------------------------------------
#define PORT_NUM_SEND_ERforce_COMMAND_YELLOW		10302
#define PORT_NUM_SEND_ERforce_COMMAND_BLUE		10301
//#define GROUP_ADDR_SEND_ERforce_COMMAND	"172.25.0.12"//div a
#define GROUP_ADDR_SEND_ERforce_COMMAND	"172.20.0.6"//div c
//#define GROUP_ADDR_SEND_ERforce_COMMAND	"127.0.0.1"

#endif