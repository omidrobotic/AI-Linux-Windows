#ifndef VISION_H
#define VISION_H
#include "Switches.h"
#include <iostream>
#if (SEND_COMMANDS_TO_ROBOTS==2)
#include "Protobuf/ER-force/messages_robocup_ssl_detection.pb.h"
#include "Protobuf/ER-force/messages_robocup_ssl_geometry.pb.h"
#include "Protobuf/ER-force/messages_robocup_ssl_wrapper.pb.h"
#else
#include "Protobuf/Vision/messages_robocup_ssl_detection.pb.h"
#include "Protobuf/Vision/messages_robocup_ssl_geometry.pb.h"
#include "Protobuf/Vision/messages_robocup_ssl_wrapper.pb.h"
#endif

#include "Socket_udp.h"
#include "filter.h"
#include "world.h"
#include "Switches.h"

class Vision
{
	public:

		 Vision(World &_world)
		 {
			 balls_num = 0;
			 balls_num_TrackingMode = 0;
			 ssl_lastball_n = 0;
			 Current_ball_index = 0;
			 Current_ball_lost = true;
			 for (int i = 0;i < 15;i++)
			 {
				 num_of_last_seen_team[i] = 0;
				 time_of_last_seen_team[i] = 0;
				 num_of_last_seen_opp[i] = 0;
				 time_of_last_seen_opp[i] = 0;
			 }
		 };
		~Vision();
		Socket_udp vision_udp;
		int TrackingMode_minimum_distance;

		int blue_robots_num ;
		int yellow_robots_num ;
		int All_robots_num ;
		int balls_num ;
		int balls_num_TrackingMode;

		void recive_Init(void);

		bool recievePacket(void);
		

		void ProcessVision(World &world);


		void ProcessRobots(World &world);
		int ExtractBlueRobots ( void );
		int ExtractYellowRobots ( void );
		int MergeRobots ( int num, SSL_DetectionRobot robot[MAX_ROBOTS_IN_THE_FIELD*CAM_COUNT]);
		void Robots_Parser(World &world,int num_blue, int num_yellow);
		int Ball_Parser(World &world, int num);
		void ProcessBalls (World &world);
		int ExtractBalls ( void );
		int MergeBalls ( int num );
		void Nearer_Ball(VecPosition pos);
	
		///////////////Ball///////////////

		VecPosition ssl_lastball_detection[MAX_BALLS*CAM_COUNT +1];
		int ssl_lastball_n;

		int ssl_ball_LastUpdate[24];
		int ssl_ball_LastcamID[24];

		int ssl_robotT_LastUpdate[MAX_ROBOTS_IN_THE_FIELD];	///would be 0 if that index's robot is seen recently,grow up if that index's robot is not seen recently
		int ssl_robotO_LastUpdate[MAX_ROBOTS_IN_THE_FIELD];

		int Current_ball_index;
		bool Current_ball_lost;
//		RobotState robotState[2][MAX_ROBOTS_IN_THE_FIELD];



		SSL_WrapperPacket packet;
		SSL_DetectionFrame frame[CAM_COUNT];
		SSL_DetectionBall ball[MAX_BALLS*CAM_COUNT];
		SSL_DetectionRobot robot_blue[MAX_ROBOTS_IN_THE_FIELD*CAM_COUNT];
		SSL_DetectionRobot robot_yellow[MAX_ROBOTS_IN_THE_FIELD*CAM_COUNT];

		double time_capture_ball[MAX_BALLS*CAM_COUNT];

		double time_capture_blue[MAX_ROBOTS_IN_THE_FIELD*CAM_COUNT];
		double time_sent_blue[MAX_ROBOTS_IN_THE_FIELD*CAM_COUNT];

		double time_capture_yellow[MAX_ROBOTS_IN_THE_FIELD*CAM_COUNT];
		double time_sent_yellow[MAX_ROBOTS_IN_THE_FIELD*CAM_COUNT];

		double time_recive;
		
		int num_of_last_seen_team[15];		///number of times that a robot seen before it added to world.robotT. if it is bigger than some value,that robot would be added to world.robotT
		double time_of_last_seen_team[15];
		int num_of_last_seen_opp[15];	
		double time_of_last_seen_opp[15];

		bool packet_recieved[CAM_COUNT];
		bool use_camera[CAM_COUNT];

		TeamSide side_field;
		bool TrackingMode;
		
		RobotKalmanFilter RTKF[MAX_BALLS*CAM_COUNT];
		RobotKalmanFilter ROKF[MAX_BALLS*CAM_COUNT];
		BallKalmanFilter BKF[MAX_BALLS*CAM_COUNT];

#pragma region "feedforward"
		/*VecPosition summation_of_movement_seen_by_vision_0to15[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
		VecPosition summation_of_movement_seen_by_vision_mines15to0[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
		VecPosition summation_of_movement_calculated_by_commands_15to30[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
		VecPosition summation_of_movement_calculated_by_commands_0to15[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];*/
#pragma endregion



};

#endif // VISION_H


