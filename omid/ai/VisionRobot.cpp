#include "Vision.h"
#include <fstream>
#include "geometry.h"
#include "Switches.h"

unsigned int fr_num = 0;

void Vision::ProcessRobots(World &world)
{
	/// blue/yellow_robots_num is summation of all robot been seen by all cameras, even if they are the same robots 
	/// ExtractYellow/BlueRobots is set all robots seen by all cameras(even if htey are same)in an array named : robot_blue/yellow
	/// MergeRobots is merging robots that has same id and has distance less than MERGE_DISTANCE into robot_blue/yellow

	blue_robots_num = 0;
	yellow_robots_num = 0;
	All_robots_num = 0;
	time_recive = world.glTimer.getTime();
	//Blue Robots
	//First we have to extract the robots!
	blue_robots_num = ExtractBlueRobots();		
	//Now lets merge them!
	blue_robots_num = MergeRobots(blue_robots_num, robot_blue);
	//Yellow Robots
	//First we have to extract the robots!
	yellow_robots_num = ExtractYellowRobots();
	//Now lets merge them!
	yellow_robots_num = MergeRobots(yellow_robots_num, robot_yellow);
	//parser
	Robots_Parser(world, blue_robots_num, yellow_robots_num);
	//We're almost done, only Prediction remains undone!
}
///alireza: get all robots seen by all cameras
int Vision::ExtractBlueRobots(void)
{
	int ans = 0;
	for (int i = 0; i < CAM_COUNT; i++)
	{

		if (use_camera[i])
		{
			for (int j = 0; j < fmin(MAX_ROBOTS_IN_THE_FIELD, frame[i].robots_blue_size()); j++)
			{
				if (((unsigned int)frame[i].robots_blue(j).robot_id())<12)
				{
					robot_blue[ans] = frame[i].robots_blue(j);
					time_capture_blue[ans] = frame[i].t_capture();;
					time_sent_blue[ans] = frame[i].t_sent();

					ans++;
				}
			}
		}
	}
	return ans;
}
///alireza: get all robots seen by all cameras
int Vision::ExtractYellowRobots(void)
{
	int ans = 0;
	for (int i = 0; i < CAM_COUNT; i++)
	{

		if (use_camera[i])
		{

			for (int j = 0; j < fmin(MAX_ROBOTS_IN_THE_FIELD, frame[i].robots_yellow_size()); j++)
			{
				if (((unsigned int)frame[i].robots_yellow(j).robot_id())<12)
				{
					robot_yellow[ans] = frame[i].robots_yellow(j);
					time_capture_yellow[ans] = frame[i].t_capture();;
					time_sent_yellow[ans] = frame[i].t_sent();

					ans++;
				}
			}
		}
	}
	return ans;
}

///alireza:unify the robots that have same id and their distanec are lower than MERGE_DISTANCE
int Vision::MergeRobots(int num, SSL_DetectionRobot robot[MAX_ROBOTS_IN_THE_FIELD*CAM_COUNT])
{
	int robots_num = 0;
	for (int i = 0; i < num; i++)
	{
		for (int j = i + 1; j < num; j++)
		{

			if ((robot[i].robot_id()== robot[j].robot_id()) && sqrtf(powf(robot[i].x() - robot[j].x(), 2) + powf(robot[i].y() - robot[j].y(), 2)) < MERGE_DISTANCE)
			{
				robot[i].set_x((robot[i].x() + robot[j].x()) / (float)2.0);
				robot[i].set_y((robot[i].y() + robot[j].y()) / (float)2.0);

				robot[j] = robot[num - 1];
				num--;

				j--;
			}
		}
		robots_num++;
	}

	return robots_num;
}
void Vision::Robots_Parser(World &world, int num_blue, int num_yellow)
{

#pragma region "feedforward"
#if USE_FEEDFORWARD == 1
	int i;
	int j;
	double guess_number = 500;	//30
	double defualt_k = 1.141; //1.5
	for (i = 0; i < ((world.team_color == TC_Yellow)? num_yellow : num_blue); i++)
	{
		World::summation_of_movement_seen_by_vision_mines15to0[i] = 0;
		World::summation_of_movement_seen_by_vision_0to15[i] = 0;

		World::summation_of_movement_calculated_by_commands_15to30[i] = 0;
		World::summation_of_movement_calculated_by_commands_0to15[i] = 0;

		for (j = 0; j < World::feedforward_number_of_speeds_to_consider/2; j++)
		{
			World::summation_of_movement_seen_by_vision_0to15[i] += world.robot_movement_seen_by_vision[i][j];
		}
		for (j = 14; j < World::feedforward_number_of_speeds_to_consider; j++)
		{
			World::summation_of_movement_calculated_by_commands_15to30[i] += world.robot_movement_calculated_by_commands[i][j];
		}
		for (j = 0; j < World::feedforward_number_of_speeds_to_consider/2; j++)
		{
			World::summation_of_movement_calculated_by_commands_0to15[i] += world.robot_movement_calculated_by_commands[i][j];
		}


		if (abs(World::summation_of_movement_calculated_by_commands_15to30[i].getX()) < guess_number && abs(World::summation_of_movement_calculated_by_commands_15to30[i].getY()) >guess_number)
		{
			World::summation_of_movement_seen_by_vision_mines15to0[i].setX(defualt_k * World::summation_of_movement_calculated_by_commands_0to15[i].getX());
			World::summation_of_movement_seen_by_vision_mines15to0[i].setY(abs(World::summation_of_movement_seen_by_vision_0to15[i].getY() / World::summation_of_movement_calculated_by_commands_15to30[i].getY()) * World::summation_of_movement_calculated_by_commands_0to15[i].getY());
			//cout << "x" << endl;
		}
		else if(abs(World::summation_of_movement_calculated_by_commands_15to30[i].getX()) > guess_number &&abs( World::summation_of_movement_calculated_by_commands_15to30[i].getY()) < guess_number)
		{
			World::summation_of_movement_seen_by_vision_mines15to0[i].setY(defualt_k * World::summation_of_movement_calculated_by_commands_0to15[i].getY());
			World::summation_of_movement_seen_by_vision_mines15to0[i].setX(abs(World::summation_of_movement_seen_by_vision_0to15[i].getX() / World::summation_of_movement_calculated_by_commands_15to30[i].getX()) * World::summation_of_movement_calculated_by_commands_0to15[i].getX());
			//cout << "y" << endl;
		}
		else if (abs(World::summation_of_movement_calculated_by_commands_15to30[i].getX()) < guess_number && abs(World::summation_of_movement_calculated_by_commands_15to30[i].getY()) < guess_number/*false*/)
		{
			World::summation_of_movement_seen_by_vision_mines15to0[i].setX(defualt_k * World::summation_of_movement_calculated_by_commands_0to15[i].getX());
			World::summation_of_movement_seen_by_vision_mines15to0[i].setY(defualt_k * World::summation_of_movement_calculated_by_commands_0to15[i].getY());
			//cout << "0"<<endl;
		}
		else
		{
			World::summation_of_movement_seen_by_vision_mines15to0[i].setX(abs(World::summation_of_movement_seen_by_vision_0to15[i].getX() / World::summation_of_movement_calculated_by_commands_15to30[i].getX()) * World::summation_of_movement_calculated_by_commands_0to15[i].getX());
			World::summation_of_movement_seen_by_vision_mines15to0[i].setY(abs(World::summation_of_movement_seen_by_vision_0to15[i].getY() / World::summation_of_movement_calculated_by_commands_15to30[i].getY()) * World::summation_of_movement_calculated_by_commands_0to15[i].getY());
			//cout << "in"<<endl;
		}
	}
#endif
#pragma endregion

	///alireza:add robots to world.robotT
	if (world.getInstance().team_color == TC_Blue)
	{
		for (int i = 0; i < num_blue; i++)  //robot_blue_n ---> number of blue robot detection .
		{

			int count = 0;
			///alireza: i guess this : [  (world.robotT[count].id != robot_blue[i].robot_id())   ]  means that if a robot is not in world.robotT
			while ((count < world.numT) && (world.robotT[count].id != robot_blue[i].robot_id()))
				count++;

			if (world.glTimer.getTime() - time_of_last_seen_team[robot_blue[i].robot_id()] > 500) 
				time_of_last_seen_team[robot_blue[i].robot_id()] = 0;
			///if robot is not in world.robotT
			if (count == world.numT)
			{
				//cout << "world:" << num_of_last_seen_team[robot_blue[i].robot_id()];
				///if num_of_last_seen_team is bigger than 60,that robot would be added to world.robotT
				if (num_of_last_seen_team[robot_blue[i].robot_id()] > 60)
				{
					num_of_last_seen_team[robot_blue[i].robot_id()] = 0;
					world.numT++;
					world.robotT[count] = Robot();
					world.robotT[count].id = robot_blue[i].robot_id();
				}
				///else num_of_last_seen_team would be incremented
				else
				{
					num_of_last_seen_team[robot_blue[i].robot_id()]++;
					time_of_last_seen_team[robot_blue[i].robot_id()] = world.glTimer.getTime();
				}
			}

			///add robot to world.robotT
			if (world.team_side == TS_RightSide)
			{
#if USE_FEEDFORWARD == 1
				world.robotT[count].position.setX((robot_blue[i].x()) + World::summation_of_movement_seen_by_vision_mines15to0[i].getX()  /*+ world.output[robot.robot_id()][0]*/);
				world.robotT[count].position.setY((robot_blue[i].y()) + World::summation_of_movement_seen_by_vision_mines15to0[i].getY()  /*+ world.output[robot.robot_id()][1]*/);
				world.robotT[count].uncorrected_position.setX((robot_blue[i].x()));
				world.robotT[count].uncorrected_position.setY((robot_blue[i].y()));
#else
				world.robotT[count].position.setX((robot_blue[i].x()) /*+ world.output[robot.robot_id()][0]*/);
				world.robotT[count].position.setY((robot_blue[i].y()) /*+ world.output[robot.robot_id()][1]*/);
#endif
				world.robotT[count].angle = robot_blue[i].orientation();
				world.robotT[count].timeCaptured = time_capture_blue[i];
				world.robotT[count].timeSent = time_sent_blue[i];
				world.robotT[count].timeRecived = time_recive;
			}
			else if (world.team_side == TS_LeftSide)
			{
#if USE_FEEDFORWARD == 1
				world.robotT[count].position.setX(-(robot_blue[i].x()) + World::summation_of_movement_seen_by_vision_mines15to0[i].getX() /*+ world.output[robot.robot_id()][0]*/);
				world.robotT[count].position.setY(-(robot_blue[i].y()) + World::summation_of_movement_seen_by_vision_mines15to0[i].getY() /*+ world.output[robot.robot_id()][1]*/);
				world.robotT[count].uncorrected_position.setX(-(robot_blue[i].x()));
				world.robotT[count].uncorrected_position.setY(-(robot_blue[i].y()));
#else
				world.robotT[count].position.setX(-(robot_blue[i].x()) /*+ world.output[robot.robot_id()][0]*/);
				world.robotT[count].position.setY(-(robot_blue[i].y()) /*+ world.output[robot.robot_id()][1]*/);
#endif
				world.robotT[count].angle = robot_blue[i].orientation()>=0? robot_blue[i].orientation()-M_PI: robot_blue[i].orientation() + M_PI;
				world.robotT[count].timeCaptured = time_capture_blue[i];
				world.robotT[count].timeSent = time_sent_blue[i];
				world.robotT[count].timeRecived = time_recive;
			}

			///shows that this robot is seen now!
			ssl_robotT_LastUpdate[count] = 0;
		}
		for (int i = 0; i < num_yellow; i++)  //robot_yellow_n ---> number of yellow robot detection .
		{

			int count = 0;
			while ((count < world.numO) && (world.robotO[count].id != robot_yellow[i].robot_id()))
				count++;

			if (world.glTimer.getTime() - time_of_last_seen_opp[robot_yellow[i].robot_id()] > 500) time_of_last_seen_opp[robot_yellow[i].robot_id()] = 0;
			if (count == world.numO)
			{
				if (num_of_last_seen_opp[robot_yellow[i].robot_id()] > 60)
				{
					num_of_last_seen_opp[robot_yellow[i].robot_id()] = 0;
					world.numO++;
					world.robotO[count].id = robot_yellow[i].robot_id();
				}
				else
				{
					num_of_last_seen_opp[robot_yellow[i].robot_id()]++;
					time_of_last_seen_opp[robot_yellow[i].robot_id()] = world.glTimer.getTime();
				}
			}

			if (world.team_side == TS_RightSide)
			{
				world.robotO[count].position.setX(robot_yellow[i].x());
				world.robotO[count].position.setY(robot_yellow[i].y());
				world.robotO[count].angle = robot_yellow[i].orientation();
				world.robotO[count].timeCaptured = time_capture_yellow[i];

			}
			else if (world.team_side == TS_LeftSide)
			{
				world.robotO[count].position.setX(-robot_yellow[i].x());
				world.robotO[count].position.setY(-robot_yellow[i].y());
				world.robotO[count].angle = robot_yellow[i].orientation() >= 0 ? robot_yellow[i].orientation() - M_PI : robot_yellow[i].orientation() + M_PI;
				world.robotO[count].timeCaptured = time_capture_yellow[i];
			}


			ssl_robotO_LastUpdate[count] = 0;
		}
	}
	else if (world.getInstance().team_color == TC_Yellow)
	{
		for (int i = 0; i < num_yellow; i++)  //robot_yellow_n ---> number of yellow robot detection .
		{

			int count = 0;
			while ((count < world.numT) && (world.robotT[count].id != robot_yellow[i].robot_id()))
				count++;
			
			if (world.glTimer.getTime() - time_of_last_seen_team[robot_yellow[i].robot_id()] > 500) 
				time_of_last_seen_team[robot_yellow[i].robot_id()] = 0;
			if (count == world.numT)
			{
				if (num_of_last_seen_team[robot_yellow[i].robot_id()] > 60)
				{
					num_of_last_seen_team[robot_yellow[i].robot_id()] = 0;
					world.numT++;
					world.robotT[count] = Robot();
					world.robotT[count].id = robot_yellow[i].robot_id();
				}
				else
				{
					num_of_last_seen_team[robot_yellow[i].robot_id()]++;
					time_of_last_seen_team[robot_yellow[i].robot_id()] = world.glTimer.getTime();
				}
			}

			if (world.team_side == TS_RightSide)
			{
#if USE_FEEDFORWARD == 1
				world.robotT[count].position.setX( (robot_yellow[i].x())    + World::summation_of_movement_seen_by_vision_mines15to0[i].getX()   );/* + world.output[robot.robot_id()][0]*/
				world.robotT[count].position.setY( (robot_yellow[i].y())    + World::summation_of_movement_seen_by_vision_mines15to0[i].getY()   );/* + world.output[robot.robot_id()][1]*/
				world.robotT[count].uncorrected_position.setX((robot_yellow[i].x()));
				world.robotT[count].uncorrected_position.setY((robot_yellow[i].y()));
#else
				world.robotT[count].position.setX((robot_yellow[i].x()) /* + world.output[robot.robot_id()][0]*/ );
				world.robotT[count].position.setY((robot_yellow[i].y()) /* + world.output[robot.robot_id()][1]*/ );
#endif
				//world.robotT[count].destination_position = world.robotT[world.getIndexForRobotTNumber(robot_yellow[i].robot_id())].destination_position;
				world.robotT[count].angle = robot_yellow[i].orientation();
				world.robotT[count].timeCaptured = time_capture_yellow[i];
				world.robotT[count].timeSent = time_sent_yellow[i];
				world.robotT[count].timeRecived = time_recive;
			}
			
			else if (world.team_side == TS_LeftSide)
			{
#if USE_FEEDFORWARD == 1
				world.robotT[count].position.setX( -(robot_yellow[i].x())   + World::summation_of_movement_seen_by_vision_mines15to0[i].getX()  );/*+ world.output[robot.robot_id()][0]*/
				world.robotT[count].position.setY( -(robot_yellow[i].y())   + World::summation_of_movement_seen_by_vision_mines15to0[i].getY()  );/*+ world.output[robot.robot_id()][1]*/
				world.robotT[count].uncorrected_position.setX(-(robot_yellow[i].x()));
				world.robotT[count].uncorrected_position.setY(-(robot_yellow[i].y()));
#else
				world.robotT[count].position.setX(-(robot_yellow[i].x()) /*+ world.output[robot.robot_id()][0]*/ );
				world.robotT[count].position.setY(-(robot_yellow[i].y()) /*+ world.output[robot.robot_id()][1]*/ );
#endif
				world.robotT[count].angle = robot_yellow[i].orientation() >= 0 ? robot_yellow[i].orientation() - M_PI : robot_yellow[i].orientation() + M_PI;
				world.robotT[count].timeCaptured = time_capture_yellow[i];
				world.robotT[count].timeSent = time_sent_yellow[i];
				world.robotT[count].timeRecived = time_recive;
			}
			ssl_robotT_LastUpdate[count] = 0;
		}
		for (int i = 0; i < num_blue; i++)  //robot_blue_n ---> number of blue robot detection .
		{

			int count = 0;
			while ((count < world.numO) && (world.robotO[count].id != robot_blue[i].robot_id()))
				count++;

			if (world.glTimer.getTime() - time_of_last_seen_opp[robot_blue[i].robot_id()] > 500) time_of_last_seen_opp[robot_blue[i].robot_id()] = 0;
			if (count == world.numO)
			{
				if (num_of_last_seen_opp[robot_blue[i].robot_id()] > 60)
				{
					num_of_last_seen_opp[robot_blue[i].robot_id()] = 0;
					world.numO++;
					world.robotO[count].id = robot_blue[i].robot_id();
				}
				else
				{
					num_of_last_seen_opp[robot_blue[i].robot_id()]++;
					time_of_last_seen_opp[robot_blue[i].robot_id()] = world.glTimer.getTime();
				}
			}


			if (world.team_side == TS_RightSide)
			{
				world.robotO[count].position.setX(robot_blue[i].x());
				world.robotO[count].position.setY(robot_blue[i].y());
				world.robotO[count].angle = robot_blue[i].orientation();
				world.robotO[count].timeCaptured = time_capture_blue[i];

			}
			else if (world.team_side == TS_LeftSide)
			{
				world.robotO[count].position.setX(-robot_blue[i].x());
				world.robotO[count].position.setY(-robot_blue[i].y());
				world.robotO[count].angle = robot_blue[i].orientation() >= 0 ? robot_blue[i].orientation() - M_PI : robot_blue[i].orientation() + M_PI;
				world.robotO[count].timeCaptured = time_capture_blue[i];

			}

			ssl_robotO_LastUpdate[count] = 0;
		}
	}  //end if
	
	///alireza:if one a robot in world.robotT array doesnt see for some tme,it will be removed from the array and all the robots in the array (both
	///world.robotT array and ssl_robotT_LastUpdate array) would be shifted one index toward 0 index
	for (int i = 0; i < world.numT; i++)
	{
		if (ssl_robotT_LastUpdate[i]++ > 60)
		{
			world.numT--;
			for (int j = i; j < world.numT; j++)
			{
				world.robotT[j] = world.robotT[j + 1];
				ssl_robotT_LastUpdate[j] = ssl_robotT_LastUpdate[j + 1];
			}
		}
	}
	for (int i = 0; i < world.numO; i++)
	{
		if (ssl_robotO_LastUpdate[i]++ > 60)
		{
			world.numO--;
			for (int j = i; j < world.numO; j++)
			{
				world.robotO[j] = world.robotO[j + 1];
				ssl_robotO_LastUpdate[j] = ssl_robotO_LastUpdate[j + 1];
			}
		}
	}

	VecPosition tt = 0;
	double temp = 0;

	///alireza:kalman filter
	for (int i = 0; i < world.numT; i++)
	{
		//world.robotT[i].position.setX(world.robotT[i].position.getX() + world.Movement_x);
		//world.robotT[i].position.setY(world.robotT[i].position.getY() + world.Movement_y);
		//RTKF[i].Update(world.robotT[i].timeCaptured, world.robotT[i].position, world.robotT[i].angle, tt, temp, world.robotT[i].velocity, world.robotT[i].w);
		RTKF[i].Update(world.robotT[i].timeCaptured, world.robotT[i].position, world.robotT[i].angle, tt, temp, world.robotT[i].velocity, world.robotT[i].w);
		//world.robotT[i].position = tt;	///alireza:was commented!
		//world.robotT[i].angle = temp;	///alireza:was commented!
		world.robotT[i].velocity *= 1000;
		//temp = 0;	///alireza:was commented!
		//tt = 0;		///alireza:was commented!
	}
	for (int i = 0; i < world.numO; i++)
	{

		ROKF[i].Update(world.robotO[i].timeCaptured, world.robotO[i].position, world.robotO[i].angle, tt, temp, world.robotO[i].velocity, world.robotT[i].w);
		//world.robotO[i].position = tt;	///alireza:was commented!
		//world.robotO[i].angle = temp;	///alireza:was commented!
		world.robotO[i].velocity *= 1000;
		temp = 0;
		tt = 0;
	}


}