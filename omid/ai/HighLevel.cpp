#include "HighLevel.h"
#include "world.h"
#include "geometry.h"
#include "GameState.h"
#include "RRT.h"
////#include "basiclevel.h"
////#include "lowlevel.h"
//#include<stringapiset.h>
#include <stdint.h>
#include <stdio.h>
#include "GL/glut.h"
#include "graphical/glframe.h"
#include <cmath>
#include "Referee.h"
//
//////function 1
////
//////function 2
////
//////function 4
////const	int	MAX_OPPONENT = 0;
////VecPosition get_target(World &world);
////VecPosition get_special_target(int my_rank, int num_of_block_robot, VecPosition general_target, int direction);
////int decider_id = 0;
////GoTowardAndStop a;
////////////mohammadhoss
int UpR_Defence[FieldLength / Length_division][FieldWidth / Width_division];
int UpL_Defence[FieldLength / Length_division][FieldWidth / Width_division];
int DownR_Defence[FieldLength / Length_division][FieldWidth / Width_division];
int DownL_Defence[FieldLength / Length_division][FieldWidth / Width_division];
int UpR_Atack[FieldLength / Length_division][FieldWidth / Width_division];
int UpL_Atack[FieldLength / Length_division][FieldWidth / Width_division];
int DownR_Atack[FieldLength / Length_division][FieldWidth / Width_division];
int DownL_Atack[FieldLength / Length_division][FieldWidth / Width_division];
int  HighLevel::onces_program = 0;
int k = 0;
int u = 0;
int plus_plan_score = 0;
int index_pass_senderT;
const double length_of_goal = Field::getUpBarO().getDistanceTo(Field::getDownBarO());
bool cant_pass_on_the_ground[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
int denger_index_robotT = 0;
int pass_sender, pass_reciver;
int timer_best_pass = 11;
int danger_index_roboto[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
int finde_for_pass = 0;

int HighLevel::robotVelocity;
bool HighLevel::robotMoving;

//////save last data
int sender_robotT_pass, reciver__robotT_pass;
int ready_for_kick = 0;
int danger_robotO[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
int sender, reciver;
VecPosition last_destination_robotT[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
double score_pass[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
int max_pass_score=0;
////dest in highlevel is defence position
////but in main is final robot position
condition_pass HighLevel::pass_mode;
ownership HighLevel::play_mode;
//*****************************
//////////If yoy have spin back
//*****************************
void HighLevel::go_to_ball(int index_robot)
{
    HighLevel::lookAtPos(index_robot, world.ball.getCurrentBallPosition());
    world.robotT[index_robot].destination_position=world.ball.getCurrentBallPosition();
    world.robotT[index_robot].destination_set= true;
}
void HighLevel::go_back_ball(int index) {
    VecPosition intersection1, intersection2;
    Line ball_to_robotO = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(),
                                                      Field::getGoalMidP());
    Circle roboto = Circle(world.ball.getCurrentBallPosition(), 2 * ROBOT_RADIUS + MERGE_DISTANCE);
    ball_to_robotO.getCircleIntersectionPoints(roboto, &intersection1, &intersection2);
        if ( Field::getGoalMidP().getDistanceTo(intersection1) <
                Field::getGoalMidP().getDistanceTo(intersection2))
            world.robotT[index].destination_position = intersection1;
        else
            world.robotT[index].destination_position = intersection2;

    HighLevel::lookAtPos(index, world.ball.getCurrentBallPosition());

}
///*
//	Begining of Farhan Daemi Code
//	 ______         _
//	|  ____|       | |
//	| |__ __ _ _ __| |__   __ _ _ __
//	|  __/ _` | '__| '_ \ / _` | '_ \
//	| | | (_| | |  | | | | (_| | | | |
//	|_|  \__,_|_|  |_| |_|\__,_|_| |_|
//
//	[ This Part of Code is Writen by Farhan Daemi. ]
//
//	Function Details:
//		1.  gotoXY(index, target)										move the robot with given index to target position.
//		2.  lookAt(index, angle)										turn robot with given index to the given angle.
//		3.  move_ball_to_position(index, target)						move the ball from its current positon to the given target position
//		4.  turn_all_spinbacks_on()    									turn all Teamate Robots Spinback on
//		5.  turn_all_spinbacks_off()    								turn all Teamate Robots Spinback off
//		6.  turn_spinbacks_on(int robotIndex)    						turn a single Teamate Robot Spinback on
//		7.  turn_spinbacks_off(int robotIndex)    						turn a single Teamate Robot Spinback on
//		8.  goalKeeper_defend_and_pass(int goalKeeperIndex)    			goalKeeper defending and passing if the ball is in penalty area
//		9.  ball_is_in_penalty_area(char team)    						check is the ball is in penalty area or not. team can be 'T' for Teamate or 'O' for Oponent.
//		10. defence_formation(int number_of_defender)    				form a couple of robots in front of attacking Oponent Robots for covering our goal
//		11. nearest_robot_to_point_except_goali(VecPosition postion)    find the nearest Teamate Robot to a position except the goal keeper
//		12. oponent_is_shooting_index()    								the Oponent robot which is shooting to our goal (-1 for none)
//
//*/
//
//
//int passTimeOut = 0;
//bool setPassTimeOutBefore = false;
//VecPosition last_ball_pos_goalkeeper;
////  1. gotoXY(index, target) -> move the robot with given index to target position.
//void HighLevel::gotoXY(int robotIndex, VecPosition target)
//{
//    world.robotT[world.getIndexForRobotTNumber(robotIndex)].destination_position = target;
//}
////  2. lookAt(index, angle) -> turn robot with given index to the given angle.
//void HighLevel::lookAt(int robotIndex, float angle)
//{
//    world.robotT[world.getIndexForRobotTNumber(robotIndex)].destination_angle = angle;
//}
//bool HighLevel::arivedToPos(int playingRobotNum, VecPosition target)
//{
//    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].position;
//    if(robot_pos.getDistanceTo(target) < 150) return true;
//    else return false;
//}
//int HighLevel::getStageRobot(int playingRobotNum)
//{
//    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].position;
//    int min_dist = 9999999, min_i;
//    for(int i=0; i<world.numT; i++)
//    {
//        VecPosition other_robot_pos = world.robotT[world.getIndexForRobotTNumber(i)].position;
//        if(i != playingRobotNum && other_robot_pos.getDistanceTo(robot_pos) < min_dist)
//        {
//            min_dist = other_robot_pos.getDistanceTo(robot_pos);
//            min_i = i;
//        }
//    }
//    return  min_i;
//}
//bool HighLevel::lookAtPos(int playingRobotNum, VecPosition target)
//{
//    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].position;
//    float robot_angle = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].angle;
//    VecPosition robot_to_targetVec = target - robot_pos;
//    float angle = robot_to_targetVec.AngleBetween(VecPosition(1, 0));
//    if((robot_pos.getY() - target.getY()) >= 0) angle = -angle;
//   // cout<<robot_angle<< "  "<< angle<<'\n';
//    if(abs(robot_angle - angle) < 0.1)
//    {
//        world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].destination_angle = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].angle;
//        return true;
//    }
//    world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].destination_angle = angle;
//    return false;
//}
//VecPosition * HighLevel::get_other_robots(int playingRobotNum)
//{
//    VecPosition other_robots[world.numT - 1];
//    int last_robot_y = - FieldWidth/2;
//    for (int i = 0; i < world.numT-1; ++i) {
//        VecPosition robot_pos;
//        int min_y = 99999;
//        int min_i;
//        for (int j = 0; j < world.numT; ++j) {
//            if(world.getRobotTNumberForIndex(j) != playingRobotNum){
//                VecPosition robot_pos_check = world.robotT[j].position;
//                if(robot_pos_check.getY() > last_robot_y && robot_pos_check.getY() < min_y)
//                {
//                    min_y = robot_pos_check.getY();
//                    robot_pos = robot_pos_check;
//                    min_i = world.getRobotTNumberForIndex(j);
//                }
//            }
//        }
////        cout<<min_i<<'\n';
//        other_robots[i] = robot_pos;
//        last_robot_y = robot_pos.getY() + 50;
//    }
//    cout<<'\n';
//    return other_robots;
//}
//void HighLevel::GoalieDefend(int goalKeeperIndex)
//{
//    // declare some variables to use in algorithm
//    VecPosition ball_pos = world.ball.getCurrentBallPosition();
//    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(goalKeeperIndex)].position;
//    VecPosition ball_velocity_vec = world.ball.getVelocity();
//    float ball_velocity = VecPosition(0,0).getDistanceTo(ball_velocity_vec);
//    Line goal_line, ball_direction;
//    int goaliX = -WholeFieldLength/2 + 300;
//    goal_line = Line::makeLineFromTwoPoints(VecPosition( goaliX, 100), VecPosition( goaliX, -100));
//    VecPosition robot_to_ballVec = ball_pos - robot_pos;
//    float ballAngle = robot_to_ballVec.AngleBetween(VecPosition(1, 0));
//    if((robot_pos.getY() - ball_pos.getY()) >= 0) ballAngle = -ballAngle;
//    VecPosition dest;
//
//    world.team_T.Goalie = goalKeeperIndex; // So important -_-
//    HighLevel::lookAt(goalKeeperIndex, ballAngle);
//    if(ball_velocity > 200) // ball is coming
//    {
//        world.robotT[world.getIndexForRobotTNumber(goalKeeperIndex)].shoot_or_chip = 0;
//        world.robotT[world.getIndexForRobotTNumber(goalKeeperIndex)].kick_power = 0;
//        ball_direction = Line::makeLineFromTwoPoints(last_ball_pos_goalkeeper, ball_pos);
//        dest = ball_direction.getIntersection(goal_line);
//
//        if(ball_pos.getX() > WholeFieldLength/3 && abs(dest.getY() - robot_pos.getY()) > ROBOT_RADIUS*1.5)
//        {
//            if(dest.getY() > robot_pos.getY())
//                dest.setY( PenaltyAreaWidth);
//            else
//                dest.setY(-PenaltyAreaWidth);
//        }
//
//    }
//    else // ball is not moving
//    {
//        if(HighLevel::ball_is_in_penalty_area('T') )//|| oponent_is_shooting_index() == -1)
//        {
//            dest = VecPosition(WholeFieldLength/2 - 300, 0);
//        }
//        else
//        {
//            int nearest_robot_o = HighLevel::nearest_robot_to_ball('T');
//            VecPosition attacker_pos = world.robotT[nearest_robot_o].position;
//            float attacker_angle = (world.robotT[nearest_robot_o].angle * 180)/M_PI;
//            // Line shoot_line = Line::makeLineFromTwoPoints(attacker_pos, ball_pos);
//            Line shoot_line = Line::makeLineFromPositionAndAngle(attacker_pos, attacker_angle);
//            dest = shoot_line.getIntersection(goal_line);
//        }
//        last_ball_pos_goalkeeper = ball_pos;
//    }
//    if(dest.getY() >  PenaltyAreaLength/2 - 200)
//        dest.setY( PenaltyAreaLength/2 - 200);
//    if(dest.getY() < -PenaltyAreaLength/2 + 200)
//        dest.setY(-PenaltyAreaLength/2 + 200);
//    dest.setX(goaliX);
//    HighLevel::gotoXY(goalKeeperIndex, dest);
//    world.robotT[goalKeeperIndex].destination_set=true;
//}
////  9. check is the ball is in penalty area or not. team can be 'T' for Teamate or 'O' for Oponent.
//bool HighLevel::ball_is_in_penalty_area(char team)
//{
//    VecPosition ball_pos = world.ball.getCurrentBallPosition();
//    float ballX = ball_pos.getX();
//    float ballY = ball_pos.getY();
//
//    if(team == 'T')
//    {
//        if(world.team_color == TC_Yellow)
//        {
//            if(ballX > WholeFieldLength/2 - PenaltyAreaWidth  && ballY > -PenaltyAreaLength/2 && ballY < PenaltyAreaLength/2)
//                return true;
//            else
//                return false;
//        }
//        else
//        {
//            if(ballX < -WholeFieldLength/2 + PenaltyAreaWidth  && ballY > -PenaltyAreaLength/2 && ballY < PenaltyAreaLength/2)
//                return true;
//            else
//                return false;
//        }
//    }
//    else
//    {
//        if(world.team_color == TC_Yellow)
//        {
//            if(ballX < -WholeFieldLength/2 + PenaltyAreaWidth  && ballY > -PenaltyAreaLength/2 && ballY < PenaltyAreaLength/2)
//                return true;
//            else
//                return false;
//        }
//        else
//        {
//            if(ballX > WholeFieldLength/2 - PenaltyAreaWidth  && ballY > -PenaltyAreaLength/2 && ballY < PenaltyAreaLength/2)
//                return true;
//            else
//                return false;
//        }
//    }
//}
//
//
///*
//	End of Farhan Daemi Code
//	 ______         _
//	|  ____|       | |
//	| |__ __ _ _ __| |__   __ _ _ __
//	|  __/ _` | '__| '_ \ / _` | '_ \
//	| | | (_| | |  | | | | (_| | | | |
//	|_|  \__,_|_|  |_| |_|\__,_|_| |_|
//
//*/
////
////
//////////////////////////////////////////////////////////m.a//////////////////////////
int HighLevel::go_back_ball(int index,VecPosition desti) {
    VecPosition intersection1, intersection2;
    Line ball_to_robotO = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(),
                                                      desti);
    Circle roboto = Circle(world.ball.getCurrentBallPosition(), 3 * ROBOT_RADIUS);
    ball_to_robotO.getCircleIntersectionPoints(roboto, &intersection1, &intersection2);
    if ( desti.getDistanceTo(intersection1) >
            desti.getDistanceTo(intersection2)) {
        if (world.robotT[index].position.getDistanceTo(intersection1) > 100) {
            world.robotT[index].destination_position = intersection1;
        } else {
            //world.robotT[index].destination_position = world.robotT[index].position;
            world.robotT[index].destination_position=world.ball.getCurrentBallPosition();
            world.robotT[index].shoot_or_chip=1;
            world.robotT[index].kick_power=2;
            return false;
            //HighLevel::Pass(index,world.getIndexForRobotTNumber(7));
        }
    }
        else {
        if (world.robotT[index].position.getDistanceTo(intersection2) > 100) {
            world.robotT[index].destination_position = intersection2;
        } else {
            //world.robotT[index].destination_position = world.robotT[index].position;
            world.robotT[index].destination_position=world.ball.getCurrentBallPosition();
            world.robotT[index].shoot_or_chip=1;
            world.robotT[index].kick_power=2;
            return false;
        }
    }
    VecPosition robot_to_mid_bigest_hollVec = desti - world.robotT[index].position;
    //DrawShape::DrawDot(mid_bigest_holl, 100, 255, 0, 0);
    //	DrawShape::DrawParaline(mid_bigest_holl, world.robotT[index_robotT].position);
    double angle_robot_to_mid_bigest_holl = -((robot_to_mid_bigest_hollVec).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index].position.getY() - desti.getY());
    world.robotT[index].destination_angle=angle_robot_to_mid_bigest_holl;
    return true;
    //HighLevel::lookAtPos(index, world.ball.getCurrentBallPosition());
    }
/*
	Begining of Farhan Daemi Code
	 ______         _
	|  ____|       | |
	| |__ __ _ _ __| |__   __ _ _ __
	|  __/ _` | '__| '_ \ / _` | '_ \
	| | | (_| | |  | | | | (_| | | | |
	|_|  \__,_|_|  |_| |_|\__,_|_| |_|

	[ This Part of Code is Writen by Farhan Daemi. ]

	Function Details:
		1.  gotoXY(index, target)										move the robot with given index to target position.
		2.  lookAt(index, angle)										turn robot with given index to the given angle.
		3.  move_ball_to_position(index, target)						move the ball from its current positon to the given target position
		4.  turn_all_spinbacks_on()    									turn all Teamate Robots Spinback on
		5.  turn_all_spinbacks_off()    								turn all Teamate Robots Spinback off
		6.  turn_spinbacks_on(int robotIndex)    						turn a single Teamate Robot Spinback on
		7.  turn_spinbacks_off(int robotIndex)    						turn a single Teamate Robot Spinback on
		8.  goalKeeper_defend_and_pass(int goalKeeperIndex)    			goalKeeper defending and passing if the ball is in penalty area
		9.  ball_is_in_penalty_area(char team)    						check is the ball is in penalty area or not. team can be 'T' for Teamate or 'O' for Oponent.
		10. defence_formation(int number_of_defender)    				form a couple of robots in front of attacking Oponent Robots for covering our goal
		11. nearest_robot_to_point_except_goali(VecPosition postion)    find the nearest Teamate Robot to a position except the goal keeper
		12. oponent_is_shooting_index()    								the Oponent robot which is shooting to our goal (-1 for none)

*/


int passTimeOut = 0;
bool setPassTimeOutBefore = false;
VecPosition last_ball_pos_goalkeeper;

//  1. gotoXY(index, target) -> move the robot with given index to target position.
void HighLevel::gotoXY(int robotIndex, VecPosition target)
{
    world.robotT[world.getIndexForRobotTNumber(robotIndex)].destination_position = target;
}

//  2. lookAt(index, angle) -> turn robot with given index to the given angle.
void HighLevel::lookAt(int robotIndex, float angle)
{
    world.robotT[world.getIndexForRobotTNumber(robotIndex)].destination_angle = angle;
}

bool HighLevel::arivedToPos(int playingRobotNum, VecPosition target)
{
    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].position;
    if(robot_pos.getDistanceTo(target) < 150) return true;
    else return false;
}

int HighLevel::getStageRobot(int playingRobotNum)
{
    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].position;
    int min_dist = 9999999, min_i;
    for(int i=0; i<world.numT; i++)
    {
        VecPosition other_robot_pos = world.robotT[world.getIndexForRobotTNumber(i)].position;
        if(i != playingRobotNum && other_robot_pos.getDistanceTo(robot_pos) < min_dist)
        {
            min_dist = other_robot_pos.getDistanceTo(robot_pos);
            min_i = i;
        }
    }
    return  min_i;
}

bool HighLevel::lookAtPos(int playingRobotNum, VecPosition target)
{
    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].position;
    float robot_angle = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].angle;
    VecPosition robot_to_targetVec = target - robot_pos;
    float angle = robot_to_targetVec.AngleBetween(VecPosition(1, 0));
    if((robot_pos.getY() - target.getY()) >= 0) angle = -angle;
//    cout<<robot_angle<< "  "<< angle<<'\n';
    if(abs(robot_angle - angle) < 0.6)
    {
        world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].destination_angle = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].angle;
        return true;
    }
    world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].destination_angle = angle;
    return false;
}

VecPosition * HighLevel::get_other_robots(int playingRobotNum)
{
    VecPosition other_robots[world.numT - 1];
    int last_robot_y = - FieldWidth/2;
    for (int i = 0; i < world.numT-1; ++i) {
        VecPosition robot_pos;
        int min_y = 99999;
        int min_i;
        for (int j = 0; j < world.numT; ++j) {
            if(world.getRobotTNumberForIndex(j) != playingRobotNum){
                VecPosition robot_pos_check = world.robotT[j].position;
                if(robot_pos_check.getY() > last_robot_y && robot_pos_check.getY() < min_y)
                {
                    min_y = robot_pos_check.getY();
                    robot_pos = robot_pos_check;
                    min_i = world.getRobotTNumberForIndex(j);
                }
            }
        }
//        cout<<min_i<<'\n';
        other_robots[i] = robot_pos;
        last_robot_y = robot_pos.getY() + 50;
    }
    cout<<'\n';
    return other_robots;
}

void HighLevel::GoalieDefend(int goalKeeperIndex)
{
    // declare some variables to use in algorithm
    VecPosition ball_pos = world.ball.getCurrentBallPosition();
    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(goalKeeperIndex)].position;
    VecPosition ball_velocity_vec = world.ball.getVelocity();
    float ball_velocity = VecPosition(0,0).getDistanceTo(ball_velocity_vec);
    Line goal_line, ball_direction;
    int goaliX;
//    if(world.team_side == 0) // Right Side
        goaliX = (FieldLength/2 - ROBOT_RADIUS*2);
//    else
//        goaliX = -(FieldLength/2 - ROBOT_RADIUS*2);
    goal_line = Line::makeLineFromTwoPoints(VecPosition( goaliX, 100), VecPosition( goaliX, -100));
    VecPosition robot_to_ballVec = ball_pos - robot_pos;
    float ballAngle = robot_to_ballVec.AngleBetween(VecPosition(1, 0));
    if((robot_pos.getY() - ball_pos.getY()) >= 0) ballAngle = -ballAngle;
    VecPosition dest;

    world.team_T.Goalie = goalKeeperIndex; // So important -_-
    HighLevel::lookAt(goalKeeperIndex, ballAngle);
    if(ball_velocity > 200) // ball is coming
    {
        world.robotT[world.getIndexForRobotTNumber(goalKeeperIndex)].shoot_or_chip = 0;
        world.robotT[world.getIndexForRobotTNumber(goalKeeperIndex)].kick_power = 0;
        ball_direction = Line::makeLineFromTwoPoints(last_ball_pos_goalkeeper, ball_pos);
        dest = ball_direction.getIntersection(goal_line);

        if(ball_pos.getX() > WholeFieldLength/3 && abs(dest.getY() - robot_pos.getY()) > ROBOT_RADIUS*1.5)
        {
            if(dest.getY() > robot_pos.getY())
                dest.setY( PenaltyAreaWidth);
            else
                dest.setY(-PenaltyAreaWidth);
        }

    }
    else // ball is not moving
    {
        if(HighLevel::ball_is_in_penalty_area('T') ) //|| oponent_is_shooting_index() == -1)
        {
//            dest = world.ball.getCurrentBallPosition();
//            HighLevel::Pass(world.getIndexForRobotTNumber(goalKeeperIndex), world.getIndexForRobotTNumber(1));
            HighLevel::Shoot(world.getIndexForRobotTNumber(goalKeeperIndex));

        }
        else
        {
            int nearest_robot_o = HighLevel::nearest_robot_to_ball('O');
            VecPosition attacker_pos = world.robotO[nearest_robot_o].position;
            float attacker_angle = (world.robotO[nearest_robot_o].angle * 180)/M_PI;
            // Line shoot_line = Line::makeLineFromTwoPoints(attacker_pos, ball_pos);
            Line shoot_line = Line::makeLineFromPositionAndAngle(attacker_pos, attacker_angle);
            dest = shoot_line.getIntersection(goal_line);
        }
        last_ball_pos_goalkeeper = ball_pos;
    }
    if(dest.getY() >  PenaltyAreaLength/3)
        dest.setY( PenaltyAreaLength/3);
    if(dest.getY() < -PenaltyAreaLength/3)
        dest.setY(-PenaltyAreaLength/3);
    if(!HighLevel::ball_is_in_penalty_area('T')){
        dest.setX(goaliX);
        HighLevel::gotoXY(goalKeeperIndex, dest);
    }

}

//  9. check is the ball is in penalty area or not. team can be 'T' for Teamate or 'O' for Oponent.
bool HighLevel::ball_is_in_penalty_area(char team)
{
    VecPosition ball_pos = world.ball.getCurrentBallPosition();
    float ballX = ball_pos.getX();
    float ballY = ball_pos.getY();

    if(team == 'T')
    {
        if(world.team_color == TC_Yellow)
        {
            if(ballX > FieldLength/2 - PenaltyAreaWidth  && ballY > -PenaltyAreaLength/2 && ballY < PenaltyAreaLength/2)
                return true;
            else
                return false;
        }
        else
        {
            if(ballX < -FieldLength/2 + PenaltyAreaWidth  && ballY > -PenaltyAreaLength/2 && ballY < PenaltyAreaLength/2)
                return true;
            else
                return false;
        }
    }
    else
    {
        if(world.team_color == TC_Yellow)
        {
            if(ballX < -FieldLength/2 + PenaltyAreaWidth  && ballY > -PenaltyAreaLength/2 && ballY < PenaltyAreaLength/2)
                return true;
            else
                return false;
        }
        else
        {
            if(ballX > FieldLength/2 - PenaltyAreaWidth  && ballY > -PenaltyAreaLength/2 && ballY < PenaltyAreaLength/2)
                return true;
            else
                return false;
        }
    }
}

// 10. formate robot
void HighLevel::RobotFormation(){
    for (int i = 0; i < world.numT; i++) {
        if(world.robotT[i].role == "None"){
            VecPosition robot_pos = world.robotT[i].position;
            world.robotT[i].destination_position = HighLevel::get_best_pos_for_cover(robot_pos, FieldLength / 5);
        }
    }
}

// 11. is any of Oponents robots in rectangle
VecPosition HighLevel::get_best_pos_for_cover(VecPosition center, float distance) {
    VecPosition dest = center;
    int nearest_robotO_index = 0;
    for (int i = 0; i < world.numO; i++) {
        VecPosition robot_pos = world.robotO[i].position;
        if(robot_pos.getDistanceTo(center) < distance) {
            if(robot_pos.getDistanceTo(center) < center.getDistanceTo(world.robotO[nearest_robotO_index].position)){
                dest.setX(robot_pos.getX()+500);
                dest.setY(robot_pos.getY());
                nearest_robotO_index = i;
            }
        }
    }
    return dest;
}

void HighLevel::move_ball_to_position(int index, VecPosition pos){


}

/*
	End of Farhan Daemi Code
	 ______         _
	|  ____|       | |
	| |__ __ _ _ __| |__   __ _ _ __
	|  __/ _` | '__| '_ \ / _` | '_ \
	| | | (_| | |  | | | | (_| | | | |
	|_|  \__,_|_|  |_| |_|\__,_|_| |_|

*/

//////////////draw line frome midp to ball circle and return closest intrsection
VecPosition HighLevel::StopSurrounding_get_general_target()
{
	Line ball_to_Goal = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(), Field::getGoalMidP());
	Circle ball_surround(world.ball.getCurrentBallPosition(), MERGE_DISTANCE + ROBOT_RADIUS);//590 is ball radios      //Nearer Distance to ball = 590//

	VecPosition _temp_target[2], target;
	ball_to_Goal.getCircleIntersectionPoints(ball_surround, &(_temp_target[0]), &(_temp_target[1]));
	if (_temp_target[0].getDistanceTo(Field::getGoalMidP()) < _temp_target[1].getDistanceTo(Field::getGoalMidP()))
		target = _temp_target[0];
	else
		target = _temp_target[1];

	return target;
}
///////









//////////////do not use its useless
void HighLevel::BlockOponent(int NumofRobot, int index_robotT[], int dangerer_robotO_index)
{
	VecPosition dest;
	///////////////////////////
	enum blockOpponetMode { normalMode, closeMode, kickOffMode };
	blockOpponetMode mode;
	blockOpponetMode _mode = normalMode;
	////////////////////////

	Line tempLine;

	double angle_mark = 0, distance_mark = 0.70;        // percentage
														//	 angle_mark = 0.20; // percentage
														//	angle_mark = 0.0;

	mode = normalMode;

	switch (mode)
	{
	case normalMode:
		distance_mark = 0.50;
		break;
	case closeMode:
		distance_mark = world.robotO[dangerer_robotO_index].position.getDistanceTo(Field::getGoalMidP());
		distance_mark = (distance_mark - 300) / distance_mark;
		break;
	case kickOffMode:
		tempLine = Line::makeLineFromTwoPoints(world.robotO[dangerer_robotO_index].position, Field::getGoalMidP());
		distance_mark = Line::makeLineFromTwoPoints(VecPosition(150, 0), VecPosition(150, 150)).getIntersection(tempLine).getDistanceTo(Field::getGoalMidP());
		distance_mark = (distance_mark / world.robotO[dangerer_robotO_index].position.getDistanceTo(Field::getGoalMidP()));

		break;
	default:
		break;
	}

	Goal_Shape goal_shape(Field::getGoalMidP().getDistanceTo(world.robotO[dangerer_robotO_index].position) * distance_mark);
	Line dangerer_line_to_goal = Line::makeLineFromTwoPoints(Field::getGoalMidP(), world.robotO[dangerer_robotO_index].position);




	VecPosition blocking_point = goal_shape.getIntersection(dangerer_line_to_goal);
	double angle_robot_to_blocking_point = (blocking_point - world.robotO[dangerer_robotO_index].position).Angle();
	double angle_betwenn_robot_to_blocking_point_and_robot_to_ball = VecPosition::AngleBetweenWithSgn(blocking_point - world.robotO[dangerer_robotO_index].position, world.ball.getCurrentBallPosition() - world.robotO[dangerer_robotO_index].position);
	dest = world.robotO[dangerer_robotO_index].position + VecPosition(world.robotO[dangerer_robotO_index].position.getDistanceTo(blocking_point), Rad2Deg(angle_robot_to_blocking_point + angle_mark * angle_betwenn_robot_to_blocking_point_and_robot_to_ball), POLAR);
	HighLevel::BlockOponent_get_general_target(0);
	several_position_Line(dest, NumofRobot, index_robotT);

}
////End BlockOppponent_Decide

//////////////?
VecPosition HighLevel::StopSurrounding_get_special_target(int my_rank, int num_of_ready_robot, double general_angle, VecPosition ball_position)
{
	//M.A

	AngDeg radius_angle = -8.7;   //-8.44582695947464 ;
	double shift_angle = 0;
	int rank_in_circle = 0;
	int first_rank = 0, last_rank = 0;

	switch (num_of_ready_robot)
	{
	case 1:
		rank_in_circle = 0;
		first_rank = 0;
		last_rank = 0;
		break;
	case 2:
		rank_in_circle = (2 * my_rank - 3);  // (1 ---> -1)  (2 --->  1)
		first_rank = -1;
		last_rank = 1;
		break;
	case 3:
		rank_in_circle = (2 * my_rank - 4);  // (1 ---> -2)  (2 --->  0)  (3 ---> 2)
		first_rank = -2;
		last_rank = 2;
		break;
	case 4:
		rank_in_circle = (2 * my_rank - 5);  // (1 ---> -3)  (2 ---> -1)  (3 ---> 1)  (4 ---> 3)
		first_rank = -3;
		last_rank = 3;
		break;
	case 5:
		rank_in_circle = (2 * my_rank - 6);  // (1 ---> -3)  (2 ---> -1)  (3 ---> 1)  (4 ---> 3)
		first_rank = -4;
		last_rank = 4;
		break;
	case 6:
		rank_in_circle = (2 * my_rank - 7);  // (1 ---> -3)  (2 ---> -1)  (3 ---> 1)  (4 ---> 3)
		first_rank = -5;
		last_rank = 5;
		break;
	case 7:
		rank_in_circle = (2 * my_rank - 8);  // (1 ---> -3)  (2 ---> -1)  (3 ---> 1)  (4 ---> 3)
		first_rank = -6;
		last_rank = 6;
		break;
	case 8:
		rank_in_circle = (2 * my_rank - 9);  // (1 ---> -3)  (2 ---> -1)  (3 ---> 1)  (4 ---> 3)
		first_rank = -7;
		last_rank = 7;
		break;
	}
	double distance_to_ball_for_stop_surrounding = abs(world.robotT[0].position.getX() - world.ball.getCurrentBallPosition().getX());//world.robotT[0].position.getDistanceTo(world.ball.getCurrentBallPosition());
	VecPosition first_positin = ball_position + VecPosition(distance_to_ball_for_stop_surrounding, Rad2Deg(general_angle) + (first_rank * radius_angle), POLAR);
	VecPosition last_position = ball_position + VecPosition(distance_to_ball_for_stop_surrounding, Rad2Deg(general_angle) + (last_rank * radius_angle), POLAR);

	Line _temp_line = Line::makeLineFromPositionAndAngle(Field::getGoalMidP(), 90);
	//draw this line
	//	world.r[0].setVecPosition(field.getGoalMidP().getX(), field.getGoalMidP().getY());
	//	world.r[1].setVecPosition((field.getGoalMidP().getX(), field.getGoalMidP().getY()));
	//End of drawing this line

	VecPosition _temp_intersect[2];
	VecPosition Main_intersect;
	Circle Surround(ball_position, distance_to_ball_for_stop_surrounding);
	//Draw this circle
	//	GLFrame::addCirlceToPainting(Surround);
	//end of drawing this circle

	if (first_positin.getX() > Field::getGoalMidP().getX())
	{
		if (_temp_line.getCircleIntersectionPoints(Surround, _temp_intersect, _temp_intersect + 1) != 0)
		{
			if (_temp_intersect[0].getDistanceTo(Field::getGoalMidP()) < _temp_intersect[1].getDistanceTo(Field::getGoalMidP()))
				Main_intersect = _temp_intersect[0];
			else
				Main_intersect = _temp_intersect[1];

			shift_angle = Rad2Deg(VecPosition::AngleBetweenWithSgn(first_positin - ball_position, Main_intersect - ball_position));
		}
		shift_angle *= 1.2;
		// logStatus("shift angle = " + QString::number(shift_angle),QColor("blue"));
	}
	if (last_position.getX() > Field::getGoalMidP().getX())
	{
		if (_temp_line.getCircleIntersectionPoints(Surround, _temp_intersect, _temp_intersect + 1) != 0)
		{
			if (_temp_intersect[0].getDistanceTo(Field::getGoalMidP()) < _temp_intersect[1].getDistanceTo(Field::getGoalMidP()))
				Main_intersect = _temp_intersect[0];
			else
				Main_intersect = _temp_intersect[1];

			shift_angle = Rad2Deg(VecPosition::AngleBetweenWithSgn(last_position - ball_position, Main_intersect - ball_position));
		}
		shift_angle *= 1.2;
		// logStatus("shift angle = " + QString::number(shift_angle),QColor("blue"));
	}
	return ball_position + VecPosition(distance_to_ball_for_stop_surrounding, Rad2Deg(general_angle) + (rank_in_circle * radius_angle) + shift_angle, POLAR);
}

/////////if you have invader robot id you have intersection roboto and goalshape
VecPosition HighLevel::BlockOponent_get_general_target(int dangerer_robotO_index)
{
	/*////////////////////////////////
	VecPosition last_danger_robot_position;
	VecPosition last_target;

	if (world.robotO[dangerer_robotO_index].position.getDistanceTo(last_danger_robot_position) < 10)
	return last_target;*/

	/*! calculating scale of goal shape ---> "scale" */
	double scale;
	double _temp_distance = Field::getGoalMidP().getDistanceTo(world.robotO[dangerer_robotO_index].position);
	scale = Estimation::getMarkOfMathematicalEquation((1.125*PenaltyAreaRadius), FieldLength, (1.125*PenaltyAreaRadius), FieldLength / 3, _temp_distance);


	/*! creating goal shape include two circle and a line ---> "up_circle" & "down_circle" & "front_goal_line" */

	Goal_Shape goal_shape(scale);

	/*! a line between dangerer robotO position and target ---> "robot_to_target" */

	VecPosition _temp_robot_to_upbar = Field::getUpBarP() - world.robotO[dangerer_robotO_index].position;
	VecPosition _temp_robot_to_downbar = Field::getDownBarP() - world.robotO[dangerer_robotO_index].position;
	double _temp_angle = VecPosition::AngleBetweenWithSgn(_temp_robot_to_upbar, _temp_robot_to_downbar);
	_temp_robot_to_upbar.rotate(_temp_angle / 2);

	Line robot_to_target = Line::makeLineFromPositionAndAngle(world.robotO[dangerer_robotO_index].position, Rad2Deg(_temp_robot_to_upbar.Angle()));




	/*! detecting intersection between "robot_to_target" and goal shape ---> "target" */

	VecPosition target = goal_shape.getIntersection(robot_to_target);

	//last_danger_robot_position = world.robotO[dangerer_robotO_index].position;
	//last_target = target;
	return target;
}
/////**************************************************Faezeh**********************************************///

/////////this function calcute the block vecposition
VecPosition HighLevel::Block_get_target()
{
	/*////////////////////
	//VecPosition point;
	VecPosition ballLastPos;
	VecPosition last_target;
	VecPosition radius_vector;
	ballLastPos = VecPosition(-100000, -100000);
	/////////////////
	if (world.ball.getCurrentBallPosition().getDistanceTo(ballLastPos) < 10)
	return last_target;

	if (world.ball.getCurrentBallPosition().getX() > Field::getGoalMidP().getX())
	return last_target;
	*/
	/*! calculating scale of goal shape ---> "scale" */
	double scale;
	double _temp_distance = Field::getGoalMidP().getDistanceTo(world.ball.getCurrentBallPosition());
	//before: scale = Estimation::getMarkOfMathematicalEquation(1500, 6000, 800, 2000, _temp_distance);
	scale = Estimation::getMarkOfMathematicalEquation((FieldLength / 4), FieldLength, PenaltyAreaRadius, FieldLength / 3, _temp_distance);//it might based on length or penaltyarearadius

																																		  //    if( (_temp_distance < 700) || (_temp_distance > 6380) )
																																		  //        scale = 700 ;
																																		  //    else
																																		  //        scale = (_temp_distance / 4) + 530 ;																																	  /*! creating goal shape */



																																		  /*! a line between ball position and target ---> "ball_to_target" */

	VecPosition _temp_ball_to_upbar = Field::getUpBarP() - world.ball.getCurrentBallPosition();
	VecPosition _temp_ball_to_downbar = Field::getDownBarP() - world.ball.getCurrentBallPosition();
	/////////////mohmmmad?
	double _temp_angle = VecPosition::AngleBetweenWithSgn(_temp_ball_to_upbar, _temp_ball_to_downbar);
	_temp_ball_to_upbar.rotate(_temp_angle / 2);

	Line ball_to_target = Line::makeLineFromPositionAndAngle(world.ball.getCurrentBallPosition(), Rad2Deg(_temp_ball_to_upbar.Angle()));

	/*! detecting intersection between "ball_to_target" and goal shape ---> "target" */

	VecPosition target = Field::getGoalLineP().getIntersection(ball_to_target);
	/*! this is for filling the empty of one side */
	Line ball_to_nearest_bar = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(), world.ball.getCurrentBallPosition().getDistanceTo(Field::getUpBarP()) > world.ball.getCurrentBallPosition().getDistanceTo(Field::getDownBarP()) ? Field::getDownBarP() : Field::getUpBarP());
	VecPosition vec_on_line = ball_to_nearest_bar.getPointOnLineClosestTo(target);
	_temp_distance = world.ball.getCurrentBallPosition().getDistanceTo(target);

	if (target.getY() > 0)
	{
		double _asin = 700 / _temp_distance;
		if (_asin <= 1 && _asin >= -1)
			_asin = asin(_asin);
		else
			_asin = 0;
		VecPosition temp = VecPosition(_temp_distance, Rad2Deg((vec_on_line - world.ball.getCurrentBallPosition()).Angle() - _asin), POLAR);
		//if (temp.toString() == "X:nan Y:nan")
		//{
		//	int x = 9 / 0;
		//}
		target = world.ball.getCurrentBallPosition() + temp;
	}

	else
	{
		double _asin = 700 / _temp_distance;
		if (_asin <= 1 && _asin >= -1)
			_asin = asin(_asin);
		else
			_asin = 0;
		VecPosition temp = VecPosition(_temp_distance, Rad2Deg((vec_on_line - world.ball.getCurrentBallPosition()).Angle() + _asin), POLAR);
		//if (temp.toString() == "X:nan Y:nan")
		//{
		//	int x = 9 / 0;
		//}
		target = world.ball.getCurrentBallPosition() + temp;

	}

	/*! this is for filling the empty of one side */

	/*radius_vector = VecPosition(700, Rad2Deg(_temp_ball_to_upbar.Angle()) - 90, POLAR);
	ballLastPos = world.ball.getCurrentBallPosition();
	last_target = target;*/
	world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = target;
	return target;
}

/////////////////guess the next position of oppoment robot?
//int HighLevel::BlockKicker_Is_Moving(VecPosition newIntersection[])
//{
//
//	VecPosition LastIntersection2[10];
//
//	double subtract1, subtract2, subtract3, subtract4;
//	LastIntersection2[0] = newIntersection[0];
//	for (int i = 4; i < 10; i++)
//	{
//		subtract1 = pow(LastIntersection2[i].getX() - LastIntersection2[i--].getX(), 2) + pow(LastIntersection2[i].getY() - LastIntersection2[i--].getY(), 2);
//		subtract2 = pow(LastIntersection2[i].getX() - LastIntersection2[i - 2].getX(), 2) + pow(LastIntersection2[i].getY() - LastIntersection2[i - 2].getY(), 2);
//		subtract3 = pow(LastIntersection2[i].getX() - LastIntersection2[i - 3].getX(), 2) + pow(LastIntersection2[i].getY() - LastIntersection2[i - 3].getY(), 2);
//		subtract4 = pow(LastIntersection2[i].getX() - LastIntersection2[i - 4].getX(), 2) + pow(LastIntersection2[i].getY() - LastIntersection2[i - 4].getY(), 2);
//
//
//
//		if (subtract2 - subtract1 > 100)
//		{
//			if (subtract3 - subtract1 > 200)
//			{
//				if (subtract4 - subtract1 >  300)
//				{
//					return 0;
//				}
//				else
//				{
//					return -1;
//					break;
//				}
//			}
//			else
//			{
//				return -1;
//				break;
//			}
//		}
//		else
//		{
//			return -1;
//			break;
//		}
//
//
//
//	}
//
//	//if (numOfLastIntersection < 4)
//	//{
//	//	LastIntersection[numOfLastIntersection++] = newIntersection;
//	//	return 0;
//	//}
//}
//
////////////////go to ball and prevent to goal
void HighLevel::StopSurrounding(int NumofRobot, int index_robotT[])
{
	////////////////////////
	//VecPosition ballLastPos;
	//VecPosition last_target;
	//ballLastPos = VecPosition(-10000, -10000);
	//////////////////////////
	StopSurrounding_get_general_target();
	VecPosition dest;

	VecPosition general_target;
	general_target = StopSurrounding_get_general_target();

	if (world.robotT[index_robotT[0]].position.getDistanceTo(general_target) > 700)
	{
		dest = general_target;
	}
	else
	{
		int num_of_ready_robot = 0;
		double angle_of_raedy_robot[5];
		double my_angle = (world.robotT[index_robotT[0]].position - Field::getGoalMidP()).Angle();
		my_angle -= M_PI / 2;
		if (my_angle  < 0)
			my_angle += 2 * M_PI;
		// 90 goes to 0 \\\ 180 goes to 90 \\\ -180 goes to 90 \\\ -90 goes to 180
		for (int i = 0; i < NumofRobot; i++)
			if (world.robotT[index_robotT[i]].position.getDistanceTo(general_target) < 700)
			{

				//logStatus(QString::number(decider.id) + ":"+QString::number(deciders[i].id) ,QColor("blue"));
				angle_of_raedy_robot[num_of_ready_robot] = (world.robotT[i].position - Field::getGoalMidP()).Angle();
				angle_of_raedy_robot[num_of_ready_robot] -= M_PI / 2;
				if (angle_of_raedy_robot[num_of_ready_robot]  < 0)
					angle_of_raedy_robot[num_of_ready_robot] += 2 * M_PI;
				// 90 goes to 0 \\\ 180 goes to 90 \\\ -180 goes to 90 \\\ -90 goes to 180
				num_of_ready_robot++;
			}

		int my_rank_in_angle = 1;
		for (int i = 0; i < num_of_ready_robot; i++)
			if (angle_of_raedy_robot[i] < my_angle)
				my_rank_in_angle++; // rank is the robots angle that have less angle than my_anlge

		VecPosition special_target = StopSurrounding_get_special_target(my_rank_in_angle, num_of_ready_robot + 1, (general_target - world.ball.getCurrentBallPosition()).Angle(), world.ball.getCurrentBallPosition());

		dest = special_target;

	}

	several_position_Line(dest, NumofRobot, index_robotT);///

}
//
//void HighLevel::AntiOneTouch_positionball(World &world, int id)
//{
//	int decider_id = world.decider_id;
//	VecPosition dest;
//	double angle;
//	angle = M_PI;
//	string  condition = "";//update condition robot
//	dest = Field::getGoalMidP();
//	angle = (VecPosition(0, 0) - Field::getGoalMidP()).Angle();
//
//
//
//	const	int	MAX_OPPONENT = 0;
//	VecPosition get_target(World &world);
//	VecPosition get_special_target(int my_rank, int num_of_block_robot, VecPosition general_target, int direction);
//
//
//
//	double angbetween;
//	double distance_to_ball;
//	double distance_to_goal;
//	//double opponent_mark[MAX_OPPONENT];
//	double opponent_mark[11];
//
//	for (int i = 0; i < world.numO; i++)
//	{
//		angbetween = (world.robotO[i].position - world.ball.getCurrentBallPosition()).AngleBetween(world.ball.getVelocity());
//		angbetween = ((M_PI / 2 - angbetween) / (M_PI / 2)) * 100;
//
//		distance_to_ball = world.robotO[i].position.getDistanceTo(world.ball.getCurrentBallPosition()) - 80;
//		if (distance_to_ball < 200)
//			distance_to_ball = 100;
//		else if (distance_to_ball > 3000)
//			distance_to_ball = 0;
//		else
//			distance_to_ball = ((3000 - distance_to_ball) / (3000 - 200)) * 100;
//
//		distance_to_goal = world.robotO[i].position.getDistanceTo(Field::getGoalMidP());
//		if (distance_to_goal < 500)
//			distance_to_goal = 100;
//		else if (distance_to_goal > 4000)
//			distance_to_goal = 0;
//		else
//			distance_to_goal = ((4000 - distance_to_ball) / (4000 - 500)) * 100;
//
//		opponent_mark[i] = angbetween * 0.33 + distance_to_ball * 0.33 + distance_to_goal * 0.33;
//	}
//
//	int index_of_dangerer_opponent = 0;
//	for (int i = 1; i < world.numO; i++)
//		if (opponent_mark[index_of_dangerer_opponent] < opponent_mark[i])
//			index_of_dangerer_opponent = i;
//
//	// world.robotO[index_of_dangerer_opponent].position
//
//	//////////////////////////////// Saeed Algorithm
//
//	Cone cone(world.robotO[index_of_dangerer_opponent].position, Field::getUpBarP(), Field::getDownBarP());
//	Circle robots[10];
//	int num_of_robots = 0;
//
//	int index_goali = Estimation::get_nearest_T(world, Field::getGoalMidP());
//
//	for (int i = 0; i < world.numT; i++)
//		if (index_goali != i)
//		{
//			robots[num_of_robots] = Circle(world.robotT[i].position, 90);
//			num_of_robots++;
//		}
//	//    for (num_of_robots = 0 ; num_of_robots < world.numT ; num_of_robots++ )
//	//        if( num_of_robots != index_goali )
//	//            robots[num_of_robots] = Circle(world.robotT[num_of_robots].position ,90 );
//
//	int num_of_hole;
//	Cone::Hole_Type holess[10];
//	Paraline holes[10];
//	//num_of_hole = cone.get_paraline(robots, num_of_robots, holes) ;
//
//	cone.Get_Free_Space_In_Cone(robots, num_of_robots, holess, num_of_hole);
//	for (int i = 0; i < num_of_hole; i++)
//		holes[i] = Paraline(holess[i].Point_1, holess[i].Point_2);
//
//
//	Paraline bigest_hole2;
//
//	bigest_hole2 = bigest_hole2.getMaxParaline(holes, num_of_hole);
//
//
//	/*! calculating scale of goal shape ---> "scale" */
//
//	double scale;
//	double _temp_distance = Field::getGoalMidP().getDistanceTo(world.robotO[index_of_dangerer_opponent].position);
//	scale = _temp_distance < 2000 ? _temp_distance / 2000 : 1;
//	scale *= 400;
//
//	/*! creating goal shape */
//
//	Goal_Shape goal_shape(scale);
//
//	/*! a line between ball position and target ---> "ball_to_target" */
//
//	VecPosition _temp_ball_to_up_of_hole = VecPosition(Field::getUpBarP().getX(), bigest_hole2.getFirstPoint().getY()) - world.robotO[index_of_dangerer_opponent].position;
//	VecPosition _temp_ball_to_down_of_hole = VecPosition(Field::getDownBarP().getX(), bigest_hole2.getSecondPoint().getY()) - world.robotO[index_of_dangerer_opponent].position;
//	double _temp_angle = _temp_ball_to_up_of_hole.AngleBetween(_temp_ball_to_down_of_hole);
//	_temp_ball_to_up_of_hole.rotate(-_temp_angle / 2);
//
//	Line ball_to_target = Line::makeLineFromPositionAndAngle(world.robotO[index_of_dangerer_opponent].position, Rad2Deg(_temp_ball_to_up_of_hole.Angle()));
//
//	/*! detecting intersection between "ball_to_target" and goal shape ---> "target" */
//
//
//	VecPosition target = goal_shape.getIntersection(ball_to_target);
//
//	AngDeg degree = Rad2Deg(_temp_ball_to_up_of_hole.Angle());
//
//	//target -= VecPosition(100,0);
//	if (((degree <= 110) && (degree >= 70)) || ((degree < -70) && (degree > -110)))
//	{
//		target = bigest_hole2.getMidPoint() + (world.robotO[index_of_dangerer_opponent].position - bigest_hole2.getMidPoint()).normalize() * scale;
//	}
//
//	angle = (world.robotO[index_of_dangerer_opponent].position - world.robotT[decider_id].position).Angle();
//
//
//	world.robotT[world.getIndexForRobotNumber(id)].destination_position = target;
//
//
//
//	/*world.dest = target;*/
//	/*decider.setMeduimLevel("MNone");
//	decider.setLowLevel("GoTowardAndStop");
//	((GoTowardAndStop*)(decider.lowlevels[decider.currentLowLevel]))->SetParam(target, angle, 2000);
//	((GoTowardAndStop*)(decider.lowlevels[decider.currentLowLevel]))->decide(world);*/
//}
//
//void HighLevel::AntiOneTouch(World &world, int NumofRobot, int index_robotT[])
//{
//	//////////define vale
//	int sizerobot;
//	///////////////////////////////
//
//	VecPosition dest;
//	double angle;
//	int index_robot_o;
//	dest = Field::getGoalMidP();
//	angle = (VecPosition(0, 0) - Field::getGoalMidP()).Angle();
//	////////////////////////////////
//	int mark_Free_Space_robotO[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
//	int mark_nearest_opponent_robotO[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
//	int mark_angle_togoal_robotO[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
//	int mark_Free_Space_goalto_robotO[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
//	int mark[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
//
//	for (int i = 0; i < world.numO; i++)
//	{
//		mark[i] = 0;
//		mark_Free_Space_robotO[i] = 0;
//		mark_nearest_opponent_robotO[i] = 0;
//		mark_angle_togoal_robotO[i] = 0;
//		mark_Free_Space_goalto_robotO[i] = 0;
//	}
//
//	Cone::Hole_Type Longest_Hole;
//	Circle robots[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
//	int num_of_robots = 0;
//	VecPosition mid_bigest_holl;
//	Cone BallToGoal(world.ball.getCurrentBallPosition(), Field::getUpBarO(), Field::getDownBarO());
//	int index_goali = 0;
//	for (int d = 0; d < world.numO; d++)
//		if (world.robotO[d].id == world.team_O.Goalie)
//			index_goali = d;//Estimation::get_nearest_T(world, Field::getGoalMidP());
//	for (int i = 0; i < world.numO; i++)
//		/*if (index_goali != i)*/
//		robots[num_of_robots++] = Circle(world.robotO[i].position, 90);
//
//	for (int i = 0; i < world.numT; i++)
//		/*	if (index_goali != i)*/
//		robots[num_of_robots++] = Circle(world.robotT[i].position, 90);
//
//
//	/////////////////////////////////////////////////////
//
//	Cone  RobotOToGoalHole[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
//	for (int i = 0; i < world.numO; i++)
//	{
//		RobotOToGoalHole[i].set_ball(world.robotO[i].position);
//		RobotOToGoalHole[i].set_down_bar(Field::getDownBarP());
//		RobotOToGoalHole[i].set_up_bar(Field::getUpBarP());
//	}
//	int k = 0;
//	int kk = 1;
//
//	for (int i = 0; i < world.numO; i++)
//	{
//		if (RobotOToGoalHole[i].Get_Free_Space_In_Cone(robots, num_of_robots, Longest_Hole) > 0)
//		{
//			mark_Free_Space_robotO[i] = 1;
//
//			//world.r[k] = world.robotO[i].position;
//			//world.r[kk] = (0,0);
//			//k++;
//			//kk++;
//			//k++;
//			//kk++;
//		}
//	}
//
//	/////////////////////////////////////////////////////
//
//	double nearest_opponent = world.robotO[0].position.getDistanceTo(Field::getGoalMidP());
//	int index_of_nearest_opponent = 0;
//	for (int i = 1; i < world.numO; i++)
//	{
//		if (world.robotO[i].position.getDistanceTo(Field::getGoalMidP()) < nearest_opponent)
//		{
//			nearest_opponent = world.robotO[i].position.getDistanceTo(Field::getGoalMidP());
//			index_of_nearest_opponent = i;
//			mark_nearest_opponent_robotO[i] = 2;
//
//
//			//world.r[k] = world.robotO[i].position;
//			//world.r[kk] = (0, 0);
//			//k++;
//			//kk++;
//			//k++;
//			//kk++;
//		}
//	}
//	//////////////////////////////////////////////////////
//
//
//	Line line_distance = Line::makeLineFromTwoPoints(Field::getDownBarP(), Field::getUpBarP());
//	for (int i = 0; i < world.numO; i++)
//	{
//		Line robot_distance_to_goal = Line::makeLineFromPositionAndAngle(world.robotO[i].position, Rad2Deg(world.robotO[i].angle));
//		if (robot_distance_to_goal.getIntersection(line_distance).isBetween(Field::getDownBarP(), Field::getUpBarP()))
//		{
//			mark_angle_togoal_robotO[i] = 3;
//
//
//			//world.r[k] = world.robotO[i].position;
//			//world.r[kk] = (0, 0);
//			//k++;
//			//kk++;
//			//k++;
//			//kk++;
//		}
//	}
//
//	////////////////////////////////////////////////////
//
//
//	int index_of_nearest_roboto = Estimation::get_nearest_O(world, world.ball.getCurrentBallPosition());
//
//	AngRad angle_nearest_roboto = world.robotO[index_of_nearest_roboto].angle;
//
//	for (int i = 0; i < world.numO; i++)
//	{
//		AngRad angle_nearest_roboto_i = world.robotO[i].angle;
//
//		if (i != index_of_nearest_roboto)
//			if (-1 * angle_nearest_roboto_i + 0.05 <= angle_nearest_roboto <= 1 * angle_nearest_roboto_i + 0.05)
//			{
//				mark_Free_Space_goalto_robotO[i] = 4;
//
//				/*world.r[k] = world.robotO[i].position;*/
//				//world.r[kk] = (0, 0);
//				//k++;
//				//kk++;
//				//k++;
//				//kk++;
//			}
//	}
//
//	////////////////////////////
//	for (int i = 0; i < world.numO; i++)
//	{
//		mark[i] = mark_Free_Space_robotO[i]
//			+ mark_nearest_opponent_robotO[i]
//			+ mark_angle_togoal_robotO[i]
//			+ mark_Free_Space_goalto_robotO[i];
//	}
//	int max_mark = mark[0];
//	int max_mark_index = 0;
//	for (int i = 1; i < world.numO; i++)
//	{
//		if (mark[i] > max_mark)
//		{
//			max_mark = mark[i];
//			max_mark_index = i;
//			continue;
//		}
//
//		else
//			if (mark[i] == max_mark)
//			{
//				if (mark_Free_Space_goalto_robotO[i] >= mark_Free_Space_goalto_robotO[max_mark_index])
//				{
//					max_mark = mark[i];
//					max_mark_index = i;
//					continue;
//				}
//				else
//				{
//					max_mark = mark[max_mark_index];
//					continue;
//				}
//
//			}
//			else
//			{
//				max_mark = mark[i];
//				max_mark_index = i;
//				continue;
//			}
//
//	}
//	cout << "max_mark_index" << max_mark_index << endl;
//	//world.r[2] = world.robotO[max_mark_index].position;
//	//world.r[3] = (0,0);
//	//////////////////////////////////
//
//	Cone cone(world.robotO[max_mark_index].position, Field::getUpBarP(), Field::getDownBarP());
//	/*	Circle robots[10];*/
//
//
//	index_goali = Estimation::get_nearest_T(world, Field::getGoalMidP());
//	num_of_robots = 0;
//	for (int i = 0; i < world.numT; i++)
//		if (index_goali != i)
//		{
//			robots[num_of_robots] = Circle(world.robotT[i].position, 90);
//			num_of_robots++;
//		}
//	//    for (num_of_robots = 0 ; num_of_robots < world.numT ; num_of_robots++ )
//	//        if( num_of_robots != index_goali )
//	//            robots[num_of_robots] = Circle(world.robotT[num_of_robots].position ,90 );
//
//	int num_of_hole;
//	Cone::Hole_Type holess[10];
//	Paraline holes[10];
//	//num_of_hole = cone.get_paraline(robots, num_of_robots, holes) ;
//
//	cone.Get_Free_Space_In_Cone(robots, num_of_robots, holess, num_of_hole);
//	for (int i = 0; i < num_of_hole; i++)
//		holes[i] = Paraline(holess[i].Point_1, holess[i].Point_2);
//
//	//????
//	Paraline bigest_hole2;
//
//	bigest_hole2 = bigest_hole2.getMaxParaline(holes, num_of_hole);
//
//
//	/*! calculating scale of goal shape ---> "scale" */
//
//	double scale;
//	double _temp_distance = Field::getGoalMidP().getDistanceTo(world.robotO[max_mark_index].position);
//	scale = _temp_distance < 2000 ? _temp_distance / 2000 : 1;
//	scale *= 400;
//
//	/*! creating goal shape */
//
//	Goal_Shape goal_shape(scale);
//
//	/*! a line between ball position and target ---> "ball_to_target" */
//
//	VecPosition _temp_ball_to_up_of_hole = VecPosition(Field::getUpBarP().getX(), bigest_hole2.getFirstPoint().getY()) - world.robotO[max_mark_index].position;
//	VecPosition _temp_ball_to_down_of_hole = VecPosition(Field::getDownBarP().getX(), bigest_hole2.getSecondPoint().getY()) - world.robotO[max_mark_index].position;
//	double _temp_angle = _temp_ball_to_up_of_hole.AngleBetween(_temp_ball_to_down_of_hole);
//	_temp_ball_to_up_of_hole.rotate(-_temp_angle / 2);
//
//	Line ball_to_target = Line::makeLineFromPositionAndAngle(world.robotO[max_mark_index].position, Rad2Deg(_temp_ball_to_up_of_hole.Angle()));
//
//	/*! detecting intersection between "ball_to_target" and goal shape ---> "target" */
//
//
//	VecPosition target = goal_shape.getIntersection(ball_to_target);
//
//	AngDeg degree = Rad2Deg(_temp_ball_to_up_of_hole.Angle());
//
//	//target -= VecPosition(100,0);
//	if (((degree <= 110) && (degree >= 70)) || ((degree < -70) && (degree > -110)))
//	{
//		target = bigest_hole2.getMidPoint() + (world.robotO[max_mark_index].position - bigest_hole2.getMidPoint()).normalize() * scale;
//	}
//	dest = target;
//
//	several_position_Line(world, dest, NumofRobot);///
//
//	AngDeg  finalangle[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
//	//world.finalyDest[0] = dest;///
//	/*decider.setMeduimLevel("MNone");
//	decider.setLowLevel("GoTowardAndStop");
//	((GoTowardAndStop*)(decider.lowlevels[decider.currentLowLevel]))->SetParam(target, angle, 2000);
//	((GoTowardAndStop*)(decider.lowlevels[decider.currentLowLevel]))->decide(world);*/
//}
//
//////////goler defendig in penalty mood or other mood
//////////need goler id
void HighLevel::CatchPenalty(World &world, int index)
{
	///////////////////////////
	VecPosition decided_point;
	AngRad decided_angle;
	decided_point = Field::getGoalMidP();
	decided_angle = M_PI;
	///////////////////////////
	int index_nearest = Estimation::get_nearest_O(world, world.ball.getCurrentBallPosition());
	Line L = Line::makeLineFromPositionAndAngle(world.robotO[index_nearest].position, Rad2Deg(world.robotO[index_nearest].angle));
	VecPosition p1(Field::getUpBarP().getX() - ROBOT_RADIUS, Field::getUpBarP().getY());
	VecPosition p2(Field::getDownBarP().getX() - ROBOT_RADIUS, Field::getDownBarP().getY());
	Line GoalLine = Line::makeLineFromTwoPoints(p1, p2);
	VecPosition intersect = L.getIntersection(GoalLine);
	if (mode_State::Play && Estimation::ballLine.getIntersection(Field::getGoalLineP()).isBetween(Field::getDownBarP(), Field::getUpBarP()))
	{
		decided_point = Estimation::ballLine.getIntersection(Field::getGoalLineP());
		if (decided_point.getDistanceTo(Field::getUpBarP()) < 90 || decided_point.getDistanceTo(Field::getDownBarP()) < 90)
		{
			decided_point += VecPosition(-1, 0) * (90 - Field::getGoalLineP().getDistanceWithPoint(decided_point));
		}
	}
	else if (intersect.getY() < Field::getUpBarP().getY() - 50 && intersect.getY() > Field::getDownBarP().getY() + 50)
	{
		decided_angle = (world.ball.getCurrentBallPosition() - intersect).Angle();
	}

	world.robotT[index].destination_position = decided_point;
	world.robotT[index].destination_angle = decided_angle;
}

void HighLevel::ReadyForKick(int index)
{
	if (index != ready_for_kick)
	{
		cout << "ready for kick= " << world.getRobotTNumberForIndex(index) << endl;
	}
	ready_for_kick = index;

	VecPosition intersect1, intersect2, dest;
	Line robot_to_ball = Line::makeLineFromTwoPoints(world.robotT[index].position, world.ball.getCurrentBallPosition());
	Line ball_to_goal = Line::makeLineFromTwoPoints(Field::getGoalMidP(), world.ball.getCurrentBallPosition());
	VecPosition robot_to_ballVec = world.ball.getCurrentBallPosition() - world.robotT[index].position;
	if ((world.robotT[index].position.getY() - world.ball.getCurrentBallPosition().getY()) <= 0)
		world.robotT[index].destination_angle = (robot_to_ballVec).AngleBetween(VecPosition(1, 0));
	else
		world.robotT[index].destination_angle = -((robot_to_ballVec).AngleBetween(VecPosition(1, 0)));
	Circle ball_area(world.ball.getCurrentBallPosition(), MERGE_DISTANCE);
	ball_to_goal.getCircleIntersectionPoints(ball_area, &intersect1, &intersect2);
	dest = Field::getGoalMidP().getFarest(intersect1, intersect2);
	AngRad angleB_ball2tobot_ball2dest = (-robot_to_ballVec).AngleBetween(dest - world.ball.getCurrentBallPosition());
	double distance_to_ball = world.robotT[index].position.getDistanceTo(world.ball.getCurrentBallPosition());
	if (distance_to_ball <  MERGE_DISTANCE + ROBOT_RADIUS)
	{
		robot_to_ball.getCircleIntersectionPoints(ball_area, &intersect1, &intersect2);
		VecPosition intersection = world.robotT[index].position.getNearer(intersect1, intersect2);
		world.robotT[index].destination_position = intersection;
	}
	else if (angleB_ball2tobot_ball2dest < M_PI / 2)
	{
		robot_to_ball.getCircleIntersectionPoints(ball_area, &intersect1, &intersect2);
		VecPosition intersection = world.robotT[index].position.getNearer(intersect1, intersect2);
		world.robotT[index].destination_position = intersection;
	}
	else
	{
		Line perpendicular_robot_to_ball = robot_to_ball.getTangentLine(world.ball.getCurrentBallPosition());
		perpendicular_robot_to_ball.getCircleIntersectionPoints(ball_area, &intersect1, &intersect2);
		VecPosition tangantVec = dest.getNearer(intersect1, intersect2);
		world.robotT[index].destination_position = tangantVec;
	}

	Line ball_to_index2 = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(), world.robotT[index].position);
	VecPosition robot1_to_robot2 = world.ball.getCurrentBallPosition() - world.robotT[index].position;
	world.robotT[index].destination_angle = -((robot1_to_robot2).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index].position.getY() - world.ball.getCurrentBallPosition().getY());
	world.robotT[index].destination_set = true;
}
void HighLevel::Ready(World &world, int index)
{
	VecPosition intersect1, intersect2, dest;

	Line robot_to_ball = Line::makeLineFromTwoPoints(world.robotT[index].position, world.ball.getCurrentBallPosition());
	Line ball_to_goal = Line::makeLineFromTwoPoints(Field::getGoalMidP(), world.ball.getCurrentBallPosition());
	VecPosition robot_to_ballVec = world.ball.getCurrentBallPosition() - world.robotT[index].position;
	Circle ball_area(world.ball.getCurrentBallPosition(), BALL_RADIUS);
	ball_to_goal.getCircleIntersectionPoints(ball_area, &intersect1, &intersect2);
	dest = Field::getGoalMidP().getFarest(intersect1, intersect2);
	AngRad angleB_ball2tobot_ball2dest = (-robot_to_ballVec).AngleBetween(dest - world.ball.getCurrentBallPosition());
	double distance_to_ball = world.robotT[index].position.getDistanceTo(world.ball.getCurrentBallPosition());
	if (distance_to_ball <  BALL_RADIUS + 10)
	{

	}
	else if (angleB_ball2tobot_ball2dest < M_PI / 2)
	{
		robot_to_ball.getCircleIntersectionPoints(ball_area, &intersect1, &intersect2);
		VecPosition intersection = world.robotT[index].position.getNearer(intersect1, intersect2);
		world.robotT[index].destination_position = intersection;
	}
	else
	{
		Line perpendicular_robot_to_ball = robot_to_ball.getTangentLine(world.ball.getCurrentBallPosition());
		perpendicular_robot_to_ball.getCircleIntersectionPoints(ball_area, &intersect1, &intersect2);
		VecPosition tangantVec = dest.getNearer(intersect1, intersect2);
		world.robotT[index].destination_position = tangantVec;
	}
}


//////////linear defense
void HighLevel::several_position_Line(VecPosition dest, int NumofRobot, int index_robott[])
{

	if (NumofRobot > 1)
	{
		int n = -1;
		int m = 0;
		for (int q = 0; q < NumofRobot; q++)
		{
			if (NumofRobot % 2 == 0)
			{
				if (q % 2 == 0)
					n = n + 2;
				if (dest.getY() >(FieldWidth / 10))
					world.robotT[index_robott[q]].destination_position = VecPosition((dest.getX() + pow(-1, q)* n * 85), (dest.getY() + pow(-1, q)* n * 85));
				else if (dest.getY() < -(FieldWidth / 10))
					world.robotT[index_robott[q]].destination_position = VecPosition((dest.getX() - pow(-1, q)* n * ROBOT_RADIUS), (dest.getY() + pow(-1, q)* n * ROBOT_RADIUS));
				else
					world.robotT[index_robott[q]].destination_position = VecPosition((dest.getX()), (dest.getY() + pow(-1, q)* n * ROBOT_RADIUS));

			}
			else
			{
				if (q % 2 != 0)
					m = m + 2;
				if (q == 0)
					world.robotT[index_robott[q]].destination_position = VecPosition((dest.getX()), (dest.getY()));
				else if (dest.getY() > (FieldWidth / 10))
					world.robotT[index_robott[q]].destination_position = VecPosition((dest.getX() + pow(-1, q + 1)* m * ROBOT_RADIUS), (dest.getY() + pow(-1, q + 1)* m * ROBOT_RADIUS));
				else if (dest.getY() < -(FieldWidth / 10))
					world.robotT[index_robott[q]].destination_position = VecPosition((dest.getX() - pow(-1, q + 1)* m * ROBOT_RADIUS), (dest.getY() + pow(-1, q + 1)* m * ROBOT_RADIUS));
				else
					world.robotT[index_robott[q]].destination_position = VecPosition((dest.getX()), (dest.getY() + pow(-1, q + 1)* m * ROBOT_RADIUS));
			}
		}
	}
	else
		world.robotT[index_robott[0]].destination_position = dest;
}



//////////////do not use its useless?
//void HighLevel::StartT(World &world)
//{
//
//}
//////////////do not use its useless?
//void HighLevel::StartO(World &world)
//{
//	int n = MAX_ROBOTS_PER_TEAM_IN_THE_FIELD - 1;
//
//	for (int i = 1; i <= MAX_ROBOTS_PER_TEAM_IN_THE_FIELD / 2; i++)
//	{
//
//	}
//}
////void HighLevel::Start_GAME(World &world, VecPosition dest, int NumofRobot)
////{
////
////}
////void HighLevel::Goli_several_position(World &world, VecPosition dest, int NumofRobot)
////{
////	VecPosition p1(Field::getUpBarP().getX() - 85, Field::getUpBarP().getY());
////	VecPosition p2(Field::getDownBarP().getX() - 85, Field::getDownBarP().getY());
////	Line GoalLine = Line::makeLineFromTwoPoints(p1, p2);
////	Line L = Line::makeLineFromTwoPoints(dest, (p1 + p2) / 2);
////	VecPosition intersect = L.getIntersection(GoalLine);//position robot Goli
////
////}
//
/////***********************************************mohadeseh**********************************************///
VecPosition HighLevel::Block_get_special_target(int my_rank, int num_of_block_robot, VecPosition general_target, int direction)
{
	VecPosition shift_vector = VecPosition(0, 0);
	int rank_in_line = 0;
	int first_rank = 0, last_rank = 0;

	if (direction > 0)
	{
		switch (num_of_block_robot)
		{
		case 1:
			rank_in_line = 0;
			first_rank = 0;
			last_rank = 0;
			break;
		case 2:
			rank_in_line = (2 * my_rank - 2);  // (1 ---> 0)  (2 --->  2)
			first_rank = 0;
			last_rank = 2;
			break;
		case 3:
			rank_in_line = (2 * my_rank - 2);  // (1 --->  0)  (2 --->  2)  (3 --->  4)
			first_rank = 0;
			last_rank = 4;
			break;
		}
	}
	else if (direction < 0)
	{
		switch (num_of_block_robot)
		{
		case 1:
			rank_in_line = 0;
			first_rank = 0;
			last_rank = 0;
			break;
		case 2:
			rank_in_line = (2 * my_rank - 4);  // (1 ---> -2)  (2 --->  0)
			first_rank = -2;
			last_rank = 0;
			break;
		case 3:
			rank_in_line = (2 * my_rank - 6);  // (1 --->  -4)  (2 --->  -2)  (3 --->  0)
			first_rank = -4;
			last_rank = 0;
			break;
		}

	}
	//	Block::get_target(&world);
	return general_target + (/*radius_vector*/ 0 * rank_in_line) + shift_vector;
}
//
//
void HighLevel::Block(int NumofRobot, int index_robotT[])
{
	VecPosition dest;
	double angle;
	/////////////////////???????????
	VecPosition point;
	VecPosition ballLastPos;
	VecPosition last_target;
	VecPosition radius_vector;
	ballLastPos = VecPosition(-10000, -10000);
	/////////////////??????????????????
	VecPosition general_target = Block_get_target();

	//if (world.robotT[index_robotT[0]].position.getDistanceTo(general_target) > 700)
	//{
	//	angle = (world.ball.getCurrentBallPosition() - general_target).Angle();
	//	dest = general_target;

	//	decider.setMeduimLevel("GoInPathAndStop");
	//((GoInPathAndStop*)(decider.mediumlevels[decider.currentMediumLevel]))->SetParam(general_target, (world.ball.getCurrentBallPosition() - general_target).Angle(), config->getRobotsMaxSpeed(), 0);
	//	((GoInPathAndStop*)(decider.mediumlevels[decider.currentMediumLevel]))->decide(world);
	//}
	//else
	{
		int num_of_block_robot = 0;
		double angle_of_block_robot[5];
		double my_angle = (world.robotT[index_robotT[0]].position - Field::getGoalMidP()).Angle();

		bool special_mode = false;

		special_mode = true;  /// WWWWWWWHHHHHHHHHHHYYYYYYYYYYYYYYY
		my_angle += M_PI / 2;
		if (my_angle < 0)
			my_angle += 2 * M_PI;

		for (int i = 0; i < NumofRobot; i++)
		{
			//if ( deciders[i].index != world.decider_id)
			angle_of_block_robot[num_of_block_robot] = (world.robotT[i].position - Field::getGoalMidP()).Angle();
			if (special_mode)
			{
				angle_of_block_robot[num_of_block_robot] += M_PI / 2;
				if (angle_of_block_robot[num_of_block_robot] < 0)
					angle_of_block_robot[num_of_block_robot] += 2 * M_PI;
			}
			num_of_block_robot++;
		}

		int my_rank_in_angle = 1;
		for (int i = 0; i < num_of_block_robot; i++)
			if (angle_of_block_robot[i] < my_angle)
				my_rank_in_angle++;
		Block_get_target();
		VecPosition special_target = Block_get_special_target(my_rank_in_angle, num_of_block_robot + 1, general_target, world.ball.getCurrentBallPosition().getY() > 0 ? 1 : -1);
		angle = (world.ball.getCurrentBallPosition() - special_target).Angle();
		dest = special_target;

		/*decider.setMeduimLevel("GoInPathAndStop");
		if (special_target.toString() == "X:nan Y:nan")
		{
		int x = 9;
		}
		((GoInPathAndStop*)(decider.mediumlevels[decider.currentMediumLevel]))->SetParam(special_target, (world.ball.getCurrentBallPosition() - special_target).Angle(), config->getRobotsMaxSpeed(), 0);

		((GoInPathAndStop*)(decider.mediumlevels[decider.currentMediumLevel]))->decide(world);*/

		several_position_Line(dest, NumofRobot, index_robotT);///

	}
	//DrawShape::DrawCircle(Circle(dest, 85));
}
//
void HighLevel::DefenceCutShoot(int NumofRobot, int index_robotT[])
{
	//////////////?????
	VecPosition dest;
	double angle = 0;
	dest = Field::getGoalMidP();
	angle = (VecPosition(0, 0) - Field::getGoalMidP()).Angle();
	////////????????????
	Circle robots[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	int num_of_robots = 0;
	int kk = 1;

	Cone::Hole_Type bigest_hole;

	VecPosition mid_bigest_holl;
	Cone ball_to_goal((world.ball.getCurrentBallPosition() /* +(world.ball.getVelocity() / kk)*/), Field::getUpBarP(), Field::getDownBarP());
	int index_goali = 0;
	for (int d = 0; d < NumofRobot; d++)
		if (world.robotT[index_robotT[d]].id == world.team_T.Goalie)
			index_goali = d;//Estimation::get_nearest_T(world, Field::getGoalMidP());

	for (int i = 0; i < NumofRobot; i++)
		//if (index_goali != i)
		robots[num_of_robots++] = Circle(world.robotT[index_robotT[i]].position, 90);


	if (ball_to_goal.Get_Free_Space_In_Cone(robots, num_of_robots, bigest_hole) > 0)
	{
		mid_bigest_holl.setX((bigest_hole.Point_1.getX() + bigest_hole.Point_2.getX()) / 2);

		mid_bigest_holl.setY((bigest_hole.Point_1.getY() + bigest_hole.Point_2.getY()) / 2);

		//	KESHIDANE KHAT AZ TOOP BE VASAT BOZORGTARIN HOL
		double R_inter = 90;
		int decider_id = 0;
		if (world.robotT[decider_id].position.getDistanceTo(world.ball.getCurrentBallPosition()) < mid_bigest_holl.getDistanceTo(world.robotT[decider_id].position))
			R_inter = world.robotT[decider_id].position.getDistanceTo(world.ball.getCurrentBallPosition());
		else
			R_inter = mid_bigest_holl.getDistanceTo(world.robotT[decider_id].position);
		robots[num_of_robots] = Circle(world.robotT[decider_id].position, R_inter);

		//cout << "world.robotT[world.decider_id].position" << world.robotT[decider_id].position << endl;
		//cout << "R_inter:    " << mid_bigest_holl.getDistanceTo(world.robotT[decider_id].position) << endl << world.robotT[decider_id].position.getDistanceTo(world.ball.getCurrentBallPosition()) << endl;
		//cout << "R_inter=" << R_inter << endl;
		Line mid_bigest_hol_to_ball = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition() /*+ (world.ball.getVelocity())*/, mid_bigest_holl);
		Estimation::ballLine = mid_bigest_hol_to_ball;
		VecPosition intersection;
		VecPosition	temp_direction = (0, 0);// world.ball.getVelocity();
											//Line perpendicular = Line::makeLineFromPositionAndAngle(world.robotT[index_goali].position, Rad2Deg(temp_direction.rotate(M_PI / 2.0).Angle()));
		VecPosition point1, point2, point3;
		mid_bigest_hol_to_ball.getCircleIntersectionPoints(robots[num_of_robots], &point1, &point2);
		point3.setX((point1.getX() + point2.getX()) / 2);
		point3.setY((point1.getY() + point2.getY()) / 2);

		//cout << "point1=" << point1 << endl << "point2" << point2 << endl << "point3" << point3 << endl;
		//cout << "world.ball.getCurrentBallPosition()" << world.ball.getCurrentBallPosition();
		Line amood = Line::makeLineFromTwoPoints(point3, world.robotT[decider_id].position);
		intersection = point3;

		if (intersection.getX() < world.ball.getCurrentBallPosition().getX())
		{
			double shoaa;
			shoaa = intersection.getDistanceTo(world.ball.getCurrentBallPosition());
			Circle temp;
			temp = Circle(world.ball.getCurrentBallPosition(), shoaa);
			mid_bigest_hol_to_ball.getCircleIntersectionPoints(temp, &point1, &point2);
			if (point1.getX() > world.ball.getCurrentBallPosition().getX()) {
				dest = point1;
			}
			else
				dest = point2;

		}

		else
			dest = intersection;



		angle = (world.ball.getCurrentBallPosition() - world.robotT[decider_id].position).Angle();

		//if (ball_to_goal.Get_Free_Space_In_Cone(robots, num_of_robots, bigest_hole) > 0)
		//{
		//	mid_bigest_holl.setX((bigest_hole.Point_1.getX() + bigest_hole.Point_2.getX()) / 2);
		//	mid_bigest_holl.setY((bigest_hole.Point_1.getY() + bigest_hole.Point_2.getY()) / 2);
		//	//	KESHIDANE KHAT AZ TOOP BE VASAT BOZORGTARIN HOL

		//	Line mid_bigest_hol_to_ball = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(), mid_bigest_holl);
		//	Estimation::ballLine = mid_bigest_hol_to_ball;
		//	VecPosition intersection;
		//	VecPosition	temp_direction = world.ball.getVelocity();
		//	Line perpendicular = Line::makeLineFromPositionAndAngle(world.robotT[num_of_robots].position, Rad2Deg(temp_direction.rotate(M_PI / 2.0).Angle()));
		//	intersection = mid_bigest_hol_to_ball.getIntersection(perpendicular);

		//	if (intersection.getX() > Field::getGoalMidP().getX())
		//				dest = Estimation::ballLine.getIntersection(Field::getGoalLineP());
		//			else if (Field::isInField(intersection))
		//				dest = intersection;

		//			else
		//	dest = intersection;        /// + (intersection - world.robotT[decider.index].position).normalize() * _tm ;*/
		//	angle = (Field::getGoalMidP()).Angle();
		//
		//}




		//z	world.angle = angle;


		several_position_Line(dest, NumofRobot, index_robotT);///




	}
}

void HighLevel::GoaliHoleCover()
{
	static Cone::Hole_Type bigest_hole;
	VecPosition target;
	Circle robots[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	int num_of_robots = 0;
	int kk = 1;

	//Cone::Hole_Type bigest_hole;

	int index_goali = Estimation::get_nearest_T(world, Field::getGoalMidP());
	for (int i = 0; i < world.numT; i++)
		if (index_goali != i)
			robots[num_of_robots++] = Circle(world.robotT[i].position, 90);

	Cone ball_to_goal((world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk)), Field::getUpBarP(), Field::getDownBarP());


	/////////////////////////////////////////
	//Circle c((Field::getUpBarP() + Field::getDownBarP()/2), 80);
	//GLFrame::addCirlceToPainting(c,true);
	//GLFrame::addLineToPainting(world.ball.getCurrentBallPosition(), Field::getUpBarP());
	//GLFrame::addLineToPainting(world.ball.getCurrentBallPosition(), Field::getDownBarP());

	////////////////////////////////////////
	if (ball_to_goal.Get_Free_Space_In_Cone(robots, num_of_robots, bigest_hole) > 5 && (world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk)).getX() < Field::getGoalMidP().getX())
	{
		//Support::bigest_hole = bigest_hole;
		bigest_hole = bigest_hole;
		/*! creating goal shape */
		//before : Goal_Shape goal_shape( Estimation::getMarkOfMathematicalEquation( 1000, 5000, 100, 500, Field::getGoalMidP().getDistanceTo(world.ball.getCurrentBallPosition()) ) );
		Goal_Shape goal_shape(Estimation::getMarkOfMathematicalEquation(1.25*PenaltyAreaRadius, 6.25*PenaltyAreaRadius, 0.125*PenaltyAreaRadius, 0.625*PenaltyAreaRadius, Field::getGoalMidP().getDistanceTo((world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk)))));

		/*! a line between ball position and target ---> "ball_to_target" */

		VecPosition _temp_ball_to_up_of_hole = bigest_hole.Point_1 - (world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk));
		VecPosition _temp_ball_to_down_of_hole = bigest_hole.Point_2 - (world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk));
		double _temp_angle = VecPosition::AngleBetweenWithSgn(_temp_ball_to_up_of_hole, _temp_ball_to_down_of_hole);
		_temp_ball_to_up_of_hole.rotate(_temp_angle / 2.0);

		Line ball_to_target = Line::makeLineFromPositionAndAngle((world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk)), Rad2Deg(_temp_ball_to_up_of_hole.Angle()));




		/*! detecting intersection between "ball_to_target" and goal shape ---> "target" */

		target = goal_shape.getIntersection(ball_to_target);

		/*! this is for not letting robot to go out of the field in our goal line */
		double _temp_distance;
		_temp_distance = Field::getGoalMidP().getDistanceTo(target);
		double distance_that_robot_will_go_out_of_field = Field::getGoalMidP().getX() - target.getX();
		if (distance_that_robot_will_go_out_of_field < 90)
		{
			distance_that_robot_will_go_out_of_field = 90 - distance_that_robot_will_go_out_of_field;
			if (target.getY() > 0)
			{
				double _asin = distance_that_robot_will_go_out_of_field / _temp_distance;
				if (_asin <= 1 && _asin >= -1)
					_asin = asin(_asin);
				else
					_asin = 0;
				VecPosition temp = VecPosition(_temp_distance, Rad2Deg((target - Field::getGoalMidP()).Angle() + _asin), POLAR);
				/*if (temp.ToString() == "X:nan Y:nan")
				{
				int x = 9 / 0;
				}*/
				target = Field::getGoalMidP() + temp;

			}
			else
			{
				double _asin = distance_that_robot_will_go_out_of_field / _temp_distance;
				if (_asin <= 1 && _asin >= -1)
					_asin = asin(_asin);
				else
					_asin = 0;

				VecPosition temp = VecPosition(_temp_distance, Rad2Deg((target - Field::getGoalMidP()).Angle() - _asin), POLAR);

				/*if (temp.ToString() == "X:nan Y:nan")
				{
				int x = 9 / 0;
				}*/
				target = Field::getGoalMidP() + temp;

			}
		}
		/*! this is for not letting robot to go out of the field in our goal line */
		//z		World::angle = (world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk) - world.robotT[world.decider_id].position).Angle();
	}
	else
	{
		//before: Goal_Shape goal_shape( Estimation::getMarkOfMathematicalEquation( 1000, 5000, 100, 500, Field::getGoalMidP().getDistanceTo(world.ball.getCurrentBallPosition()) ) );
		Goal_Shape goal_shape(Estimation::getMarkOfMathematicalEquation(1.25*PenaltyAreaRadius, 6.25*PenaltyAreaRadius, 0.125*PenaltyAreaRadius, 0.625*PenaltyAreaRadius, Field::getGoalMidP().getDistanceTo((world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk)))));
		target = goal_shape.getIntersection(Line::makeLineFromTwoPoints(VecPosition(0, 0), Field::getGoalMidP()));

		//target = Field::getGoalMidP(); //+ VecPosition(0, 100);
		//z	World::angle = M_PI;
	}
	/*decider.setMeduimLevel("MNone");
	decider.setLowLevel("GoTowardAndStop");
	((GoTowardAndStop*)(decider.lowlevels[decider.currentLowLevel]))->SetParam(target, angle, config->getRobotsMaxSpeed());

	((MNone*)(decider.mediumlevels[decider.currentMediumLevel]))->decide(world);*/
	world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = target;
	world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;

}



//////do not use?
void  HighLevel::GoaliCutShoot(World &world, int id)
{
	//world.decider_id = 1;
	//	int decider_id = 0;
	VecPosition dest;
	double angle;
	angle = M_PI;
	string  condition = "";


	///khodam shoro kardam b neveshtan:
	Circle robots[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	int num_of_robots = 0;
	int kk = 1;

	Cone::Hole_Type bigest_hole;

	VecPosition mid_bigest_holl;
	Cone ball_to_goal((world.ball.getCurrentBallPosition() /* +(world.ball.getVelocity() / kk)*/), Field::getUpBarP(), Field::getDownBarP());
	int index_goali = 0;
	for (int d = 0; d < world.numT; d++)
		if (world.robotT[d].id == world.team_T.Goalie)
			index_goali = d;//Estimation::get_nearest_T(world, Field::getGoalMidP());

	//cout << "index_goali=" << index_goali << endl;
	for (int i = 0; i < world.numT; i++)
		if (index_goali != i)
			robots[num_of_robots++] = Circle(world.robotT[i].position, 90);


	if (ball_to_goal.Get_Free_Space_In_Cone(robots, num_of_robots, bigest_hole) > 0)
	{
		mid_bigest_holl.setX((bigest_hole.Point_1.getX() + bigest_hole.Point_2.getX()) / 2);

		mid_bigest_holl.setY((bigest_hole.Point_1.getY() + bigest_hole.Point_2.getY()) / 2);

		//cout << "mid_bigest_holl=" << mid_bigest_holl << endl;
		//	KESHIDANE KHAT AZ TOOP BE VASAT BOZORGTARIN HOL
		double R_inter = 90;
		if (world.robotT[index_goali].position.getDistanceTo(world.ball.getCurrentBallPosition()) <  mid_bigest_holl.getDistanceTo(world.robotT[index_goali].position))
			R_inter = world.robotT[index_goali].position.getDistanceTo(world.ball.getCurrentBallPosition());
		else
			R_inter = mid_bigest_holl.getDistanceTo(world.robotT[index_goali].position);
		robots[num_of_robots] = Circle(world.robotT[index_goali].position, R_inter);

		//cout << "world.robotT[index_goali].position" << world.robotT[index_goali].position << endl;
		//cout << "R_inter:    " << mid_bigest_holl.getDistanceTo(world.robotT[index_goali].position) << endl << world.robotT[index_goali].position.getDistanceTo(world.ball.getCurrentBallPosition()) << endl;
		//cout << "R_inter=" << R_inter << endl;
		Line mid_bigest_hol_to_ball = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition() /*+ (world.ball.getVelocity())*/, mid_bigest_holl);
		Estimation::ballLine = mid_bigest_hol_to_ball;
		VecPosition intersection;
		VecPosition	temp_direction = (0, 0);// world.ball.getVelocity();
											//Line perpendicular = Line::makeLineFromPositionAndAngle(world.robotT[index_goali].position, Rad2Deg(temp_direction.rotate(M_PI / 2.0).Angle()));
		VecPosition point1, point2, point3;
		mid_bigest_hol_to_ball.getCircleIntersectionPoints(robots[num_of_robots], &point1, &point2);
		point3.setX((point1.getX() + point2.getX()) / 2);
		point3.setY((point1.getY() + point2.getY()) / 2);

		//cout << "point1=" << point1 << endl << "point2" << point2 << endl << "point3" << point3 << endl;
		//cout << "world.ball.getCurrentBallPosition()" << world.ball.getCurrentBallPosition();
		Line amood = Line::makeLineFromTwoPoints(point3, world.robotT[index_goali].position);
		intersection = point3;
		//intersection = amood.getIntersection(mid_bigest_hol_to_ball);

		//intersection = mid_bigest_hol_to_ball.getIntersection(perpendicular);

		//Goal_Shape goal_shape(PenaltyAreaRadius);//it was 600 for single size
		Goal_Shape goal_shape(Estimation::getMarkOfMathematicalEquation(1.25*PenaltyAreaRadius, 6.25*PenaltyAreaRadius, 0.125*PenaltyAreaRadius, 0.625*PenaltyAreaRadius, Field::getGoalMidP().getDistanceTo((world.ball.getCurrentBallPosition()))));
		if (goal_shape.isInside(intersection))
		{
			dest = intersection;        /// + (intersection - world.robotT[decider.index].position).normalize() * _tm ;
			angle = (-Field::getGoalMidP()).Angle();
		}

		/*dest = intersection;        /// + (intersection - world.robotT[decider.index].position).normalize() * _tm ;
		angle = (-Field::getGoalMidP()).Angle();*/
		else
		{
			//dest = Estimation::ballLine.getIntersection(Field::getGoalLineP());
			////dest= Estimation::ballLine.getIntersection()
			angle = (world.ball.getCurrentBallPosition() - dest).Angle();
			//	front_line = Line::makeLineFromPositionAndAngle(VecPosition((FieldLength / 2), 0), 90);

			//bool 	output_1 = false, output_2 = false, output_3 = false;
			if (world.robotT[index_goali].position.getDistanceTo(VecPosition((FieldLength / 2), (FreeBound / 2))) < PenaltyAreaRadius)
			{
				Line amood = Line::makeLineFromTwoPoints(point3, world.robotT[index_goali].position);
				Circle s1(VecPosition((FieldLength / 2), (FreeBound / 2)), PenaltyAreaRadius);
				intersection = amood.getCircleIntersectionPoints(s1, &point1, &point2);        /// + (intersection - world.robotT[decider.index].position).normalize() * _tm ;
			}
			if (intersection.getDistanceTo(VecPosition((FieldLength / 2), -(FreeBound / 2)))< PenaltyAreaRadius)
			{
				Line amood = Line::makeLineFromTwoPoints(point3, world.robotT[index_goali].position);
				Circle s1(VecPosition((FieldLength / 2), (FreeBound / 2)), PenaltyAreaRadius);
				intersection = amood.getCircleIntersectionPoints(s1, &point1, &point2);
			}
			if ((((FieldLength / 2) - PenaltyAreaRadius) < intersection.getX() < (FieldLength / 2)) && (-(FreeBound / 2) < intersection.getY() < (FreeBound / 2)))
			{
				Line amood = Line::makeLineFromTwoPoints(point3, world.robotT[index_goali].position);

			}

			/*if (world.ball.getCurrentBallPosition().getY() < -1000) {
			intersection=	mid_bigest_hol_to_ball.getIntersection()

			}*/

		}
	}

	//z world.angle = angle;
	world.robotT[world.getIndexForRobotTNumber(id)].destination_position = dest;
	//DrawShape::DrawCircle(Circle(dest, 85));

}


//
////////////////////////////////end of mohadeseh///////////////////////
//
//
//
//
//
//
/////**************************************************reihane**********************************************///
//
void HighLevel::BlockKicker(int NumofRobot, int index_robotT[])
{
	/*	setParam(4);*/
	///////////////////////////
	enum BlockMode { NormalBlockKicker = 0, KickOffBlockKicker = 1 };

	int indexOppKicker;
	VecPosition LastIntersection[10];
	int numOfLastIntersection = 0;
	BlockMode mode =/* NormalBlockKicker*/KickOffBlockKicker;
	indexOppKicker = -1;
	////////////////////////
	//if (indexOppKicker == -1 || indexOppKicker > 10 || indexOppKicker < 0)
	//{
	//	//logStatus("ERORR: Strategy Must Set Robot To Be Blocked ");
	//	return;
	//}

	VecPosition intersection1, intersection2, dest;

	Circle ball_area(world.ball.getCurrentBallPosition(), 500 + 90);
	int decider_id = 0;
	if (mode == KickOffBlockKicker)
	{
		Cone BallToGoal(world.ball.getCurrentBallPosition(), Field::getUpBarP(), Field::getDownBarP());
		Cone::Hole_Type LongestHole;
		BallToGoal.Get_Free_Space_In_Cone(LongestHole, 1, 0, decider_id/*decider.index*/);
		if (LongestHole.Point_1.getDistanceTo(LongestHole.Point_2) > 5)
		{
			dest = (LongestHole.Point_1 + LongestHole.Point_2) * 0.5f;


			//if (NumOfRobot > 1)
			//{
			//	for (int a = 1; a <= numOfRobot; a++)
			//	{
			//		if (numOfRobot % 2 == 1)
			//		{
			//			dest = dest - (0, (numOfRobot / 2) * 2 * 600);
			//			world.finalyDest[a - 1] = dest + (0, (a - 1) * 2 * 600);
			//		}
			//		else
			//		{
			//			dest = dest - (0, (numOfRobot - 1) * 600);
			//			world.finalyDest[a - 1] = dest + (0, (a - 1) * 2 * 600);
			//		}
			//	}
			//}
			//else
			//	world.finalyDest[0] = dest;

			several_position_Line(dest, NumofRobot, index_robotT);///
		}

		else
			dest = Field::getGoalMidP();
		Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(), dest).getCircleIntersectionPoints(ball_area, &intersection1, &intersection2);
		dest = Field::getGoalMidP().getNearer(intersection1, intersection2);
		//if (numOfRobot > 1)
		//{
		//	for (int b = 1; b <= numOfRobot; b++)
		//	{
		//		if (numOfRobot % 2 == 1)
		//		{
		//			dest = dest - (0, (numOfRobot / 2) * 2 * 600);
		//			world.finalyDest[b - 1] = dest + (0, (b - 1) * 2 * 600);
		//		}
		//		else
		//		{
		//			dest = dest - (0, (numOfRobot - 1) * 600);
		//			world.finalyDest[b - 1] = dest + (0, (b - 1) * 2 * 600);
		//		}
		//	}
		//}
		//else
		//	world.finalyDest[0] = dest;
		several_position_Line(dest, NumofRobot, index_robotT);///
	}
	else if (NormalBlockKicker == mode)
	{
		indexOppKicker = 7;
		Line opponent_to_ball = Line::makeLineFromPositionAndAngle(world.ball.getCurrentBallPosition(), Rad2Deg(world.robotO[indexOppKicker].angle));

		opponent_to_ball.getCircleIntersectionPoints(ball_area, &intersection1, &intersection2);
		if ((intersection1 - world.ball.getCurrentBallPosition()).AngleBetween(VecPosition::directVector(world.robotO[indexOppKicker].angle)) < Deg2Rad(20))
			dest = intersection1;
		else
			dest = intersection2;

		if (VecPosition::directVector(world.robotO[indexOppKicker].angle).AngleBetween(Field::getGoalMidP() - world.ball.getCurrentBallPosition()) > M_PI / 2)
		{
			Line tangentLine = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(), Field::getGoalMidP()).getTangentLine(world.ball.getCurrentBallPosition());
			tangentLine.getCircleIntersectionPoints(ball_area, &intersection1, &intersection2);
			if (dest.getDistanceTo(intersection1) < dest.getDistanceTo(intersection2))
				dest = intersection1;
			else
				dest = intersection2;

			//if (numOfRobot > 1)
			//{
			//	VecPosition p1(Field::getUpBarP().getX() - 85, Field::getUpBarP().getY());
			//	VecPosition p2(Field::getDownBarP().getX() - 85, Field::getDownBarP().getY());
			//	Line GoalLine = Line::makeLineFromTwoPoints(p1, p2);
			//	Line L = Line::makeLineFromTwoPoints(dest, (p1 + p2) / 2);
			//	VecPosition intersect = L.getIntersection(GoalLine);
			//	for (int c = 1; c <= numOfRobot; c++)
			//	{
			//		Line verticalLine = Line::makeLineFromPositionAndAngle(dest, Rad2Deg(dest.Angle()));
			//		double A = 0, B = 0, C = 0;
			//		A = verticalLine.getACoefficient();
			//		B = verticalLine.getBCoefficient();
			//		C = verticalLine.getCCoefficient();
			//		if (numOfRobot % 2 == 1)
			//		{
			//			if (c == 1) {
			//				dest.setY(dest.getY() - ((numOfRobot / 2) * 2 * 600));
			//			}
			//			dest.setY(dest.getY() + (c - 1) * 2 * 600);
			//			dest.setX(-(A / B)*dest.getY() + (-(C / A)));
			//			world.finalyDest[c - 1] = dest;
			//		}
			//		else
			//		{
			//			if (c == 1) {
			//				dest.setY(dest.getY() - ((numOfRobot - 1) * 600));
			//			}
			//			dest.setY(dest.getY() + (c - 1) * 2 * 600);
			//			dest.setX(dest.getY() * (-(A / B)) + (-(C / A)));
			//			world.finalyDest[c - 1] = dest;
			//		}
			//	}
			//}
			//else
			//	world.finalyDest[0] = dest;

			several_position_Line(dest, NumofRobot, index_robotT);///

		///	GLFrame::addCirlceToPainting(Circle(intersection1, 50));
		///	GLFrame::addCirlceToPainting(Circle(intersection2, 50));
		}

		if (!Field::getFieldRectangle().isInside(dest))
		{
			VecPosition rect_Intersect[5];
			int numOfRectIntersect = 0;

			numOfRectIntersect += Line::makeLineFromTwoPoints(Field::getUpLeftPoint() + VecPosition(1, -1) * 90, Field::getUpRightPoint() + VecPosition(-1, -1) * 90).getCircleIntersectionPoints(ball_area, &rect_Intersect[numOfRectIntersect], &rect_Intersect[numOfRectIntersect + 1]);
			numOfRectIntersect += Line::makeLineFromTwoPoints(Field::getUpLeftPoint() + VecPosition(1, -1) * 90, Field::getDownLeftPoint() + VecPosition(1, 1) * 90).getCircleIntersectionPoints(ball_area, &rect_Intersect[numOfRectIntersect], &rect_Intersect[numOfRectIntersect + 1]);
			numOfRectIntersect += Line::makeLineFromTwoPoints(Field::getDownRightPoint() + VecPosition(-1, 1) * 90, Field::getUpRightPoint() + VecPosition(-1, -1) * 90).getCircleIntersectionPoints(ball_area, &rect_Intersect[numOfRectIntersect], &rect_Intersect[numOfRectIntersect + 1]);
			numOfRectIntersect += Line::makeLineFromTwoPoints(Field::getDownRightPoint() + VecPosition(-1, 1) * 90, Field::getDownLeftPoint() + VecPosition(1, 1) * 90).getCircleIntersectionPoints(ball_area, &rect_Intersect[numOfRectIntersect], &rect_Intersect[numOfRectIntersect + 1]);

			for (int i = 0; i < numOfRectIntersect; i++)
			{
				if (!Field::getFieldRectangle().isInside(rect_Intersect[i]))
				{
					rect_Intersect[i] = rect_Intersect[numOfRectIntersect - 1];
					numOfRectIntersect--;

				}
			}

			for (int i = 0; i < numOfRectIntersect; i++)
///				GLFrame::addCirlceToPainting(Circle(rect_Intersect[i], 30), 255.0f/* QColor("blue")*/);

			if (numOfRectIntersect > 0)
			{
				for (int i = 1; i < numOfRectIntersect; i++)
				{
					rect_Intersect[0] = dest.getNearer(rect_Intersect[0], rect_Intersect[i]);
				}
				dest = rect_Intersect[0];
				/*for (int m = 1; m <= numOfRobot;m++)
				{
				if (numOfRectIntersect > 0)
				{
				for (int i = 1; i < numOfRectIntersect; i++)
				{
				rect_Intersect[0] = dest.getNearer(rect_Intersect[0], rect_Intersect[i]);
				}
				dest = rect_Intersect[0];
				rect_Intersect[0].isNan;
				}
				world.finalyDest[m - 1] = dest;
				}*/

				/*	if (numOfRobot > 1)
				{
				for (int b = 1;b <= numOfRobot;b++)
				{
				if (numOfRobot % 2 == 1)
				{
				}
				else
				{
				}
				}
				}
				else
				world.finalyDest[0] = dest;*/


			}
		}

	}
	else
	{
	} //int c = /* 1 / 0*/;

	  /*decider.setMeduimLevel("MNone");
	  decider.setLowLevel("GoOnCircle");
	  ((GoOnCircle*)(decider.lowlevels[decider.currentLowLevel]))->SetParam(world.ball.getCurrentBallPosition(), 500 + 90, (final_dest - world.ball.getCurrentBallPosition()).Angle(), config->getRobotsMaxSpeed());
	  ((MNone*)(decider.mediumlevels[decider.currentMediumLevel]))->decide(world);*/

	  //z	World::radius = 500 + 90;
	  //z	World::destAngle = (dest - world.ball.getCurrentBallPosition()).Angle();
	  //z World::speed = 0; /*getRobotsMaxSpeed();*/
	  //z	World::direction = 0;
	//DrawShape::DrawCircle(Circle(dest, 85));
	several_position_Line(dest, NumofRobot, index_robotT);///
}
//
void HighLevel::Banish(int index_robotT)
{
	Line ball_to_goal = Line::makeLineFromTwoPoints(Field::getGoalMidP(), world.ball.getCurrentBallPosition());
	VecPosition robot_to_ballVec = world.ball.getCurrentBallPosition() - world.robotT[index_robotT].position;
	if ((world.robotT[index_robotT].position.getY() - world.ball.getCurrentBallPosition().getY()) <= 0)
		world.robotT[index_robotT].destination_angle = (robot_to_ballVec).AngleBetween(VecPosition(1, 0));
	else
		world.robotT[index_robotT].destination_angle = -((robot_to_ballVec).AngleBetween(VecPosition(1, 0)));
	VecPosition dest = world.ball.getCurrentBallPosition();
	int decider_id = 0;
	AngRad angle = (world.ball.getCurrentBallPosition() - world.robotT[HighLevel::nearest_robot_to_ball('T')/*decider.index*/].position).Angle();

	if (angle <= M_PI / 2 && angle > 0)
	{
		angle = M_PI / 2;
		dest += VecPosition(0, -ROBOT_RADIUS);
	}
	else if (angle >= -M_PI / 2 && angle <= 0)
	{
		angle = -M_PI / 2;
		dest += VecPosition(0, ROBOT_RADIUS);
	}

	int chip;
	Paraline ball_to_dest = Paraline(world.ball.getCurrentBallPosition(), dest);
	VecPosition field(FieldLength, FieldWidth);
	double min_distance_to_nearer_opponent = field.getMagnitude();
	for (int i = 0; i < world.numO; i++)
	{
		if (ball_to_dest.getDistanceTo(world.robotO[i].position) < min_distance_to_nearer_opponent)
		{
			min_distance_to_nearer_opponent = ball_to_dest.getDistanceTo(world.robotO[i].position);
		}
	}
	/*if (min_distance_to_nearer_opponent < 1000)
	chip = 1;
	else
	chip = 1;*/

	if (world.robotT[/*decider.index*/decider_id].position.getDistanceTo(world.ball.getCurrentBallPosition()) < 500)
	{
		//if (chip == 0)
		//{
		//	//((GoToward*)(decider.basiclevels["GoToward"]))->setDirectkick(7);
		//	world.decider_kickPower = 7/*power*/;
		//	world.decider_chipKick = 0;
		//	world.decider_directKick = 1;
		//}
		if (min_distance_to_nearer_opponent < 1000)
		{
			/*if (chip == 1)
			{*/
			//((GoToward*)(decider.basiclevels["GoToward"]))->setChipkick(7);
			//z		world.decider_kickPower = 7/*power*/;
			//z		world.decider_chipKick = 1;
			//z		world.decider_directKick = 0;
		}
		//}
	}
	else
	{
		//((GoToward*)(decider.basiclevels["GoToward"]))->setCancelKick();
		//z	world.decider_kickPower = 0;
		//z	world.decider_chipKick = 0;
		//z		world.decider_directKick = 0;
		//z	world.decider_spinPower = 0;
	}

	/*decider.setMeduimLevel("MNone");
	decider.setLowLevel("GoInPath");
	((GoInPath*)(decider.lowlevels[decider.currentLowLevel]))->SetParam(dest, angle, 2000, 0);
	((GoInPath*)(decider.lowlevels[decider.currentLowLevel]))->decide(world);*/
	//z	World::angle = angle;
	//z	World::speed = 2000;
	//z	World::isBallObstacle;
	/////Circle* ??????????????????
	/////int ??????????????????????

	world.robotT[index_robotT].destination_position = dest;
}


void HighLevel::Shoot(int index_robotT)
{
	index_pass_senderT = index_robotT;
	////////////////////
	VecPosition goal_dest;
	int power;
	bool chip;
	bool is_in_hurry;
	bool _is_in_hurry = true;
	////////////////////
	double angle;
	angle = M_PI;
	Cone::Hole_Type Longest_Hole;
	Circle robots[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	int num_of_robots = 0;
	int kk = 1;
	VecPosition mid_bigest_holl;
	Cone BallToGoal(world.ball.getCurrentBallPosition(), Field::getUpBarO(), Field::getDownBarO());
	int index_goali = 0;
	for (int d = 0; d < world.numO; d++)
		if (world.robotO[d].id == world.team_O.Goalie)
			index_goali = d;//Estimation::get_nearest_T(world, Field::getGoalMidP());

							//cout << "index_goali=" << index_goali << endl;
	for (int i = 0; i < world.numO; i++)
		//if (index_goali != i)
		robots[num_of_robots++] = Circle(world.robotO[i].position, ROBOT_RADIUS);

	VecPosition dest;
	//world.robotT[id].destination_angle = angle;


	double angle_robot_to_mid_bigest_holl = 0;



	if (BallToGoal.Get_Free_Space_In_Cone(robots, num_of_robots, Longest_Hole) > 0)
	{
		mid_bigest_holl.setX((Longest_Hole.Point_1.getX() + Longest_Hole.Point_2.getX()) / 2);

		mid_bigest_holl.setY((Longest_Hole.Point_1.getY() + Longest_Hole.Point_2.getY()) / 2);
		VecPosition robot_to_mid_bigest_hollVec = mid_bigest_holl - world.robotT[index_robotT].position;
		//DrawShape::DrawDot(mid_bigest_holl, 100, 255, 0, 0);
		//	DrawShape::DrawParaline(mid_bigest_holl, world.robotT[index_robotT].position);
		 angle_robot_to_mid_bigest_holl = -((robot_to_mid_bigest_hollVec).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index_robotT].position.getY() - mid_bigest_holl.getY());
		//cout << "mid_bigest_holl.angle=" << mid_bigest_holl.Angle(true) << endl;
		Line mid_bigest_hol_to_ball = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition() /*+ (world.ball.getVelocity())*/, mid_bigest_holl);

		/*world.r[0].setVecPosition(world.ball.getCurrentBallPosition().getX(), world.ball.getCurrentBallPosition().getY());
		world.r[1].setVecPosition(mid_bigest_holl.getX(), mid_bigest_holl.getY());*/
		VecPosition point1, point2;
		Circle circle(world.ball.getCurrentBallPosition(), ROBOT_RADIUS + BALL_RADIUS);
		mid_bigest_hol_to_ball.getCircleIntersectionPoints(circle, &point1, &point2);
		if (mid_bigest_holl.getDistanceTo(point1) > mid_bigest_holl.getDistanceTo(point2))
		{
			dest = point1;
		}
		else
			dest = point2;
		if (dest.getDistanceTo(world.robotT[index_robotT/*decider.index*/].position) < REACH_DESTINATION_APPROXIMATION)
		{
			if (abs(world.robotT[index_robotT/*decider.index*/].angle- angle_robot_to_mid_bigest_holl) <  REACH_ANGLE_APPROXIMATION)
			{
				dest = world.ball.getCurrentBallPosition();
				//angle = mid_bigest_holl.Angle(true);
				//z		world.angle = angle;

			}

		}

	}
	world.robotT[index_robotT].destination_angle = angle_robot_to_mid_bigest_holl;
	world.robotT[index_robotT].destination_position = dest;
	//DrawShape::DrawDot(dest,25,0,0,255);
	//Cone robot_to_goal = Cone(world.ball.getCurrentBallPosition(), Field::getUpBarO(), Field::getDownBarO());
	////robot_to_goal.show(o);

	//if (robot_to_goal.Get_Free_Space_In_Cone(world, Longest_Hole, 1, 0, world.decider_id/* decider.index*/) != 0)
	//{
	//	int o_goali_index = Estimation::get_nearest_O(world, Field::getGoalMidO());
	//	if (world.robotO[o_goali_index].position.isBetween(Field::getUpBarO(), Field::getDownBarO()))
	//	{
	//		if (Longest_Hole.Point_1.getDistanceTo(world.robotO[o_goali_index].position) <
	//			Longest_Hole.Point_2.getDistanceTo(world.robotO[o_goali_index].position))
	//			goal_dest = Longest_Hole.Point_1 + (Longest_Hole.Point_2 - Longest_Hole.Point_1) * 0.7;
	//		else
	//			goal_dest = Longest_Hole.Point_2 + (Longest_Hole.Point_1 - Longest_Hole.Point_2) * 0.7;
	//	}
	//	else
	//		goal_dest = (Longest_Hole.Point_2 + Longest_Hole.Point_1) * 0.5;
	//}
	//else
	//	goal_dest = Field::getGoalMidO();

	VecPosition robot_to_ballVec = world.ball.getCurrentBallPosition() - world.robotT[index_robotT].position;
	double angle_robot_to_ball = -((robot_to_ballVec).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index_robotT].position.getY() - world.ball.getCurrentBallPosition().getY());

//	GLFrame::addCirlceToPainting(Circle(goal_dest, 30), 255.0f/*QColor("red")*/);

//	int speed = 2000;
//	if (world.getInstance().playMode == mode_State::PlayMode::Wait/*Waiting*/)
//		speed = 500;


	double distance = world.ball.getCurrentBallPosition().getDistanceTo(goal_dest);
	//decider.setMeduimLevel("KickToPoint");
	//if (world.getInstance().kickMode == mode_State::KickMode::IndirectFreeKickT/*IndirectFreeKickT*/)
	//{

	//	//((KickToPoint*)(decider.mediumlevels[decider.currentMediumLevel]))->SetParam(goal_dest, 7, 0, speed, 1, is_in_hurry);

	//	//tree *******
	//	//z	world.power = 7;
	//	//z	world.spin = 0;

	//	//z	world.chip = 1;
	//	world.robotT[index_robotT].shoot_or_chip = 0;
	//	world.robotT[index_robotT].kick_power = 7;
	//	//**********da


	//	//z	world.is_in_hurry = is_in_hurry;
	//	//z	world.speed = speed;

	//}
	//else
	{
		if ((abs(world.robotT[index_robotT/*decider.index*/].angle - angle_robot_to_mid_bigest_holl)<  REACH_ANGLE_APPROXIMATION) && (abs(world.robotT[index_robotT/*decider.index*/].angle - angle_robot_to_ball)<  REACH_ANGLE_APPROXIMATION))
		{
			if (Longest_Hole.Point_1.getDistanceTo(Longest_Hole.Point_2) < 90)
			{
				//((KickToPoint*)(decider.mediumlevels[decider.currentMediumLevel]))->SetParam(goal_dest, 7, 0, speed, 1, is_in_hurry);
				//z	world.power = 7;
				world.robotT[index_robotT].shoot_or_chip = 1;
				//	world.robotT[id].kick_power = 7/((FieldLength/2)/mid_bigest_holl.getDistanceTo(world.ball.getCurrentBallPosition()));
				if (0.70710678*sqrt(((mid_bigest_holl.getDistanceTo(world.ball.getCurrentBallPosition()) / 1000)*9.8) / (1 - (0.43*0.70710678*0.70710678))) >= MAX_BALL_SPEED)
					//	world.robotT[index_robotT].kick_power = MAX_BALL_SPEED / 0.70710678;
					world.robotT[index_robotT].kick_power = 3;
				else
					world.robotT[index_robotT].kick_power = 3;
					//world.robotT[index_robotT].kick_power = 0.8*sqrt(((mid_bigest_holl.getDistanceTo(world.ball.getCurrentBallPosition()) / 1000)*9.8) / (1 - (0.43*0.70710678*0.70710678)));
				//z	world.spin = 0;
				//z	world.speed = speed;
				//z	world.chip = 1;
				//z	world.is_in_hurry = is_in_hurry;
			}
			else
			{
				//((KickToPoint*)(decider.mediumlevels[decider.currentMediumLevel]))->SetParam(goal_dest, config->getKP_direct_shoot(), 0, speed, 0, is_in_hurry);
				//	World::goal = goal_dest;
				//World::power = getKP_direct_shoot();  ???
				//z	world.spin = 0;
				//z	world.speed = speed;
				//z	world.chip = 0;
				world.robotT[index_robotT].shoot_or_chip = 0;
				world.robotT[index_robotT].kick_power = 5;
				//world.robotT[index_robotT].kick_power = MAX_BALL_SPEED;
				//z	world.is_in_hurry = is_in_hurry;
			}
		}
		else
		{
			world.robotT[index_robotT].shoot_or_chip = 0;
			world.robotT[index_robotT].kick_power = 0;
		}
	}
	//((KickToPoint*)(decider.mediumlevels[decider.currentMediumLevel]))->decide(world);

}




//void Pass::decide(World &world)
//{
//	setParam(1, 0);
//	// int speed;// = config->getRobotsMaxSpeed() ;
//	if (world.robotT[world.decider_id/*decider.index*/].position.getDistanceTo(world.ball.getCurrentBallPosition()) >600)
//	{
//		//        if ( World::getInstance().playMode == Waiting )
//		//            speed = 500;
//		//        else
//		speed = 500;
//	}
//	else {
//		speed = 100;
//	}
//	dest = world.robotT[decided_index].position + VecPosition::directVector(world.robotT[decided_index].angle) * robot_radius /*config->getDistanceRobotCentertoPlunger()*/;
//	double distance = world.robotT[decided_index].position.getDistanceTo(world.ball.getCurrentBallPosition());
//	bool is_chip = false;
//	Paraline _temp_paraline = Paraline(world.ball.getCurrentBallPosition(), world.robotT[decided_index].position);
//	for (int i = 0; i < world.numT; i++)
//		if (i != world.decider_id/*decider.index*/ && i != decided_index)
//			if (_temp_paraline.getDistanceTo(world.robotT[i].position) < 150)
//				is_chip = true;
//	for (int i = 0; i < world.numO; i++)
//		if (_temp_paraline.getDistanceTo(world.robotO[i].position) < 200)
//			is_chip = true;
//	is_chip = false;
//	//    logStatus( is_chip ? "yes" : "no" );
//	if (is_chip) {
//		world.kickpower = 7;
//	}
//	else {
//		if (world.ball.getCurrentBallPosition().getDistanceTo(world.robotT[decided_index].position)<2000) {
//			world.kickpower = 3;
//		}
//		else if (world.ball.getCurrentBallPosition().getDistanceTo(world.robotT[decided_index].position)<4000) {
//			world.kickpower = 4;
//		}
//		else
//			world.kickpower = 5;
//	}
//	/*decider.setMeduimLevel("KickToPoint");
//	((KickToPoint*)decider.mediumlevels[decider.currentMediumLevel])->SetParam(dest, world.kickpower, 0, speed, is_chip, time_to_intrupt);
//	((KickToPoint*)decider.mediumlevels[decider.currentMediumLevel])->decide(world);*/
//	World::goal = dest;
//	World::power = world.kickpower;
//	World::spin = 0;
//	World::speed = speed;
//	World::chip = is_chip;
//	World::is_in_hurry = time_to_intrupt;
//	//world.dest = dest;
//}




//***************************************************MohammadHossein Z************************************//


/*find nearst robot to ball*/
int HighLevel::nearest_robot_to_ball(char Team)
{
	if (Team == 'T')
	{
		double robotT_to_ball = world.robotT[0].position.getDistanceTo(world.ball.getCurrentBallPosition());
		int robotT_To_ball_indexd = 0;
		for (int i = 0; i < world.numT - 1; i++)
		{
			if (world.robotT[i + 1].position.getDistanceTo(world.ball.getCurrentBallPosition()) < robotT_to_ball)
			{
				robotT_to_ball = world.robotT[i + 1].position.getDistanceTo(world.ball.getCurrentBallPosition());
				robotT_To_ball_indexd = i + 1;
			}
		}
		return robotT_To_ball_indexd;
	}
	else if (Team == 'O')
	{
		double robotO_to_ball = world.robotO[0].position.getDistanceTo(world.ball.getCurrentBallPosition());
		int robotO_To_ball_indexd = 0;
		for (int i = 0; i < world.numO - 1; i++)
		{
			if (world.robotO[i+1].position.getDistanceTo(world.ball.getCurrentBallPosition()) < robotO_to_ball)
			{
				robotO_to_ball = world.robotO[i+1].position.getDistanceTo(world.ball.getCurrentBallPosition());
				robotO_To_ball_indexd = i+1;
			}
		}
		return robotO_To_ball_indexd;
	}

}


/*char team is 'O' or 'T' and  position nearest robotT or robotO*/
int HighLevel::nearest_robot_to_point(char Team, VecPosition postion)
{
	if (Team == 'T')
	{
		double robotT_to_point;
		int RobotT_Distance_To_Ball_index = 0;
		if (world.robotT[0].destination_set == false)//if havenot position index 0
			robotT_to_point = world.robotT[0].position.getDistanceTo(postion);
		else
			robotT_to_point = FieldLength * 10 + FieldWidth * 5;
		for (int i = 0; i < world.numT-1; i++)
		{
			if (world.robotT[i + 1].destination_set == false)
			{
				if (world.robotT[i + 1].position.getDistanceTo(postion) < robotT_to_point)
				{
					RobotT_Distance_To_Ball_index = i + 1;
					robotT_to_point = world.robotT[i + 1].position.getDistanceTo(postion);
				}
			}
		}
		if (RobotT_Distance_To_Ball_index == 1 && world.robotT[RobotT_Distance_To_Ball_index].destination_set == true)
		{
			return -1;//all robots have destination
		}
		else
		{
			world.robotT[RobotT_Distance_To_Ball_index].destination_set = true;
			return RobotT_Distance_To_Ball_index;
		}
	}
	else if (Team == 'O')
	{
		double robotO_to_point;
		int RobotO_Distance_To_Ball_index = 0;
		robotO_to_point = world.robotO[0].position.getDistanceTo(postion);
		for (int i = 0; i < world.numO - 1; i++)
		{
			if (world.robotO[i + 1].position.getDistanceTo(postion) < robotO_to_point)
			{
				RobotO_Distance_To_Ball_index = i + 1;
				robotO_to_point = world.robotO[i + 1].position.getDistanceTo(postion);
			}
		}
		return RobotO_Distance_To_Ball_index;
	}

}


/*start game desined robot destination*/
void HighLevel::start_robotT_format_NoKickMode(string playmode)
{

	/*set first robot angle*/
	for (int i = 0; i <= world.numT; i++)
		world.robotT[i].destination_angle = M_PI;


	//x is robott  kicker robot
	/*if kick of for teamate x=1 else x=0*/
	int x;
	if (playmode == "DirectFreeKickT" || playmode == "IndirectFreeKickT" || playmode == "KickOffT" || playmode == "KickOffTPrepare")
		x = 1;
	else
		x = 0;


	/*set first robot format*/
	switch (world.numT - x)
	{
	case 8:
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, 0))].destination_position = VecPosition(FieldLength / 4, 0);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, FieldWidth / 16))].destination_position = VecPosition(FieldLength / 8, FieldWidth / 16);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, -FieldWidth / 16))].destination_position = VecPosition(FieldLength / 8, -FieldWidth / 16);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, -FieldWidth / 4))].destination_position = VecPosition(FieldLength / 4, -FieldWidth / 4);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, FieldWidth / 4))].destination_position = VecPosition(FieldLength / 4, FieldWidth / 4);
		break;
	case 7:
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, FieldWidth / 16))].destination_position = VecPosition(FieldLength / 8, FieldWidth / 16);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, -FieldWidth / 16))].destination_position = VecPosition(FieldLength / 8, -FieldWidth / 16);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, -FieldWidth / 8))].destination_position = VecPosition(FieldLength / 4, -FieldWidth / 8);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, FieldWidth / 8))].destination_position = VecPosition(FieldLength / 4, FieldWidth / 8);
		break;
	case 6:
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, 0))].destination_position = VecPosition(FieldLength / 8, 0);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, -FieldWidth / 4))].destination_position = VecPosition(FieldLength / 8, -FieldWidth / 4);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, FieldWidth / 4))].destination_position = VecPosition(FieldLength / 8, FieldWidth / 4);
		break;
	case 5:
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, -FieldWidth / 6))].destination_position = VecPosition(FieldLength / 8, -FieldWidth / 6);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, FieldWidth / 6))].destination_position = VecPosition(FieldLength / 8, FieldWidth / 6);
		break;
	case 4:
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, 0))].destination_position = VecPosition(FieldLength / 8, 0);
		break;
	case 3:
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		break;
	case 2:
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, 0))].destination_position = VecPosition(FieldLength / 2.5, 0);
		break;
	case 1:
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
		break;
	case 0:
		break;
	}


}


//      O_O     //
void HighLevel::catch_the_ball_2point_whithspeenback(World &world, int index, VecPosition dest)
{


	//world.robotT[index].ballBalkMode = Ball::BalkMode::notBalk;
	if (HighLevel::find_robot_have_ball('T') == index)
	{
		VecPosition robot_to_dest = dest - world.robotT[index].position;
		world.robotT[index].destination_angle = -((robot_to_dest).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index].position.getY() - dest.getY());
		world.robotT[index].destination_position = dest;
	}
	else
	{
		VecPosition mid_bigest_holl;
		VecPosition robot_to_dest = dest - world.robotT[index].position;
		world.robotT[index].destination_angle = -((robot_to_dest).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index].position.getY() - dest.getY());
		Line mid_bigest_hol_to_ball = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition() /*+ (world.ball.getVelocity())*/, dest);
		//DrawShape::DrawParaline(world.ball.getCurrentBallPosition(), dest, 0, 100, 255);
		VecPosition point1, point2, near_point;
		Circle circle(world.ball.getCurrentBallPosition(), ROBOT_RADIUS + BALL_RADIUS);
		//DrawShape::DrawCircle(circle);
		mid_bigest_hol_to_ball.getCircleIntersectionPoints(circle, &point1, &point2);
		//DrawShape::DrawDot(point1, 30, 255, 255, 255);
		//DrawShape::DrawDot(point2, 30, 0, 0, 0);
		if (dest.getDistanceTo(point1) > dest.getDistanceTo(point2))
			near_point = point1;
		else
			near_point = point2;

		//DrawShape::DrawDot(near_point);


		world.robotT[index].destination_position = near_point;
	}



}


//      O_O     //
void HighLevel::catch_the_ball_2point(World &world, VecPosition dest, int index1, int index2)
{
	VecPosition robot1_to_dest = dest - world.robotT[index1].position;
	VecPosition robot2_to_dest = world.robotT[index2].position - dest;
	//cout << world.robotT[index2].position.getDistanceTo(world.ball.getCurrentBallPosition()) << endl;
	Line mid_bigest_hol_to_ball = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition() /*+ (world.ball.getVelocity())*/, dest);

	if ((world.robotT[index1].position.getDistanceTo(world.ball.getCurrentBallPosition())<ROBOT_RADIUS +BALL_RADIUS+ CORRECTION_FACTOR*10) &&( world.robotT[index2].position.getDistanceTo(world.ball.getCurrentBallPosition())<ROBOT_RADIUS + BALL_RADIUS + CORRECTION_FACTOR * 1000) && (abs(world.robotT[index2].angle - ((-robot2_to_dest).AngleBetween(VecPosition(1, 0)))*sign(dest.getY() - world.robotT[index2].position.getY()))<CORRECTION_FACTOR&&abs(world.robotT[index1].angle - ((-robot1_to_dest).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index1].position.getY() - dest.getY()))<CORRECTION_FACTOR) || (abs(world.robotT[index1].angle + ((robot1_to_dest).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index1].position.getY() - dest.getY()))<CORRECTION_FACTOR&&abs(world.robotT[index2].angle + ((robot2_to_dest).AngleBetween(VecPosition(1, 0)))*sign(dest.getY() - world.robotT[index2].position.getY()))<CORRECTION_FACTOR))
	{
		world.robotT[index1].ballBalkMode = Ball::BalkMode::notBalk;
		world.robotT[index2].ballBalkMode = Ball::BalkMode::notBalk;
		VecPosition point1, point2, near_point,point3,point4;
		Circle circle(world.ball.getCurrentBallPosition(), ROBOT_RADIUS + BALL_RADIUS - CORRECTION_FACTOR * 50);
		//DrawShape::DrawCircle(circle);
		mid_bigest_hol_to_ball.getCircleIntersectionPoints(circle, &point1, &point2);
		//DrawShape::DrawDot(point1, 30, 255, 255, 255);
	//	DrawShape::DrawDot(point2, 30, 0, 0, 0);
		if (dest.getDistanceTo(point1) > dest.getDistanceTo(point2))
		{
			world.robotT[index1].destination_angle = -((robot1_to_dest).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index1].position.getY() - dest.getY());
			world.robotT[index2].destination_angle = -((robot2_to_dest).AngleBetween(VecPosition(1, 0)))*sign(dest.getY() - world.robotT[index2].position.getY());
			near_point = point1;
		}
		else
		{
			world.robotT[index1].destination_angle = ((-robot1_to_dest).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index1].position.getY() - dest.getY());
			world.robotT[index2].destination_angle = ((-robot2_to_dest).AngleBetween(VecPosition(1, 0)))*sign(dest.getY() - world.robotT[index2].position.getY());
			near_point = point2;
		}

		Circle circle1(point1, ROBOT_RADIUS + BALL_RADIUS - CORRECTION_FACTOR * 50);
		Line mid_bigest_hol_to_robot1 = Line::makeLineFromTwoPoints(point1  /*+ (world.ball.getVelocity())*/, dest);
	//	DrawShape::DrawParaline(point1, dest, 0, 100, 255);
		mid_bigest_hol_to_robot1.getCircleIntersectionPoints(circle1, &point3, &point4);
		if (/*point3.getDistanceTo(world.ball.getCurrentBallPosition()) > point4.getDistanceTo(world.ball.getCurrentBallPosition())&&*/point4!= VecPosition (0,0))
		{
			world.robotT[index1].destination_position = point4;
			//DrawShape::DrawDot(point4,30,255,255,255);

		}
		else
		{
			world.robotT[index1].destination_position = point3;
			//DrawShape::DrawDot(point3, 30, 255, 255, 255);

		}


		Circle circle2(point2, ROBOT_RADIUS + BALL_RADIUS - CORRECTION_FACTOR * 50);
		Line mid_bigest_hol_to_robot2 = Line::makeLineFromTwoPoints(point2 /*+ (world.ball.getVelocity())*/, dest);
	//	DrawShape::DrawParaline(point2, dest, 0, 100, 255);
		mid_bigest_hol_to_robot2.getCircleIntersectionPoints(circle2, &point3, &point4);
		if (/*point3.getDistanceTo(world.ball.getCurrentBallPosition()) > point4.getDistanceTo(world.ball.getCurrentBallPosition()) &&*/ point4 != VecPosition(0, 0))
		{
			world.robotT[index2].destination_position = point4;
			//DrawShape::DrawDot(point4, 30, 255, 0, 0);
		}
		else
		{
			world.robotT[index2].destination_position = point3;
			//DrawShape::DrawDot(point3, 30, 255, 0, 0);
		}
	//	world.robotT[index1].destination_position = dest;
	//	world.robotT[index2].destination_position = dest;
	}
	else
	{

		VecPosition mid_bigest_holl;

		//DrawShape::DrawParaline(world.ball.getCurrentBallPosition(), dest, 0, 100, 255);
		VecPosition point1, point2, near_point,point3,point4;
		Circle circle(world.ball.getCurrentBallPosition(), ROBOT_RADIUS + BALL_RADIUS- CORRECTION_FACTOR * 50);
	//	DrawShape::DrawCircle(circle);
		mid_bigest_hol_to_ball.getCircleIntersectionPoints(circle, &point1, &point2);
	//	DrawShape::DrawDot(point1, 30, 255, 255, 255);
	//	DrawShape::DrawDot(point2, 30, 0, 0, 0);
		if (dest.getDistanceTo(point1) > dest.getDistanceTo(point2))
		{
			world.robotT[index1].destination_angle = -((robot1_to_dest).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index1].position.getY() - dest.getY());
			world.robotT[index2].destination_angle = -((robot2_to_dest).AngleBetween(VecPosition(1, 0)))*sign(dest.getY() - world.robotT[index2].position.getY());
			near_point = point1;
		}
		else
		{
			world.robotT[index1].destination_angle = ((-robot1_to_dest).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[index1].position.getY() - dest.getY());
			world.robotT[index2].destination_angle = ((-robot2_to_dest).AngleBetween(VecPosition(1, 0)))*sign(dest.getY() - world.robotT[index2].position.getY());
			near_point = point2;
		}
	//	DrawShape::DrawDot(near_point);


		world.robotT[index1].destination_position = point1;
		world.robotT[index2].destination_position = point2;
	}
	/*VecPosition robot_to_ballVec = world.ball.getCurrentBallPosition() - world.robotT[index1].position;
	if ((world.robotT[index1].position.getY() - world.ball.getCurrentBallPosition().getY()) <= 0)
		world.robotT[index1].destination_angle = (robot_to_ballVec).AngleBetween(VecPosition(1, 0));
	else
		world.robotT[index1].destination_angle = -((robot_to_ballVec).AngleBetween(VecPosition(1, 0)));

	robot_to_ballVec = world.ball.getCurrentBallPosition() - world.robotT[index2].position;
	if ((world.robotT[index2].position.getY() - world.ball.getCurrentBallPosition().getY()) <= 0)
		world.robotT[index2].destination_angle = (robot_to_ballVec).AngleBetween(VecPosition(1, 0));
	else
		world.robotT[index2].destination_angle = -((robot_to_ballVec).AngleBetween(VecPosition(1, 0)));

	world.robotT[index1].destination_position = world.ball.getCurrentBallPosition() + VecPosition(0, BALL_RADIUS + ROBOT_RADIUS);
	world.robotT[index2].destination_position = world.ball.getCurrentBallPosition() + VecPosition(0, -BALL_RADIUS - ROBOT_RADIUS);*/
}



void HighLevel::ready_for_penaltyt(World &world, int index)
{
	VecPosition intersect1, intersect2;
	Line ball_to_goal = Line::makeLineFromTwoPoints(Field::getGoalMidO(), world.ball.getCurrentBallPosition());
	VecPosition robot_to_ballVec = world.ball.getCurrentBallPosition() - world.robotT[index].position;
	if ((world.robotT[index].position.getY() - world.ball.getCurrentBallPosition().getY()) <= 0)
		world.robotT[index].destination_angle = (robot_to_ballVec).AngleBetween(VecPosition(1, 0));
	else
		world.robotT[index].destination_angle = -((robot_to_ballVec).AngleBetween(VecPosition(1, 0)));
	Circle ball_area(world.ball.getCurrentBallPosition(), MERGE_DISTANCE + ROBOT_RADIUS);
	ball_to_goal.getCircleIntersectionPoints(ball_area, &intersect1, &intersect2);
	world.robotT[index].destination_position = Field::getGoalMidO().getFarest(intersect1, intersect2);
}


void HighLevel::start_robotT_format_penaltyt(World &world, int index)
{
	VecPosition robot2goal;
	for (int i = 0; i <= world.numT; i++)
	{
		robot2goal = Field::getGoalMidO() - world.robotT[i].position;
		if ((Field::getGoalMidO().getY() - world.robotT[i].position.getY()) >= 0)
			world.robotT[i].destination_angle = (robot2goal).AngleBetween(VecPosition(1, 0));
		else
			world.robotT[i].destination_angle = -((robot2goal).AngleBetween(VecPosition(1, 0)));
	}
	HighLevel::ready_for_penaltyt(world, index);
	world.robotT[index].destination_set = true;
	HighLevel::CatchPenalty(world, world.team_T.Goalie);
	world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
	//world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(FieldLength / 2.2, 0);
	world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;

	switch (world.numT)
	{
	case 8:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, FieldWidth / 7))].destination_position = VecPosition(FieldLength / 4, FieldWidth / 7);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, -FieldWidth / 7))].destination_position = VecPosition(FieldLength / 4, -FieldWidth / 7);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, FieldWidth / 16);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, -FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, -FieldWidth / 16);
		break;
	case 7:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, 0))].destination_position = VecPosition(FieldLength / 4, 0);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, FieldWidth / 16);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, -FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, -FieldWidth / 16);
		break;
	case 6:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, 0))].destination_position = VecPosition(FieldLength / 8, 0);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, -FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, -FieldWidth / 16);
		break;
	case 5:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, -FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, -FieldWidth / 16);
		break;
	case 4:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		break;
	case 3:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, 0))].destination_position = VecPosition(FieldLength / 2.5, 0);
		break;
	case 2:
		break;
	case 1:
		break;
	case 0:
		break;
	}
}

void HighLevel::start_robotT_format_penaltyo(World &world)
{
	VecPosition robot2goal;
	for (int i = 0; i <= world.numT; i++)
	{
		if (world.getIndexForRobotTNumber(world.team_T.Goalie) != i)
		{
			robot2goal = Field::getGoalMidO() - world.robotT[i].position;
			if ((Field::getGoalMidO().getY() - world.robotT[i].position.getY()) >= 0)
				world.robotT[i].destination_angle = (robot2goal).AngleBetween(VecPosition(1, 0));
			else
				world.robotT[i].destination_angle = -((robot2goal).AngleBetween(VecPosition(1, 0)));
		}
	}
	HighLevel::CatchPenalty(world, world.team_T.Goalie);
	world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set = true;
	switch (world.numT)
	{
	case 8:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, FieldWidth / 7))].destination_position = VecPosition(FieldLength / 4, FieldWidth / 7);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, -FieldWidth / 7))].destination_position = VecPosition(FieldLength / 4, -FieldWidth / 7);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, FieldWidth / 9))].destination_position = VecPosition(FieldLength / 4, FieldWidth / 9);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, -FieldWidth / 9))].destination_position = VecPosition(FieldLength / 4, -FieldWidth / 9);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, -FieldWidth / 9))].destination_position = VecPosition(FieldLength / 4, -FieldWidth / 9);
		break;
	case 7:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4, 0))].destination_position = VecPosition(FieldLength / 4, 0);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, FieldWidth / 16);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, -FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, -FieldWidth / 16);
		break;
	case 6:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 8, 0))].destination_position = VecPosition(FieldLength / 8, 0);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, -FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, -FieldWidth / 16);
		break;
	case 5:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(-FieldLength / 3, -FieldWidth / 16))].destination_position = VecPosition(-FieldLength / 3, -FieldWidth / 16);
		break;
	case 4:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, -FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, -FieldWidth / 12);
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, FieldWidth / 12))].destination_position = VecPosition(FieldLength / 2.5, FieldWidth / 12);
		break;
	case 3:
		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 2.5, 0))].destination_position = VecPosition(FieldLength / 2.5, 0);
		break;
	case 2:
		break;
	case 1:
		break;
	case 0:
		break;
	}
}

//cout danger robotO
void HighLevel::defence_format()
{
	Line roboto2roboto, roboto2goal;
	Circle robot;
	bool space = false;
	VecPosition q1, q2;
	double danger_importance_angle[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double danger_importance_pass[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double danger_importance_goal[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double danger_importance_goal_distance[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double danger_importance_haveball[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double dange_importance_number_robott_block_roboto[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double score_danger[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];

	///////////////danger robot shoot to goal && know whitch robot have ball
	VecPosition robot_front(world.robotO[0].position.getX() + cos(world.robotO[0].angle)*ROBOT_RADIUS, world.robotO[0].position.getY() + sin(world.robotO[0].angle)*ROBOT_RADIUS);
	double distance2ball = robot_front.getDistanceTo(world.ball.getCurrentBallPosition());
	int distance2ball_index = 0;
	int number_robott_block_roboto = 0;
	//VecPosition robot_front;
	//robot_front.setY = world.robotO[distans2ball_index].position.getY();// +sin(world.robotO[distans2ball_index].angle)*ROBOT_RADIUS;
	//robot_front.setX = world.robotO[distans2ball_index].position.getX() + cos(world.robotO[distans2ball_index].angle)*ROBOT_RADIUS;
	for (int i = 0; i < world.numO; i++)
	{
		danger_importance_haveball[i] = 0;
		danger_importance_pass[i] = 0;

		//DrawShape::DrawDot(robot_front);

		number_robott_block_roboto = 0;
		space = true;
		roboto2goal = Line::makeLineFromTwoPoints(world.robotO[i].position, Field::getGoalMidP());
		for (int z = 0; z < world.numT; z++)
		{
			robot = Circle(world.robotT[z].position, ROBOT_RADIUS);
			roboto2goal.getCircleIntersectionPoints(robot, &q1, &q2);
			if (q1 != q2)
			{
				space = false;
				number_robott_block_roboto++;
			}
		}
		if (space)
		{
			danger_importance_goal[i] = 100;
			//DrawShape::DrawLine(world.robotO[i].position, Field::getGoalMidP());
		}
		else
		{
			danger_importance_goal[i] = 0;
		}
		dange_importance_number_robott_block_roboto[i] = number_robott_block_roboto;
	}


	//////////check for realy have ball

	if (HighLevel::find_robot_have_ball('O') != -1)

	{

		danger_importance_haveball[HighLevel::find_robot_have_ball('O')] = 200;
		///////////////danger robot pass
		for (int j = 0; j < world.numO; j++)
		{
			space = true;
			roboto2roboto = Line::makeLineFromTwoPoints(world.robotO[HighLevel::find_robot_have_ball('O')].position, world.robotO[j].position);
			for (int z = 0; z < world.numT; z++)
			{
				robot = Circle(world.robotT[z].position, ROBOT_RADIUS);
				roboto2roboto.getCircleIntersectionPoints(robot, &q1, &q2);
				if (q1 != q2)
				{
					space = false;
				}
			}
			danger_importance_pass[j] = -(world.robotO[HighLevel::find_robot_have_ball('O')].position.getDistanceTo(world.robotO[j].position) / (ROBOT_RADIUS));
			if (space)
			{
				danger_importance_pass[j] += 60;
				//DrawShape::DrawLine(world.robotO[distance2ball_index].position, world.robotO[j].position);
			}
			else
			{
				danger_importance_pass[j] += 0;
			}
		}
	}


	/////////////////score with angle to goal
	double robot_angle2upbar, robot_angle2downbar;
	for (int i = 0; i < world.numO; i++)
	{
		VecPosition robot_to_upbar = Field::getUpBarP() - world.robotO[i].position;
		VecPosition robot_to_downbar = Field::getDownBarP() - world.robotO[i].position;
		robot_angle2upbar = ((world.robotO[i].position.getY() - Field::getUpBarP().getY()) / abs((world.robotO[i].position.getY() - Field::getUpBarP().getY())))*(robot_to_upbar).AngleBetween(VecPosition(1, 0));
		robot_angle2downbar = ((world.robotO[i].position.getY() - Field::getDownBarP().getY()) / abs((world.robotO[i].position.getY() - Field::getDownBarP().getY())))*(robot_to_downbar).AngleBetween(VecPosition(1, 0));
		if ((world.robotO[i].angle > robot_angle2upbar) && (world.robotO[i].angle < robot_angle2downbar))

		{
			danger_importance_angle[i] = 20;
			//DrawShape::DrawDot(world.robotO[i].position);
		}
		else
		{
			danger_importance_angle[i] = 0;
		}
	}

	///////////////score whith spacegoal
	Cone::Hole_Type s;
	//	for  (int i= 0; i < world.numO; i++)
	//{
	Circle a = Circle(world.robotT[world.getIndexForRobotTNumber(1)].position, ROBOT_RADIUS);
	Cone space_goal(world.robotO[world.getIndexForRobotONumber(1)].position, Field::getDownBarP(), Field::getUpBarP());
	//space_goal.Get_Free_Space_In_Cone(&a,1,s);
	//space_goal.Get_Free_Space_In_Cone(world,s,1,1,1,1,1,1);
	//DrawShape::DrawParaline(s.Point_1, s.Point_2);
	//DrawShape::DrawParaline(world.robotO[0].position, s.Point_2);
	//DrawShape::DrawParaline(world.robotO[0].position, s.Point_1);
	//}


	int danger_roboto_index;
	for (int i = 0; i < world.numO; i++)
	{
		//DrawShape::DrawDot(Field::getGoalMidP());
		//;// -dange_importance_number_robott_block_roboto[i];
		if (world.robotO[i].position.getDistanceTo(Field::getGoalMidP()) < (FieldLength / 6))
		{
			danger_importance_goal_distance[i] = 150;
		}
		else if (world.robotO[i].position.getDistanceTo(Field::getGoalMidP()) < (FieldLength / 4))
		{
			danger_importance_goal_distance[i] = 100;
		}
		else if (world.robotO[i].position.getDistanceTo(Field::getGoalMidP()) < (FieldLength / 2))
		{
			danger_importance_goal_distance[i] = 60;
		}
		else
		{
			danger_importance_goal_distance[i] = 0;
		}
		score_danger[i] = danger_importance_angle[i] + danger_importance_pass[i] + danger_importance_goal[i] + danger_importance_haveball[i] + danger_importance_goal_distance[i];
	}
	int max_index=0;
	int danger_index_roboto[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	int max = danger_importance_angle[0] + danger_importance_pass[0] + danger_importance_goal[0] + danger_importance_haveball[0] + danger_importance_goal_distance[0];
	for (int i = 0; i < world.numO; i++)
	{
		for (int j = 0; j < world.numO; j++)
		{
			if (max < score_danger[j])
			{
				max = score_danger[j];
				max_index = j;
			}

		}
		score_danger[max_index] = -1000;
		max = -1000;
		danger_index_roboto[i] = max_index;
		if (danger_index_roboto[i] != danger_robotO[i])
		{
			cout <<"danger ID robotO"<< i << ":" << world.getRobotONumberForIndex(max_index) << endl;
		}
		danger_robotO[i] = max_index;
	}
	//world.robotT[0].destination_position = world.robotO[danger_index_roboto[0]].position;
}


int HighLevel::find_best_robot_pass(int index_robotT)
{
	Line roboto2roboto, robotT1_2_robotT2;
	Circle robot;
	VecPosition q1, q2;
	double danger_importance_distance_to_robott[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double danger_importance_goal_distance[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double danger_importance_goal[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double danger_importance_number_robott_block_roboto[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double distance_roboto_for_chipe[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	int max_index = 0;
	int last_max = 0;
	if (finde_for_pass == 0)
	{
		int number_robott_block_robotT = 0;
		Circle ball = Circle(world.ball.getCurrentBallPosition(), ROBOT_RADIUS);
		Line ball_to_robotT;
		Line ball_to_shoot = Line::makeLineFromTwoPoints(Field::getGoalMidP(), world.ball.getCurrentBallPosition());
		Line robot_2_robot_a1_b1;
		Line robot_2_robot_a2_b2;
		VecPosition q1, q2,q3,q4,q5,q6;
		VecPosition a1, a2, b1, b2;
		VecPosition robot1_robot2;
		VecPosition satnd_robot1_robot2;
		double satnd_robot1_robot2_length;
		bool is_in_block[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];

		for (int i = 0; i < world.numT; i++)
		{
			ball_to_robotT = Line::makeLineFromTwoPoints(world.robotT[i].position, world.ball.getCurrentBallPosition());
			ball_to_robotT.getCircleIntersectionPoints(ball, &q1, &q2);
			if (world.ball.getCurrentBallPosition().getDistanceTo(q1) > world.ball.getCurrentBallPosition().getDistanceTo(q2))
				is_in_block[i]=Balk::isIn_(q1, true, true, true, sender_robotT_pass);
			else
				is_in_block[i] = Balk::isIn_(q2, true, true, true, sender_robotT_pass);
			number_robott_block_robotT = 0;
			robotT1_2_robotT2 = Line::makeLineFromTwoPoints(world.robotT[sender_robotT_pass].position, world.robotT[i].position);
			robot1_robot2 = world.robotT[i].position - world.robotT[sender_robotT_pass].position;
			satnd_robot1_robot2.setX(robot1_robot2.getX() * 0 + robot1_robot2.getY() * 1);/////stand matris is [ 0,1]
			satnd_robot1_robot2.setY(robot1_robot2.getX() * -1 + robot1_robot2.getY() * 0);/////////////////// [-1,0]
			satnd_robot1_robot2_length=(sqrt(pow(satnd_robot1_robot2.getX(), 2) + pow(satnd_robot1_robot2.getY(), 2)));
			satnd_robot1_robot2.setX(ROBOT_RADIUS*(satnd_robot1_robot2.getX() / satnd_robot1_robot2_length));
			satnd_robot1_robot2.setY (ROBOT_RADIUS*(satnd_robot1_robot2.getY() / satnd_robot1_robot2_length));
			a1 = world.robotT[sender_robotT_pass].position + satnd_robot1_robot2;
			a2 = world.robotT[sender_robotT_pass].position - satnd_robot1_robot2;
			b1 = world.robotT[i].position + satnd_robot1_robot2;
			b2 = world.robotT[i].position - satnd_robot1_robot2;
			//DrawShape::DrawParaline(a1, b1, 255, 0, 0);
			//DrawShape::DrawParaline(a2, b2, 255, 0, 0);
			robot_2_robot_a1_b1 = Line::makeLineFromTwoPoints(a1, b1);
			robot_2_robot_a2_b2 = Line::makeLineFromTwoPoints(a2, b2);
			for (int z = 0; z < world.numO; z++)
			{

				robot = Circle(world.robotO[z].position, ROBOT_RADIUS);
				robotT1_2_robotT2.getCircleIntersectionPoints(robot, &q1, &q2);
				robot_2_robot_a1_b1.getCircleIntersectionPoints(robot, &q3, &q4);
				robot_2_robot_a2_b2.getCircleIntersectionPoints(robot, &q5, &q6);
				if (q1 != q2 && q3 != q4 && q5 != q6)
					number_robott_block_robotT++;
				if ((VecPosition(0, 0) != q1 && VecPosition(0, 0) != q2 && q1 != q2)|| (VecPosition(0, 0) != q3 && VecPosition(0, 0) != q4 && q3 != q4)|| (VecPosition(0, 0) != q5 && VecPosition(0, 0) != q6 && q5 != q6))
					cant_pass_on_the_ground[i] = true;

			}

			for (int z = 0; z < world.numT; z++)
			{
				if (z != sender_robotT_pass && z != i)
				{
					robotT1_2_robotT2.getCircleIntersectionPoints(Circle(world.robotT[z].position, ROBOT_RADIUS), &q1, &q2);
					robot_2_robot_a1_b1.getCircleIntersectionPoints(Circle(world.robotT[z].position, ROBOT_RADIUS), &q3, &q4);
					robot_2_robot_a2_b2.getCircleIntersectionPoints(Circle(world.robotT[z].position, ROBOT_RADIUS), &q5, &q6);
				}
				if ((VecPosition(0, 0) != q1 && VecPosition(0, 0) != q2 && q1 != q2) || (VecPosition(0, 0) != q3 && VecPosition(0, 0) != q4 && q3 != q4) || (VecPosition(0, 0) != q5 && VecPosition(0, 0) != q6 && q5 != q6))
					cant_pass_on_the_ground[i] = true;


			}
			danger_importance_number_robott_block_roboto[i] = 2.0*(1.0 - HighLevel::rel(number_robott_block_robotT, 0, MAX_ROBOTS_PER_TEAM_IN_THE_FIELD));
			danger_importance_number_robott_block_roboto[sender_robotT_pass] = 0;
			danger_importance_goal_distance[i] =2*(1-HighLevel::rel(world.robotT[i].position.getDistanceTo(Field::getGoalMidO()), 0, sqrt(pow(FieldLength, 2) + pow(FieldWidth, 2)) *(1.0/2)));
			danger_importance_distance_to_robott[i] = 0;// (1 - HighLevel::rel(world.robotT[index_robotT].position.getDistanceTo(world.ball.getCurrentBallPosition()), 0, sqrt(pow(FieldLength, 2) + pow(FieldWidth, 2)) *(1.0 / 2)));
			//////////////////////////distance roboto for chipe
			distance_roboto_for_chipe[i] = 0;//1-HighLevel::rel(world.robotT[i].position.getDistanceTo(world.robotO[HighLevel::nearest_robot_to_point('O', world.robotT[i].position)].position), 0, SAFE_PASS_DISTANCE);
			/////////////////////////////////
			if ((!is_in_block[i]) && (i != world.team_T.Goalie && cant_pass_on_the_ground[i] == true && SAFE_PASS_DISTANCE < world.robotT[i].position.getDistanceTo(world.robotT[index_robotT].position)))
			{
				danger_importance_number_robott_block_roboto[i] = 0;
				danger_importance_goal_distance[i] = 0;
			}
			else
			{
				cant_pass_on_the_ground[i] = false;
			}

		}

		////////////////////////////////goal
		Cone::Hole_Type Longest_Hole;
		Circle robots[MAX_ROBOTS_IN_THE_FIELD];
		int num_of_robots = 0;

		for (int i = 0; i < world.numO; i++)
			robots[num_of_robots++] = Circle(world.robotO[i].position, ROBOT_RADIUS);

		//for (int i = 0; i < world.numT; i++)
		//	robots[num_of_robots++] = Circle(world.robotT[i].position, ROBOT_RADIUS);

		for (int i = 0; i < world.numT; i++)
		{
			Cone BallToGoal(world.robotT[i].position, Field::getUpBarO(), Field::getDownBarO());
			if (BallToGoal.Get_Free_Space_In_Cone(robots, num_of_robots, Longest_Hole) > 0)
				danger_importance_goal[i] = 3*HighLevel::rel(Longest_Hole.Point_1.getDistanceTo(Longest_Hole.Point_2), 0, length_of_goal);
			else
				danger_importance_goal[i] = 0;

		}

		////////////////////////////////



		if ((timer_best_pass > RESOULOTION_OF_FIND_PASS)&&(HighLevel::find_robot_have_ball('T') != sender_robotT_pass))
		{
			int maximum = 0, maximum_index = 0;;
			timer_best_pass = 0;
			for (int j = 0; j < world.numT; j++)
			{
				if (maximum < danger_index_roboto[j])
				{
					maximum_index = j;
					maximum = danger_index_roboto[j];
				}
				danger_index_roboto[j] = 0;

			}
			denger_index_robotT = maximum_index;

		}



		/////////////////////////find max
		int max = danger_importance_number_robott_block_roboto[0]+danger_importance_goal_distance[0]+ distance_roboto_for_chipe[0]+ danger_importance_goal[0]+ danger_importance_distance_to_robott[0];
		max_index = 0;


		for (int j = 0; j < world.numT; j++)
		{

			if(danger_importance_number_robott_block_roboto[j]!=0)

			if (max < danger_importance_number_robott_block_roboto[j] + danger_importance_goal_distance[j] + distance_roboto_for_chipe[j] + danger_importance_goal[j]+ danger_importance_distance_to_robott[j])
			{
			/*	if (j == index_robotT)
				{
					if(world.robotT[index_robotT].position.getDistanceTo(Field::getGoalMidP()) > (FieldLengthG / 4) || danger_importance_goal[index_robotT] < (Field::getUpBarO().getDistanceTo(Field::getDownBarO()) / 3))
					{

					}
					else
					{
						max = danger_importance_number_robott_block_roboto[j] * 200 - danger_importance_goal_distance[j] / 100 + distance_roboto_for_chipe[j] / 5000 + danger_importance_goal[j];
						max_index = j;
					}
				}
				else*/
				{
					max = danger_importance_number_robott_block_roboto[j]+ danger_importance_goal_distance[j] + distance_roboto_for_chipe[j] + danger_importance_goal[j]+ danger_importance_distance_to_robott[j];
					max_index = j;
					last_max = j;
				}
			}
			score_pass[j] = danger_importance_number_robott_block_roboto[j] + danger_importance_goal_distance[j] + distance_roboto_for_chipe[j] + danger_importance_goal[j] + danger_importance_distance_to_robott[j];
		}

		///cout << abs(score_pass[max_pass_score] - score_pass[max_index]) << endl;
		if ((abs(score_pass[max_pass_score] - score_pass[max_index]) > 2)||(HighLevel::pass_mode != submit))
		{
			max_pass_score = max_index;
			cout << "sdf" << endl;
		}
        max_index=max_pass_score;
		///to shoot

		ball_to_shoot.getCircleIntersectionPoints(ball, &q1, &q2);
		VecPosition robotT_to_up_bar = Field::getUpBarO() - world.robotT[sender_robotT_pass].position;
		double degree = (robotT_to_up_bar).AngleBetween(Field::getDownBarO());
	//	cout << "degree= " << degree << endl;
		if ((!Balk::isIn_(q1, true, true, true, sender_robotT_pass) && !Balk::isIn_(q1, true, true, true, sender_robotT_pass)) && (world.kickMode != mode_State::KickMode::IndirectFreeKickT && (world.ball.getCurrentBallPosition().getDistanceTo(Field::getGoalMidO()) < PROBLITY_GOAL_DISTANCE) && (danger_importance_goal[sender_robotT_pass] > PROBLITY_GOAL_FREE_SPACE) && degree < 1.3))
		{
			max_index = sender_robotT_pass;
		}
		timer_best_pass++;
		danger_index_roboto[max_index]++ ;
		denger_index_robotT = max_index;
	}
	//cout << danger_index_roboto[0] << endl;
	///////////////////////////
	//cout <<"out og renge" <<world.getRobotTNumberForIndex(denger_index_robotT) << endl;

	pass_sender = index_robotT;
	if (HighLevel::pass_mode == submit) {
	//	cout << "submit" << endl;

		sender_robotT_pass = index_robotT;
		reciver__robotT_pass = denger_index_robotT;
		if (reciver__robotT_pass != index_robotT)
		{
			pass_reciver = reciver__robotT_pass;
			HighLevel::Pass(sender_robotT_pass, reciver__robotT_pass);
		}
		else
		{
			HighLevel::Shoot(sender_robotT_pass);
		}
	}
	if (HighLevel::pass_mode == expectation) {
		//cout << "expectation" << endl;
		finde_for_pass = 1;
		if (reciver__robotT_pass != sender_robotT_pass)
		{
			pass_reciver = reciver__robotT_pass;
			HighLevel::Pass(sender_robotT_pass, reciver__robotT_pass);
		}
		else
		{
			HighLevel::Shoot(sender_robotT_pass);
		}
		if (HighLevel::find_robot_have_ball('T') != -1)
		{

			HighLevel::pass_mode = receive;
		}
	}
	if (HighLevel::pass_mode == receive) {
		//cout << "receive" << endl;
		HighLevel::pass_mode = submit;
		finde_for_pass = 0;
		for (int z = 0; z < world.numT; z++)
		{
				cant_pass_on_the_ground[z] = false;
		}
	}
/*	DrawShape::DrawDot(world.robotT[index_robotT].position, 50, 0, 255, 0);
	DrawShape::DrawDot(world.robotT[denger_index_robotT].position, 50, 0, 0, 255);
	DrawShape::DrawDot(Field::getGoalMidO());
	sleep(0.0100);
	DrawShape::ClearCircles();
	DrawShape::ClearLines();*/
	//world.robotT[index_robotT].destination_set = true;
	max_pass_score = last_max;
	return 0;
}

//checked
int HighLevel::find_robot_have_ball(char team)
{
	VecPosition robot_front;
	double distance2ball;
	int distance2ball_index = 0;
	int number_robott_block_roboto = 0;
	if (team == 'O')
	{
		VecPosition robot_front(world.robotO[0].position.getX() + cos(world.robotO[0].angle)*ROBOT_RADIUS, world.robotO[0].position.getY() + sin(world.robotO[0].angle)*ROBOT_RADIUS);
		distance2ball = robot_front.getDistanceTo(world.ball.getCurrentBallPosition());
		for (int i = 0; i < world.numO; i++)
		{
			VecPosition robot_front(world.robotO[i].position.getX() + cos(world.robotO[i].angle)*ROBOT_RADIUS, world.robotO[i].position.getY() + sin(world.robotO[i].angle)*ROBOT_RADIUS);
			if (distance2ball > robot_front.getDistanceTo(world.ball.getCurrentBallPosition()))
			{
				distance2ball = robot_front.getDistanceTo(world.ball.getCurrentBallPosition());
				distance2ball_index = i;
			}
		}
	}
	else if (team == 'T')
	{
		VecPosition robot_front(world.robotT[0].position.getX() + cos(world.robotT[0].angle)*ROBOT_RADIUS, world.robotT[0].position.getY() + sin(world.robotT[0].angle)*ROBOT_RADIUS);
		distance2ball = robot_front.getDistanceTo(world.ball.getCurrentBallPosition());
		for (int i = 0; i < world.numT; i++)
		{
			VecPosition robot_front(world.robotT[i].position.getX() + cos(world.robotT[i].angle)*ROBOT_RADIUS, world.robotT[i].position.getY() + sin(world.robotT[i].angle)*ROBOT_RADIUS);
			if (distance2ball > robot_front.getDistanceTo(world.ball.getCurrentBallPosition()))
			{
				distance2ball = robot_front.getDistanceTo(world.ball.getCurrentBallPosition());
				distance2ball_index = i;
			}
		}
	}
	//DISTANCE ERROR = 90
	if (distance2ball <= BALL_RADIUS + DISTANCE_ROBOT_HAVE_BALL)
		return distance2ball_index;
	else
		return -1;
}

//pass index one to index2
void HighLevel::Pass(int index1, int index2)
{
    if(index2==world.team_T.Goalie)
    {
        HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));

    }
    else {
        // cout<<"id sender:  "<<world.getRobotTNumberForIndex(index1)<<"id reciver:   "<<world.getRobotTNumberForIndex(index2)<<'\n';
        index_pass_senderT = index1;
        VecPosition mid_bigest_holl;
        VecPosition dest;
        Line ball_to_index2 = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(),
                                                          world.robotT[index2].position);
        VecPosition point1, point2;
        Circle circle(world.ball.getCurrentBallPosition(), ROBOT_RADIUS + BALL_RADIUS);
        ball_to_index2.getCircleIntersectionPoints(circle, &point1, &point2);
        if (HighLevel::pass_mode == submit) {

            if (world.robotT[index2].position.getDistanceTo(point1) >
                world.robotT[index2].position.getDistanceTo(point2))
                dest = point1;
            else
                dest = point2;

            VecPosition robot1_to_robot2 = world.robotT[index2].position - world.robotT[index1].position;
            //DrawShape::DrawParaline(world.robotT[index1].position, world.robotT[index2].position);
            double angle_robot1_to_robot2 = -
                                                    ((robot1_to_robot2).AngleBetween(VecPosition(1, 0))) *
                                            sign(world.robotT[index1].position.getY() -
                                                 world.robotT[index2].position.getY());
            double angle_robot2_to_robot1 = -((-robot1_to_robot2).AngleBetween(VecPosition(1, 0))) *
                                            sign(world.robotT[index2].position.getY() -
                                                 world.robotT[index1].position.getY());

            world.robotT[index1].destination_angle = angle_robot1_to_robot2;
            world.robotT[index2].destination_angle = angle_robot2_to_robot1;
            world.robotT[index1].destination_position = dest;


//angle robot to ball
            VecPosition robot1_to_ballVec = world.ball.getCurrentBallPosition() - world.robotT[index1].position;
            double angle_robot1_to_ball = -((robot1_to_ballVec).AngleBetween(VecPosition(1, 0))) *
                                          sign(world.robotT[index1].position.getY() -
                                               world.ball.getCurrentBallPosition().getY());


            Line robot12robot2 = Line::makeLineFromTwoPoints(world.robotT[index1].position,
                                                             world.robotT[index2].position);
            VecPosition point3, point4;

            if ((abs(angle_robot1_to_robot2 - world.robotT[index1].angle) <= CORRECTION_FACTOR) &&
                (abs(angle_robot1_to_ball - world.robotT[index1].angle) <= CORRECTION_FACTOR)) {

                //HighLevel::pass_mode = expectation;
                finde_for_pass = 1;
                if (cant_pass_on_the_ground[index1] == true) {
                    world.robotT[index1].destination_position = world.ball.getCurrentBallPosition();
                    //		world.robotT[index1].velocity = VecPosition(2*(world.robotT[index2].position.getX() - world.robotT[index1].position.getX()), 2*(world.robotT[index2].position.getY() - world.robotT[index1].position.getY()));
                    world.robotT[index1].shoot_or_chip = 1;
                    if (1.1 * 0.70710678 *
                        sqrt(((world.robotT[index1].position.getDistanceTo(world.robotT[index2].position) / 1000) *
                              9.8) / (1.5 - (0.43))) >= MAX_BALL_SPEED) {
                        world.robotT[index1].kick_power = MAX_BALL_SPEED / 0.70710678;
                        //world.robotT[index1].kick_power = 3;
                    } else
                        world.robotT[index1].kick_power = 0.6 * 1 * sqrt(((world.robotT[index1].position.getDistanceTo(
                                world.robotT[index2].position) / 1000) * 9.8) / (1.5 - (0.43)));
                } else {
                    world.robotT[index1].destination_position = world.ball.getCurrentBallPosition();
                    world.robotT[index1].shoot_or_chip = 0;
                    if (1.1 *
                        sqrt(((world.robotT[index1].position.getDistanceTo(world.robotT[index2].position) / 1000) *
                              9.8) / (1.5 - (0.43))) >= MAX_BALL_SPEED)
                        world.robotT[index1].kick_power = MAX_BALL_SPEED;
                    else
                        world.robotT[index1].kick_power = 0.8 * sqrt(((world.robotT[index1].position.getDistanceTo(
                                world.robotT[index2].position) / 1000) * 9.8) / (1.5 - (0.43)));
                }
            } else {
                world.robotT[index1].shoot_or_chip = 1;
                world.robotT[index1].kick_power = 0;
            }
            if (HighLevel::nearest_robot_to_ball('T') != index1 &&
                ((abs(world.ball.velocity.getX()) > 0.1) || (abs(world.ball.velocity.getY()) > 0.1)) &&
                finde_for_pass == 1) {
                cout << "pass\n";
                HighLevel::pass_mode = expectation;
            }

        } else if (HighLevel::pass_mode == expectation) {
            finde_for_pass = 0;
            VecPosition end = VecPosition((world.ball.getCurrentBallPosition().getX() + 7 * world.ball.velocity.getX()),
                                          (world.ball.getCurrentBallPosition().getY() +
                                           7 * world.ball.velocity.getY()));
            Paraline ball_go = Paraline(world.ball.getCurrentBallPosition(), end);
            VecPosition stand = ball_go.getPointOnParalineClosestTo(world.robotT[index2].position);
            //DrawShape::DrawDot(stand, 85, 255, 255, 0);
            //DrawShape::DrawParaline(ball_go, 255, 0, 255);
            VecPosition robot1_to_robot2 = world.robotT[index2].position - world.ball.getCurrentBallPosition();
            double angle_robot2_to_robot1 = -((-robot1_to_robot2).AngleBetween(VecPosition(1, 0))) *
                                            sign(world.robotT[index2].position.getY() -
                                                 world.ball.getCurrentBallPosition().getY());
            world.robotT[index2].destination_angle = angle_robot2_to_robot1;
            if (world.robotT[index1].shoot_or_chip == 0) {
                if (world.robotT[index2].position.getDistanceTo(point1) >
                    world.robotT[index2].position.getDistanceTo(point2))
                    dest = point2;
                else
                    dest = point1;
                world.robotT[index2].destination_position = dest;
            }
            //Line ball_velocity = Line::makeLineFromTwoPoints(VecPosition(0,0), world.ball.velocity);
            //world.robotT[index2].destination_position= /*world.ball.velocity +*/ world.ball.getCurrentBallPosition();

            if (abs(robot1_to_robot2.AngleBetween(world.ball.velocity)) > (1.5 * M_PI)) {
                world.robotT[index2].destination_position = world.ball.getCurrentBallPosition();
                world.robotT[index2].destination_set = true;
            } else {
                //world.robotT[index2].velocity = VecPosition(4 * (stand.getX() - world.robotT[index2].position.getX()), 4 * (stand.getY() - world.robotT[index2].position.getY()));
                world.robotT[index2].destination_position = stand;
                world.robotT[index2].destination_set = true;
            }


            if (HighLevel::find_robot_have_ball('T') == index2) {
                world.robotT[index2].destination_position = world.ball.getCurrentBallPosition();
                world.robotT[index2].destination_set = true;

                HighLevel::pass_mode = receive;
            } else if ((world.ball.velocity).AngleBetween(robot1_to_robot2) > CORRECTION_FACTOR * 5 ||
                       ((world.ball.velocity.getX() < CORRECTION_FACTOR * 5) &&
                        world.ball.velocity.getY() < CORRECTION_FACTOR * 5)) {
                world.robotT[index2].destination_position = world.ball.getCurrentBallPosition();
                world.robotT[index2].destination_set = true;
                HighLevel::pass_mode = receive;
            }
        }
        world.robotT[index1].destination_set = true;
        //world.robotT[index2].destination_set = true;

    }
}


void HighLevel::time_out()
{
	for (int i = 0; i < world.numT; i++)
	{

		world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(FieldLength / 4 + (pow(-1, i)*floor((i + 1) / 2) * 300), FieldWidth / 2))].destination_position = VecPosition(FieldLength / 4 + (pow(-1, i)*floor((i + 1) / 2) * 300), FieldWidth / 2);
		world.robotT[i].destination_angle = -M_PI / 2;
	}
}

///////////////////////probabilities
double HighLevel::rel(double x, double min, double max)
{
	if (x >= max)
	{
		return 1;
	}
	else if (x <= min)
	{
		return min;
	}
	else
	{
		return ((x-min)/(max-min));
	}
}

void HighLevel::plan_scor(int number_of_attacker)
{
	VecPosition max_score_position;
	VecPosition socre_position[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	///////////////////////////////////////////////////////////set if you click on x>0
	if (world.mouseX > 0 && k == 0 && plus_plan_score > 1)
	{
		u = 0;
		k = 1;
		plus_plan_score = 0;
		//DrawShape::ClearCircles();
	}
	else
	{
		k = 0;
		plus_plan_score++;

	}
	//////////////////////////////////////////////////////////////////////////
	if (u == 0)
	{
		double max = 0;
		typedef std::vector<double> int_vector;
		typedef std::vector<int_vector> int_double_vector;
		int_double_vector score((int)(FieldLength / ROBOT_RADIUS * PLAN_SCORE_CPU) + 2, int_vector((int)(FieldWidth / ROBOT_RADIUS * PLAN_SCORE_CPU)));
		//VecPosition point[((int)(FieldLength / ROBOT_RADIUS * 2)) + 2][(int)(FieldWidth / ROBOT_RADIUS * 2)];
		//double score[(int)(FieldLength / ROBOT_RADIUS * 2) + 2][(int)(FieldWidth / ROBOT_RADIUS * 2)];
		//cout << (int)((FieldLength*FieldWidth) / (ROBOT_RADIUS * 2 *ROBOT_RADIUS * 2)) << endl;
		//for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 2; i++)
		//{
		//	for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
		//	{
		//		kk = 0;
		//		point[i][j] = VecPosition(((FieldLength / 2) - ROBOT_RADIUS) - (i*ROBOT_RADIUS * 2), ((FieldWidth / 2) - ROBOT_RADIUS) - (j*ROBOT_RADIUS * 2));
		//		score[i][j] = 0;
		//		for (int k = 0; k < world.numO; k++)
		//		{
		//			score[i][j] +=  HighLevel::rel((world.robotO[k].position.getDistanceTo(point[i][j])), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 8));
		//			kk += (world.robotO[k].position.getDistanceTo(point[i][j]));
		//		}
		//		//score[i][j] += HighLevel::rel(point[i][j].getDistanceTo(Field::getGoalMidO()), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 2));
		//		//score[i][j] *= 1-((point[i][j].getDistanceTo(Field::getGoalMidO())) / (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth)/2));
		//		score[i][j] -= HighLevel::rel(kk/ world.numO, 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 8));
		//
		//		if (score[i][j] > max)
		//			max = score[i][j];
		//	}
		//}


		/////////////////////////////////////////////////////////////distance to robot
		double x, y, sum = 0, sum1 = 0;
		for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * PLAN_SCORE_CPU)) + 1; i++)
		{
			for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * PLAN_SCORE_CPU)); j++)
			{
			//	point[i][j] = VecPosition(((FieldLength / 2) - ROBOT_RADIUS) - (i*ROBOT_RADIUS * 2), ((FieldWidth / 2) - ROBOT_RADIUS) - (j*ROBOT_RADIUS * 2));
				score[i][j] = 0;
				sum = 0;
				for (int a = 1; a < world.numO /*+ num_of_robotT + 1*/; a++)
				{
					x = world.numO - 1/* num_of_robotT - 2*/;
					y = 1;
					sum1 = 0;
					for (int b = 2; b <= a; b++)
					{
						y *= x;
						x -= 1;
					}
					for (int c = 0; c < world.numO /*+ num_of_robotT*/; c++)
					{
						sum1 += y*HighLevel::rel(world.robotO[c].position.getDistanceTo(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 8));

					}
					sum1 /= a*((a % 2 == 1) ? 1 : -1);
					sum += sum1;
				}
				score[i][j] = sum;

				if (score[i][j] > max)
					max = score[i][j];
			}
		}
		/////////////////////////////////////////////////////////////////////





		/////////////////////////////////////////////////////////////



		Cone::Hole_Type Longest_Hole;
		Circle robots[MAX_ROBOTS_IN_THE_FIELD];
		int num_of_robots = 0;

		for (int t = 0; t < world.numO; t++)
			robots[num_of_robots++] = Circle(world.robotO[t].position, ROBOT_RADIUS);
		/*	for (int t = 0; t < num_of_robotT; t++)
				robots[num_of_robots++] = Circle(socre_position[t], ROBOT_RADIUS);*/


		double max_score = 0;
		double max_posx = 0, max_posy = 0;

		for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * PLAN_SCORE_CPU)) + 1; i++)
		{
			if ((i >= ((int)(FieldLength / (ROBOT_RADIUS * PLAN_SCORE_CPU)))))
			{

				u = 1;
			}
			for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * PLAN_SCORE_CPU)); j++)
			{
				///////////////////////////////////////////////////longest to goal
				Cone BallToGoal(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/, Field::getUpBarO(), Field::getDownBarO());
				score[i][j] = 1 * (HighLevel::rel(score[i][j], 0, max));
				if ((BallToGoal.Get_Free_Space_In_Cone(robots, num_of_robots, Longest_Hole) > 0) && (PLAN_SCORE_POINT_X/*point[i][j].getX()*/ > (((-FieldLength / 2) + (PenaltyAreaWidth / 2)))))
				{
					score[i][j] += HighLevel::rel((Longest_Hole.Point_1 - VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/).AngleBetween(Longest_Hole.Point_2 - VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/), 0, 3.14 / 2);
				}
				if (PLAN_SCORE_POINT_X/*point[i][j].getX()*/ > ((-FieldLength / 2) + (PenaltyAreaWidth / 2)))
				{
					score[i][j] += 0.25*(1 - HighLevel::rel(/*point[i][j]*/VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y).getDistanceTo(Field::getGoalMidO()), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 2)));
				}
				/////////////////////////////////////////////////////////
				score[i][j] += 0.50*(1 - HighLevel::rel(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y).getDistanceTo(world.ball.getCurrentBallPosition()),0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 2)));
				score[i][j] = HighLevel::rel(score[i][j], 0, 2.75);
				///penalty area
				//abs(point[i][j].getY()) < ((PenaltyAreaLength / 2) + 3.3 * ROBOT_RADIUS)
				//(point[i][j].getX() < (((-FieldLength / 2) + PenaltyAreaWidth) + 3.3*ROBOT_RADIUS)&&(point[i][j].getX() >((-FieldLength / 2) - 3.3 * ROBOT_RADIUS))
				if (((PLAN_SCORE_POINT_X< PLAN_SCORE_MAXIMOM_X && PLAN_SCORE_POINT_Y < PLAN_SCORE_MAXIMOM_Y) && (PLAN_SCORE_POINT_X> PLAN_SCORE_MINIMOM_X && PLAN_SCORE_POINT_Y > PLAN_SCORE_MINIMOM_Y)) || ((PLAN_SCORE_POINT_X> -PLAN_SCORE_MAXIMOM_X && PLAN_SCORE_POINT_Y < PLAN_SCORE_MAXIMOM_Y) && (PLAN_SCORE_POINT_X< -PLAN_SCORE_MINIMOM_X && PLAN_SCORE_POINT_Y > PLAN_SCORE_MINIMOM_Y)) || (world.robotT[index_pass_senderT].position.getDistanceTo(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y))<MERGE_DISTANCE * 3))
				{
					score[i][j] = 0;
					//DrawShape::DrawDot(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y), 70, 0, 0, 255);
				}
				if (max_score <= score[i][j])
				{
					max_score_position.setX(/*point[i][j].getX()*/PLAN_SCORE_POINT_X);
					max_score_position.setY(/*point[i][j].getY()*/PLAN_SCORE_POINT_Y);
					max_posx = /*point[i][j].getX()*/PLAN_SCORE_POINT_X;
					max_posy = /*point[i][j].getY()*/PLAN_SCORE_POINT_Y;
					//DrawShape::DrawDot(max_score_position, 70, 255, 0, 0);
					max_score = score[i][j];
				}
			//	if ((u == 0))
			//	{
			//		DrawShape::DrawDot(/*point[i][j]*/VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y), 70, (score[i][j] > 0.75) ? 0 : ((score[i][j])) * 255, /*(score[i][j]>0.80) ? */ (score[i][j]) * 255/*:0*/, 0);
			//		DrawShape::DrawDot(max_score_position, 70, 255, 0, 0);
			//	}

			}

		}

		socre_position[0] = max_score_position;
		//DrawShape::DrawDot(VecPosition(max_posx, max_posy), 70, 255, 0, 0);
		//for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 1; i++)
		//{
		//	for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
		//	{
		//		score[i][j] -= (1 - HighLevel::rel(world.robotT[pass_sender].position.getDistanceTo(/*point[i][j]*/VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 6)));
		//		score[i][j] -= (1 - HighLevel::rel(world.robotT[pass_reciver].position.getDistanceTo(/*point[i][j]*/VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 6)));
		//		score[i][j] = HighLevel::rel(score[i][j], 0, 1);
		//	}
		//}
		//DrawShape::ClearCircles();
	//	cout << "/////" <<world.getRobotTNumberForIndex( index_pass_senderT) << endl;
		//for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 1; i++)
		//{
		//	for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
		//	{
		//		if (world.robotT[index_pass_senderT].position.getDistanceTo(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y))<MERGE_DISTANCE*3/*point[i][j]*/)
		//		{
		//			score[i][j] = -1000;//1.75*(1 - HighLevel::rel(world.robotT[index_pass_senderT].position.getDistanceTo(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 6)));
		//			//score[i][j] = HighLevel::rel(score[i][j], 0, 1);
		//		}
		//	}
		//}
		for (int l = 0; l <  number_of_attacker; l++)
		{
			socre_position[l] = max_score_position;
			HighLevel::check_last_destination_set(world.getIndexForRobotTNumber(7), VecPosition(max_posx, max_posy));
			//DrawShape::DrawDot(VecPosition(max_posx, max_posy), 70, 255, 0, 0);
			for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * PLAN_SCORE_CPU)) + 1; i++)
			{
				for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * PLAN_SCORE_CPU)); j++)
				{
					score[i][j] -= (1 - HighLevel::rel(VecPosition(max_posx, max_posy).getDistanceTo(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 6)));
					score[i][j] = HighLevel::rel(score[i][j], 0, 1);
				}
			}
			max_score = 0;
			for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * PLAN_SCORE_CPU)) + 1; i++)
			{
				for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * PLAN_SCORE_CPU)); j++)
				{
					if (max_score <= score[i][j])
					{
						max_score_position.setX(/*point[i][j].getX()*/PLAN_SCORE_POINT_X);
						max_score_position.setY(/*point[i][j].getY()*/PLAN_SCORE_POINT_Y);
						max_score = score[i][j];
						max_posx = /*point[i][j].getX()*/PLAN_SCORE_POINT_X;
						max_posy = /*point[i][j].getY()*/PLAN_SCORE_POINT_Y;
					}
				}
			}

		}
	}

}



void HighLevel::DefenceHoleCover(int number_robotT,int index_robot[])
{
	static Cone::Hole_Type bigest_hole;
	VecPosition target;
	Circle robots[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	int num_of_robots = 0;

	//Cone::Hole_Type bigest_hole;
	bool is_defence_robot;
	for (int i = 0; i < world.numT; i++)
	{
		is_defence_robot = false;
		for (int j = 0; j < world.numT; j++)
		{
			if (i == index_robot[j])
				is_defence_robot = true;

		}
		if ((world.getIndexForRobotTNumber(world.team_T.Goalie) != i)&&(!is_defence_robot))
			robots[num_of_robots++] = Circle(world.robotT[i].position, ROBOT_RADIUS);
	}
	Cone ball_to_goal((world.ball.getCurrentBallPosition() + (world.ball.getVelocity())), Field::getUpBarP(), Field::getDownBarP());


	/////////////////////////////////////////
	//Circle c((Field::getUpBarP() + Field::getDownBarP()/2), 80);
	//GLFrame::addCirlceToPainting(c,true);
	//GLFrame::addLineToPainting(world.ball.getCurrentBallPosition(), Field::getUpBarP());
	//GLFrame::addLineToPainting(world.ball.getCurrentBallPosition(), Field::getDownBarP());

	////////////////////////////////////////
	if (ball_to_goal.Get_Free_Space_In_Cone(robots, num_of_robots, bigest_hole) > 5 && (world.ball.getCurrentBallPosition() + (world.ball.getVelocity())).getX() < Field::getGoalMidP().getX())
	{
		//Support::bigest_hole = bigest_hole;
		bigest_hole = bigest_hole;
		/*! creating goal shape */
		//before : Goal_Shape goal_shape( Estimation::getMarkOfMathematicalEquation( 1000, 5000, 100, 500, Field::getGoalMidP().getDistanceTo(world.ball.getCurrentBallPosition()) ) );
		Goal_Shape goal_shape(Estimation::getMarkOfMathematicalEquation(6*PenaltyAreaRadius, 24*PenaltyAreaRadius, 2*PenaltyAreaRadius, 2.4*PenaltyAreaRadius, Field::getGoalMidP().getDistanceTo((world.ball.getCurrentBallPosition() + (world.ball.getVelocity() )))));

		/*! a line between ball position and target ---> "ball_to_target" */

		VecPosition _temp_ball_to_up_of_hole = bigest_hole.Point_1 - (world.ball.getCurrentBallPosition() + (world.ball.getVelocity() ));
		VecPosition _temp_ball_to_down_of_hole = bigest_hole.Point_2 - (world.ball.getCurrentBallPosition() + (world.ball.getVelocity()));
		double _temp_angle = VecPosition::AngleBetweenWithSgn(_temp_ball_to_up_of_hole, _temp_ball_to_down_of_hole);
		_temp_ball_to_up_of_hole.rotate(_temp_angle / 2.0);

		Line ball_to_target = Line::makeLineFromPositionAndAngle((world.ball.getCurrentBallPosition() + (world.ball.getVelocity())), Rad2Deg(_temp_ball_to_up_of_hole.Angle()));




		/*! detecting intersection between "ball_to_target" and goal shape ---> "target" */

		target = goal_shape.getIntersection(ball_to_target);



		/*! this is for not letting robot to go out of the field in our goal line */
		double _temp_distance;
		_temp_distance = Field::getPenaltyPointP().getDistanceTo(target);
		double distance_that_robot_will_go_out_of_field = Field::getPenaltyPointP().getX() - target.getX();
		//if (distance_that_robot_will_go_out_of_field < 90)
		{
			distance_that_robot_will_go_out_of_field = 90 - distance_that_robot_will_go_out_of_field;
			if (target.getY() > 0)
			{
				double _asin = distance_that_robot_will_go_out_of_field / _temp_distance;
				if (_asin <= 1 && _asin >= -1)
					_asin = asin(_asin);
				else
					_asin = 0;
				VecPosition temp = VecPosition(_temp_distance, Rad2Deg((target - Field::getPenaltyPointP()).Angle() + _asin), POLAR);
				/*if (temp.ToString() == "X:nan Y:nan")
				{
				int x = 9 / 0;
				}*/
				target = Field::getPenaltyPointP() + temp;

			}
			else
			{
				double _asin = distance_that_robot_will_go_out_of_field / _temp_distance;
				if (_asin <= 1 && _asin >= -1)
					_asin = asin(_asin);
				else
					_asin = 0;

				VecPosition temp = VecPosition(_temp_distance, Rad2Deg((target - Field::getPenaltyPointP()).Angle() - _asin), POLAR);

				/*if (temp.ToString() == "X:nan Y:nan")
				{
				int x = 9 / 0;
				}*/
				target = Field::getPenaltyPointP() + temp;

			}
		}
		/*! this is for not letting robot to go out of the field in our goal line */
		//z		World::angle = (world.ball.getCurrentBallPosition() + (world.ball.getVelocity() / kk) - world.robotT[world.decider_id].position).Angle();
	}
	//else
	{
		//before: Goal_Shape goal_shape( Estimation::getMarkOfMathematicalEquation( 1000, 5000, 100, 500, Field::getGoalMidP().getDistanceTo(world.ball.getCurrentBallPosition()) ) );
		Goal_Shape goal_shape(Estimation::getMarkOfMathematicalEquation(1.25*PenaltyAreaRadius, 6.25*PenaltyAreaRadius, 0.125*PenaltyAreaRadius, 0.625*PenaltyAreaRadius, Field::getGoalMidP().getDistanceTo((world.ball.getCurrentBallPosition() + (world.ball.getVelocity() )))));
	//	target = goal_shape.getIntersection(Line::makeLineFromTwoPoints(VecPosition(0, 0), Field::getGoalMidP()));

		//target = Field::getGoalMidP(); //+ VecPosition(0, 100);
		//z	World::angle = M_PI;
	}
	/*decider.setMeduimLevel("MNone");
	decider.setLowLevel("GoTowardAndStop");
	((GoTowardAndStop*)(decider.lowlevels[decider.currentLowLevel]))->SetParam(target, angle, config->getRobotsMaxSpeed());

	((MNone*)(decider.mediumlevels[decider.currentMediumLevel]))->decide(world);*/
	HighLevel::several_position_Line(target, number_robotT, index_robot);

}


void HighLevel::direct_free_kick(int number_of_attacker,int sender_index)
{
	VecPosition max_score_position;
	VecPosition socre_position[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	///////////////////////////////////////////////////////////set if you click on x>0


		double max = 0;
		typedef std::vector<double> int_vector;
		typedef std::vector<int_vector> int_double_vector;
		int_double_vector score((int)(FieldLength / ROBOT_RADIUS * 2) + 2, int_vector((int)(FieldWidth / ROBOT_RADIUS * 2)));
		//VecPosition point[((int)(FieldLength / ROBOT_RADIUS * 2)) + 2][(int)(FieldWidth / ROBOT_RADIUS * 2)];
		//double score[(int)(FieldLength / ROBOT_RADIUS * 2) + 2][(int)(FieldWidth / ROBOT_RADIUS * 2)];
		//cout << (int)((FieldLength*FieldWidth) / (ROBOT_RADIUS * 2 *ROBOT_RADIUS * 2)) << endl;
		//for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 2; i++)
		//{
		//	for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
		//	{
		//		kk = 0;
		//		point[i][j] = VecPosition(((FieldLength / 2) - ROBOT_RADIUS) - (i*ROBOT_RADIUS * 2), ((FieldWidth / 2) - ROBOT_RADIUS) - (j*ROBOT_RADIUS * 2));
		//		score[i][j] = 0;
		//		for (int k = 0; k < world.numO; k++)
		//		{
		//			score[i][j] +=  HighLevel::rel((world.robotO[k].position.getDistanceTo(point[i][j])), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 8));
		//			kk += (world.robotO[k].position.getDistanceTo(point[i][j]));
		//		}
		//		//score[i][j] += HighLevel::rel(point[i][j].getDistanceTo(Field::getGoalMidO()), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 2));
		//		//score[i][j] *= 1-((point[i][j].getDistanceTo(Field::getGoalMidO())) / (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth)/2));
		//		score[i][j] -= HighLevel::rel(kk/ world.numO, 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 8));
		//
		//		if (score[i][j] > max)
		//			max = score[i][j];
		//	}
		//}


		/////////////////////////////////////////////////////////////distance to robot
		double x, y, sum = 0, sum1 = 0;
		for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 1; i++)
		{

			for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
			{
				score[i][j] = 0;
				sum = 0;
				for (int a = 1; a < world.numO ; a++)
				{
					x = world.numO - 1;
					y = 1;
					sum1 = 0;
					for (int b = 2; b <= a; b++)
					{
						y *= x;
						x -= 1;
					}
					for (int c = 0; c < world.numO ; c++)
					{
						sum1 += y*HighLevel::rel(world.robotO[c].position.getDistanceTo(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 8));

					}
					sum1 /= a*((a % 2 == 1) ? 1 : -1);
					sum += sum1;
				}
				score[i][j] = sum;
				if (score[i][j] > max)
					max = score[i][j];
			}
		}
		/////////////////////////////////////////////////////////////////////

		/////////////////////////////////////////////////////////////



		Cone::Hole_Type Longest_Hole;
		Circle robots[MAX_ROBOTS_IN_THE_FIELD];
		int num_of_robots = 0;

		for (int t = 0; t < world.numO; t++)
			robots[num_of_robots++] = Circle(world.robotO[t].position, ROBOT_RADIUS);
		/*	for (int t = 0; t < num_of_robotT; t++)
		robots[num_of_robots++] = Circle(socre_position[t], ROBOT_RADIUS);*/


		double max_score = 0;
		double max_posx = 0, max_posy = 0;

		for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 1; i++)
		{
			for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
			{
				///////////////////////////////////////////////////longest to goal
				Cone BallToGoal(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/, Field::getUpBarO(), Field::getDownBarO());
				score[i][j] = 1 * (HighLevel::rel(score[i][j], 0, max));
				if ((BallToGoal.Get_Free_Space_In_Cone(robots, num_of_robots, Longest_Hole) > 0) && (PLAN_SCORE_POINT_X/*point[i][j].getX()*/ > (((-FieldLength / 2) + (PenaltyAreaWidth / 2)))))
				{
					score[i][j] += HighLevel::rel((Longest_Hole.Point_1 - VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/).AngleBetween(Longest_Hole.Point_2 - VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/), 0, 3.14 / 2);
				}
				if (PLAN_SCORE_POINT_X/*point[i][j].getX()*/ > ((-FieldLength / 2) + (PenaltyAreaWidth / 2)))
				{
					score[i][j] += 0.25*(1 - HighLevel::rel(/*point[i][j]*/VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y).getDistanceTo(Field::getGoalMidO()), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 2)));
				}
				/////////////////////////////////////////////////////////

				score[i][j] = HighLevel::rel(score[i][j], 0, 2.25);
				///penalty area
				//abs(point[i][j].getY()) < ((PenaltyAreaLength / 2) + 3.3 * ROBOT_RADIUS)
				//(point[i][j].getX() < (((-FieldLength / 2) + PenaltyAreaWidth) + 3.3*ROBOT_RADIUS)&&(point[i][j].getX() >((-FieldLength / 2) - 3.3 * ROBOT_RADIUS))
				if ((/*point[i][j].getX() */PLAN_SCORE_POINT_X< PLAN_SCORE_MAXIMOM_X && /*point[i][j].getY()*/PLAN_SCORE_POINT_Y < PLAN_SCORE_MAXIMOM_Y) && (/*point[i][j].getX()*/ PLAN_SCORE_POINT_X> PLAN_SCORE_MINIMOM_X && /*point[i][j].getY()*/PLAN_SCORE_POINT_Y > PLAN_SCORE_MINIMOM_Y))
				{
					score[i][j] = 0;
					//DrawShape::DrawDot(point[i][j], 70, 0, 0, 255);
				}
				if (max_score <= score[i][j])
				{
					max_score_position.setX(/*point[i][j].getX()*/PLAN_SCORE_POINT_X);
					max_score_position.setY(/*point[i][j].getY()*/PLAN_SCORE_POINT_Y);
					max_posx = /*point[i][j].getX()*/PLAN_SCORE_POINT_X;
					max_posy = /*point[i][j].getY()*/PLAN_SCORE_POINT_Y;
					//DrawShape::DrawDot(max_score_position, 70, 255, 0, 0);
					max_score = score[i][j];
				}
				//	if ((u == 0))
				//	{
				//		DrawShape::DrawDot(/*point[i][j]*/VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y), 70, (score[i][j] > 0.75) ? 0 : ((score[i][j])) * 255, /*(score[i][j]>0.80) ? */ (score[i][j]) * 255/*:0*/, 0);
				//		DrawShape::DrawDot(max_score_position, 70, 255, 0, 0);
				//	}

			}

		}

		socre_position[0] = max_score_position;
		for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 1; i++)
		{
			for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
			{
				//score[i][j] += (1 - HighLevel::rel(world.robotT[pass_sender].position.getDistanceTo(/*point[i][j]*/VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 6)));
				//score[i][j] += (1 - HighLevel::rel(world.robotT[pass_reciver].position.getDistanceTo(/*point[i][j]*/VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 6)));
				score[i][j] += 0.5 * (1 - HighLevel::rel(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y).getDistanceTo(world.ball.getCurrentBallPosition()), 0, (FieldLength + FieldWidth) / 2));
				//score[i][j] -= 1 - HighLevel::rel(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y).getDistanceTo(world.ball.getCurrentBallPosition()), 0, DISTANCE_TO_BALL_IN_STOPMODE / 4);
				score[i][j] = HighLevel::rel(score[i][j], 0, 1);
			}
		}
		//world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set == true;
		//world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_position = VecPosition(1000, 1000);
		//DrawShape::ClearCircles();
		int op;
		for (int l = 0; l < number_of_attacker; l++)
		{
			socre_position[l] = max_score_position;
			op = HighLevel::nearest_robot_to_point('T', VecPosition(max_posx, max_posy));
			//cout << world.getRobotTNumberForIndex(op) << ":::::::::" << VecPosition(max_posx, max_posy)<<endl;
			world.robotT[op].destination_position = VecPosition(max_posx, max_posy);
			//DrawShape::DrawDot(VecPosition(max_posx, max_posy), 70, 255, 0, 0);
			for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 1; i++)
			{
				for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
				{

					score[i][j] -= (1 - HighLevel::rel(VecPosition(max_posx, max_posy).getDistanceTo(VecPosition(PLAN_SCORE_POINT_X, PLAN_SCORE_POINT_Y)/*point[i][j]*/), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 6)));
					score[i][j] = HighLevel::rel(score[i][j], 0, 1);
				}
			}
			max_score = 0;
			for (int i = 0; i < (int)(FieldLength / (ROBOT_RADIUS * 2)) + 1; i++)
			{
				for (int j = 0; j < (int)(FieldWidth / (ROBOT_RADIUS * 2)); j++)
				{

					if (max_score <= score[i][j])
					{
						max_score_position.setX(/*point[i][j].getX()*/PLAN_SCORE_POINT_X);
						max_score_position.setY(/*point[i][j].getY()*/PLAN_SCORE_POINT_Y);
						max_score = score[i][j];
						max_posx = /*point[i][j].getX()*/PLAN_SCORE_POINT_X;
						max_posy = /*point[i][j].getY()*/PLAN_SCORE_POINT_Y;
					}
				}
			}

		}


}

//void HighLevel::defence_scor(int number_of_defender)
//{
//	VecPosition point[((int)(FieldLength / ROBOT_RADIUS * 2)) + 2][(int)(FieldWidth / ROBOT_RADIUS * 2)];
//	double score[(int)(FieldLength / ROBOT_RADIUS * 2) + 2][(int)(FieldWidth / ROBOT_RADIUS * 2)];
//	int resolution;
//	////////////////////////////////////////////////////////specify resolution
//	resolution =  (ROBOT_RADIUS);
//	if (onces_program == 0)
//	{
//		for (int i = 0; i < (int)(FieldLength / (2*resolution)); i++)
//			for (int j = 0; j < (int)(FieldWidth / (resolution)); j++)
//				point[i][j] = VecPosition((FieldLength / 2) - (i*resolution), (FieldWidth / 2) - (j*resolution));
//		onces_program = 1;
//	}
//
//	///////////////////////////////////////////////////////////set if you click on x>0
//	if (world.mouseX > 0 && k == 0)
//	{
//		u = 0;
//		k = 1;
//		DrawShape::ClearCircles();
//	}
//	else
//	{
//		k = 0;
//	}
//	//////////////////////////////////////////////////////////////////////////
//	if (u == 0)
//	{
//		Cone::Hole_Type Longest_Hole;
//		Cone::Hole_Type Longest_Hole_no_Robot;
//		Circle robots[MAX_ROBOTS_IN_THE_FIELD];
//		int num_of_robots = 1;
//
//		for (int t = 0; t < world.numT; t++)
//			robots[num_of_robots++] = Circle(world.robotT[t].position, ROBOT_RADIUS);
//
//
//		double max_score = 0;
//		double max_posx = 0, max_posy = 0;
//		Circle pointrobot[MAX_ROBOTS_IN_THE_FIELD];
//		Circle goalpoint[MAX_ROBOTS_IN_THE_FIELD];
//		goalpoint[0] = Circle(Field::getGoalMidO(), ROBOT_RADIUS);
//
//
//
//		for (int i = 0; i < (int)(FieldLength / (2 * resolution)); i++)
//		{
//			for (int j = 0; j < (int)(FieldWidth / (1 * resolution)); j++)
//			{
//				score[i][j] = 0;
//				if ((point[i][j].getX() < (((FieldLength / 2) - PenaltyAreaWidth) - ROBOT_RADIUS) || point[i][j].getY() > ((PenaltyAreaLength / 2) + ROBOT_RADIUS)) || point[i][j].getY() < ((-PenaltyAreaLength / 2) - ROBOT_RADIUS))
//				{
//					///////////////////////////////////////////////////longest to goal
//					for (int z = 0; z < world.numO; z++)
//					{
//						Cone BallToGoal(world.robotO[z].position, Field::getUpBarP(), Field::getDownBarP());
//						pointrobot[0] = Circle(point[i][j], ROBOT_RADIUS);
//						if ((BallToGoal.Get_Free_Space_In_Cone(pointrobot, num_of_robots, Longest_Hole) > 0) && (Longest_Hole.Point_1.getDistanceTo(Longest_Hole.Point_2) < Field::getDownBarO().getDistanceTo(Field::getUpBarO())) && (world.robotO[z].position.getDistanceTo(Field::getGoalMidP()) < FieldLength / 3))
//						{
//							score[i][j] += 1 - HighLevel::rel(Field::getDownBarO().getDistanceTo(Field::getUpBarO()) - Longest_Hole.Point_1.getDistanceTo(Longest_Hole.Point_2), 0, PenaltyAreaLength);
//						}
//					}
//					score[i][j] = HighLevel::rel(score[i][j], 0, world.numO / 2);
//					/*		for (int z = 0; z < world.numO; z++)
//							{
//								score[i][j] += 2*(1 - HighLevel::rel(world.robotO[z].position.getDistanceTo(point[i][j]), 0, FieldLength/4));
//							}
//							score[i][j] = HighLevel::rel(score[i][j], 0, world.numO);*/
//
//					if (point[i][j].getX() < ((FieldLength / 2)))
//					{
//						score[i][j] += 3 * (1 - HighLevel::rel(point[i][j].getDistanceTo(Field::getGoalMidP()), 0, (sqrt(FieldLength*FieldLength + FieldWidth*FieldWidth) / 2)));
//					}
//					/////////////////////////////////////////////////////////
//
//					score[i][j] = HighLevel::rel(score[i][j], 0, 4);
//
//				}
//
//				if (max_score <= score[i][j])
//				{
//					max_posx = point[i][j].getX();
//					max_posy = point[i][j].getY();
//					max_score = score[i][j];
//				}
//
//				if ((u == 0))
//				{
//			//		DrawShape::DrawDot(point[i][j], 70, (score[i][j] > 0.75) ? 0 : ((score[i][j])) * 255, /*(score[i][j]>0.80) ? */ (score[i][j]) * 255/*:0*/, 0);
//				//	//DrawShape::DrawDot(max_score_position, 70, 255, 0, 0);
//				}
//
//			}
//
//		}
//
//		world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set == true;
//		for (int l = 0; l < number_of_defender; l++)
//		{
//			world.robotT[HighLevel::nearest_robot_to_point('T', VecPosition(max_posx, max_posy))].destination_position = VecPosition(max_posx, max_posy);
//			for (int i = 0; i < (int)(FieldLength / (2 * resolution)); i++)
//			{
//				for (int j = 0; j < (int)(FieldWidth / (1 * resolution)); j++)
//				{
//					if ((point[i][j].getX() < (((FieldLength / 2) - PenaltyAreaWidth) - ROBOT_RADIUS) || point[i][j].getY() > ((PenaltyAreaLength / 2) + ROBOT_RADIUS)) || point[i][j].getY() < ((-PenaltyAreaLength / 2) - ROBOT_RADIUS))
//					{
//						for (int z = 0; z < world.numO; z++)
//						{
//
//							Cone BallToGoal(world.robotO[z].position, Field::getUpBarP(), Field::getDownBarP());
//							pointrobot[num_of_robots] = Circle(VecPosition(max_posx, max_posy), ROBOT_RADIUS);
//							goalpoint[0] = Circle(point[i][j], ROBOT_RADIUS);
//							if ((BallToGoal.Get_Free_Space_In_Cone(goalpoint, 1, Longest_Hole) > 0) && (Longest_Hole.Point_1.getDistanceTo(Longest_Hole.Point_2) < Field::getDownBarO().getDistanceTo(Field::getUpBarO())))
//							{
//								if ((BallToGoal.Get_Free_Space_In_Cone(pointrobot, num_of_robots, Longest_Hole) > 0) && (Longest_Hole.Point_1.getDistanceTo(Longest_Hole.Point_2) < Field::getDownBarO().getDistanceTo(Field::getUpBarO()))/*&&(BallToGoal.Get_Free_Space_In_Cone(pointrobot, 1, Longest_Hole) > (BallToGoal.Get_Free_Space_In_Cone(goalpoint, 1, Longest_Hole_no_Robot)))*/)
//								{
//									Cone Longest_Hole_no_Robot(world.robotO[z].position, Longest_Hole.Point_1, Longest_Hole.Point_2);
//									double d = Longest_Hole.Point_1.getDistanceTo(Longest_Hole.Point_2);
//									if ((Longest_Hole_no_Robot.Get_Free_Space_In_Cone(goalpoint, 1, Longest_Hole) < 0) || (Longest_Hole.Point_1.getDistanceTo(Longest_Hole.Point_2) > d))
//									{
//										//		DrawShape::DrawDot(point[i][j], 70, 255, 25, 255);
//										score[i][j] = 0;
//									}
//
//								}
//
//							}
//
//						}
//
//
//						if (score[i][j] > 0)
//							score[i][j] -= (1 - HighLevel::rel(VecPosition(max_posx, max_posy).getDistanceTo(point[i][j]), 0, ROBOT_RADIUS * 5));
//						score[i][j] = HighLevel::rel(score[i][j], 0, 1);
//					}
//				}
//			}
//
//			num_of_robots++;
//			max_score = 0;
//			for (int i = 0; i < (int)(FieldLength / (4 * resolution)); i++)
//			{
//				for (int j = 0; j < (int)(FieldWidth / (2 * resolution)); j++)
//				{
//					if (max_score <= score[i][j])
//					{
//						max_score = score[i][j];
//						max_posx = point[i][j].getX();
//						max_posy = point[i][j].getY();
//					}
//				}
//			}
//
//		}
//	}
//
//}



void HighLevel::defence_scor2(int number_of_defender)
{
	double score[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	Cone::Hole_Type Longest_Hole;
	Circle robots[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	robots[0] = Circle(world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].position, ROBOT_RADIUS);
	int num_of_robots = 0;
	double max = 0;
	int num = 0;
	int out;
	int max_index;
	VecPosition point,pointup,pointfront,pointdown;
	Line getLeftParaline_RightPenaltyArea = Line::makeLineFromTwoPoints(Field::getUpLeft_RightPenaltyArea() - VecPosition (ROBOT_RADIUS+ DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE,-ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE), Field::getDownLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE));
	Line getUpParaline_RightPenaltyArea = Line::makeLineFromTwoPoints(Field::getUpLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE), Field::getUpRight_RightPenaltyArea() - VecPosition(0, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE));
	Line getDownParaline_RightPenaltyArea = Line::makeLineFromTwoPoints(Field::getDownLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE), Field::getDownRight_RightPenaltyArea() - VecPosition(0, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE));
	//world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].destination_set == true;
	if(world.numO!=0)
	for (int j = 0; j < number_of_defender; j++)
	{

		for (int z = 0; z < world.numO; z++)
		{
			Cone RobotOToGoal(world.robotO[z].position, Field::getUpBarP(), Field::getDownBarP());
			num = 0;
			out=RobotOToGoal.Get_Free_Space_In_Cone(robots, num_of_robots, Longest_Hole);
			if (out <= 0)
			{
				num++;
			}
			score[z] = HighLevel::rel((Longest_Hole.Point_1 - world.robotO[z].position).AngleBetween(Longest_Hole.Point_2 - world.robotO[z].position), 0, M_PI / 2);
			score[z] += 0.25*(1 - HighLevel::rel(world.robotO[z].position.getDistanceTo(Field::getGoalMidP()), 0, FieldLength / 2));
			score[z] = HighLevel::rel(score[z], 0, 1.25);
		}
		score[world.getIndexForRobotONumber(world.team_O.Goalie)] = 0;
		max = 0;
		for (int i = 0; i < world.numO; i++)
		{
			if (max < score[i])
			{
				max = score[i];
				max_index = i;
			}
		}

		//DrawShape::DrawDot(world.robotO[max_index].position, 255, 255, 0);
		Cone RobotOToGoal(world.robotO[max_index].position, Field::getUpBarP(), Field::getDownBarP());
		RobotOToGoal.Get_Free_Space_In_Cone(robots, num_of_robots, Longest_Hole);
		Line robotO_to_Goal = Line::makeLineFromTwoPoints(world.robotO[max_index].position, ((Longest_Hole.Point_1 + Longest_Hole.Point_2) / 2));
		pointup= robotO_to_Goal.getIntersection(getUpParaline_RightPenaltyArea);
		pointfront = robotO_to_Goal.getIntersection(getLeftParaline_RightPenaltyArea);
		pointdown= robotO_to_Goal.getIntersection(getDownParaline_RightPenaltyArea);

		if (pointfront.getX() == (Field::getUpLeft_RightPenaltyArea()- VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getX() && pointfront.getY() <= (Field::getUpLeft_RightPenaltyArea()- VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getY() && pointfront.getY() >= (Field::getDownLeft_RightPenaltyArea()- VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getY())
		{
			point = pointfront;
		}
		else if (pointup.getX() >= (Field::getUpLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getX() && pointup.getX() <= (Field::getUpRight_RightPenaltyArea() - VecPosition(0, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getX() && pointup.getY() == (Field::getUpLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getY())
		{
			point = pointup;
		}
		else if (pointdown.getX() >= (Field::getDownLeft_RightPenaltyArea()- VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getX() && pointdown.getX() <= (Field::getDownRight_RightPenaltyArea() - VecPosition(0, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getX() && pointdown.getY() == (Field::getDownLeft_RightPenaltyArea()- VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE)).getY())
		{
			point = pointdown;
		}

		/*if (world.robotO[max_index].position.getX()<Field::getUpLeft_RightPenaltyArea().getX())
		{
			point = robotO_to_Goal.getIntersection(getLeftParaline_RightPenaltyArea);
		}
		else if(world.robotO[max_index].position.getY()>0)
		{
			point = robotO_to_Goal.getIntersection(getUpParaline_RightPenaltyArea);
		}
		else
		{
			point = robotO_to_Goal.getIntersection(getDownParaline_RightPenaltyArea);
		}*/

		/*Line Ball_to_Goal = Line::makeLineFromTwoPoints(Field::getGoalMidO(), world.ball.getCurrentBallPosition());
		if (num == world.numO)
		{
			if (world.robotO[max_index].position.getX()<Field::getUpLeft_RightPenaltyArea().getX())
			{
				point = Ball_to_Goal.getIntersection(getLeftParaline_RightPenaltyArea);
			}
			else if (world.robotO[max_index].position.getY()>0)
			{
				point = Ball_to_Goal.getIntersection(getUpParaline_RightPenaltyArea);
			}
			else
			{
				point = Ball_to_Goal.getIntersection(getDownParaline_RightPenaltyArea);
			}
		}*/
		//DrawShape::DrawDot(point,50, 255, 0, 255);
		world.robotT[HighLevel::nearest_robot_to_point('T', point)].destination_position = point;
		robots[num_of_robots++] = Circle(point, ROBOT_RADIUS);

	}
	//DrawShape::ClearCircles();
}


void HighLevel::find_roboto_pass(int number_of_robott_block_roboto)
{
	double score[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	double number[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	Cone::Hole_Type Longest_Hole;
	Circle robots[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
	robots[0] = Circle(world.robotT[world.getIndexForRobotTNumber(world.team_T.Goalie)].position, ROBOT_RADIUS);
	int num_of_robots = 0;
	double max = 0;
	int max_index;
	VecPosition point, pointup, pointfront, pointdown, intersection1, intersection2;
	Line getLeftParaline_RightPenaltyArea = Line::makeLineFromTwoPoints(Field::getUpLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE), Field::getDownLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE));
	Line getUpParaline_RightPenaltyArea = Line::makeLineFromTwoPoints(Field::getUpLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE), Field::getUpRight_RightPenaltyArea() - VecPosition(0, -ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE));
	Line getDownParaline_RightPenaltyArea = Line::makeLineFromTwoPoints(Field::getDownLeft_RightPenaltyArea() - VecPosition(ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE), Field::getDownRight_RightPenaltyArea() - VecPosition(0, ROBOT_RADIUS + DISTANCE_TO_PENALTY_AREA_FOR_TEAMMATE));

	for (int z = 0; z < world.numO; z++)
	{
		if (world.robotO[z].position.getDistanceTo(world.ball.getCurrentBallPosition()) > DISTANCE_TO_BALL_IN_STOP_MODE + 2*ROBOT_RADIUS)
		{
			Cone RobotOToGoal(world.robotO[z].position, Field::getUpBarP(), Field::getDownBarP());
			RobotOToGoal.Get_Free_Space_In_Cone(robots, num_of_robots, Longest_Hole);
			score[z] = HighLevel::rel((Longest_Hole.Point_1 - world.robotO[z].position).AngleBetween(Longest_Hole.Point_2 - world.robotO[z].position), 0, M_PI / 2);
			score[z] += 0.25*(1 - HighLevel::rel(world.robotO[z].position.getDistanceTo(Field::getGoalMidP()), 0, FieldLength / 2));
			score[z] += 0.5*(1 - HighLevel::rel(world.robotO[z].position.getDistanceTo(world.ball.getCurrentBallPosition()), 0, FieldLength / 2));
			score[z] = HighLevel::rel(score[z], 0, 1.75);
		}
		else
		{
			score[z] = 0;
		}
	}
	score[world.getIndexForRobotONumber(world.team_O.Goalie)] = 0;
	max = 0;
	for (int j = 0; j < number_of_robott_block_roboto; j++)
	{
		for (int i = 0; i < world.numO; i++)
		{
			if (max < score[i])
			{
				max = score[i];
				max_index = i;
			}
		}
		max = 0;
		score[max_index] = 0;
		//cout << world.getRobotONumberForIndex(max_index)<<endl;
		Line ball_to_robotO = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(), world.robotO[max_index].position);
		Circle roboto = Circle(world.robotO[max_index].position, 2 * ROBOT_RADIUS + BALL_RADIUS);
		ball_to_robotO.getCircleIntersectionPoints(roboto, &intersection1, &intersection2);
		if (world.ball.getCurrentBallPosition().getDistanceTo(intersection1) < world.ball.getCurrentBallPosition().getDistanceTo(intersection2))
			world.robotT[HighLevel::nearest_robot_to_point('T', intersection1)].destination_position = intersection1;
		else
			world.robotT[HighLevel::nearest_robot_to_point('T', intersection2)].destination_position = intersection2;
	}
}


void HighLevel::set_last_destination_set()
{
	for (int i = 0; i < world.numT; i++)
	{
		last_destination_robotT[i] = world.robotT[i].destination_set;
	}
}


void HighLevel::check_last_destination_set(int index_robotT,VecPosition destination)
{
	if (destination.getDistanceTo(last_destination_robotT[index_robotT]) < ROBOT_RADIUS)
		world.robotT[index_robotT].destination_position = last_destination_robotT[index_robotT];
	else
		world.robotT[index_robotT].destination_position = destination;
}


void HighLevel::ownership_ball()
{
	if (HighLevel::find_robot_have_ball('O') != -1)
	{
		HighLevel::play_mode = opponent;
		sender = HighLevel::find_robot_have_ball('O');

	}
	else if (HighLevel::find_robot_have_ball('T') != -1)
	{
		HighLevel::play_mode = teammate;
	}
	else
	{
		for (int i = 0; i < world.numT; i++)
		{
			if (HighLevel::pass_mode != expectation && ((world.ball.getVelocity().getX() > 10 || world.ball.getVelocity().getY() > 10) && (abs(world.ball.getVelocity().AngleBetween(world.robotT[i].position)) < 1.2 || M_PI - abs(world.ball.getVelocity().AngleBetween(world.robotT[i].position)) < 1.2)))
			{
				HighLevel::play_mode = ex;
				if (sender != i)
					reciver = i;
			}
		}
	}
	if (HighLevel::play_mode == ex)
	{
		if (HighLevel::block_the_ball_to_point(world.robotO[sender].position, world.robotO[reciver].position))
		{
			HighLevel::play_mode = teammate;
		}
	}
	//cout << HighLevel::play_mode << endl;
}

int HighLevel::block_the_ball_to_point(VecPosition send, VecPosition recive)
{
	Paraline send_to_recive = Paraline(send, recive);
	VecPosition dest;
	double velocity;
	double time_ball_to_dest;
	double time_robotT_to_dest;
	bool can=false;
	for (int i = 0; i < world.numT; i++)
	{
		dest = send_to_recive.getPointOnParalineClosestTo(world.robotT[i].position);
		if ((dest.getDistanceTo(send) > ROBOT_RADIUS * 2 && dest.getDistanceTo(recive) > ROBOT_RADIUS * 2)&& !Balk::isInPenaltyArea(recive))
		{
			velocity = sqrt(pow(world.ball.velocity.getX(), 2) + pow(world.ball.velocity.getY(), 2));
			time_ball_to_dest = dest.getDistanceTo(world.ball.getCurrentBallPosition()) / velocity;
			time_robotT_to_dest = dest.getDistanceTo(world.robotT[i].position) / MAX_ROBOT_SPEED;
			if ((time_ball_to_dest>time_robotT_to_dest) && (world.getIndexForRobotTNumber(world.team_T.Goalie) != i))
			{
				world.robotT[i].destination_set = true;
				world.robotT[i].destination_position = dest;
				can = true;
			}
		}
	}
	return can;
}


void HighLevel::defence_hol_robotO(int robotO_index, int number_of_defender)
{
	if (robotO_index == -1)
	{
		robotO_index = reciver;
	}
	HighLevel::defence_format();
	VecPosition q1, q2, dest_goal, dest_reciver;
	Line goal_to_robotO = Line::makeLineFromTwoPoints(Field::getGoalMidP(), world.robotO[robotO_index].position);
	Circle robotO = Circle(world.robotO[robotO_index].position, ROBOT_RADIUS * 2);
	goal_to_robotO.getCircleIntersectionPoints(robotO, &q1, &q2);
	if (Field::getGoalMidP().getDistanceTo(q1) > Field::getGoalMidP().getDistanceTo(q2))
	{
		dest_goal = q2;
	}
	else
	{
		dest_goal = q1;
	}
	world.robotT[HighLevel::nearest_robot_to_point('T', dest_goal)].destination_position = dest_goal;



	int danger_pass = (danger_robotO[0] == robotO_index) ? danger_robotO[1] : danger_robotO[0];
	VecPosition robot_reciver = world.robotO[danger_pass].position;
	Line robotO_to_robotO = Line::makeLineFromTwoPoints(robot_reciver, world.robotO[robotO_index].position);
	if (number_of_defender == 2)
	{
		for (int i = 1; i < world.numO; i++)
		{
			danger_pass = (danger_robotO[i] == robotO_index) ? danger_robotO[i] : danger_robotO[i - 1];
			robot_reciver = world.robotO[danger_pass].position;
			robotO_to_robotO = Line::makeLineFromTwoPoints(robot_reciver, world.robotO[robotO_index].position);
			robotO_to_robotO.getCircleIntersectionPoints(robotO, &q1, &q2);
			if (dest_goal.getDistanceTo(q1) > 2 * ROBOT_RADIUS && dest_goal.getDistanceTo(q2) > 2 * ROBOT_RADIUS)
			{
				i = world.numO;
			}
		}
		if (robot_reciver.getDistanceTo(q1) > robot_reciver.getDistanceTo(q2))
		{
			dest_reciver = q2;
		}
		else
		{
			dest_reciver = q1;
		}
		//DrawShape::DrawDot(robot_reciver, 85, 255, 0, 0);
		world.robotT[HighLevel::nearest_robot_to_point('T', dest_reciver)].destination_position = dest_reciver;
	}

}

///////////////////////////////happy new term
void HighLevel::robot_t_can_pass(int robot_hav_ball)
{

}

/*
گراف بین ربات ها و دروازه
Line roboto2roboto, roboto2goal;
Circle robot;
bool space = false;
VecPosition q1, q2;
double danger_importance_angle[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
double danger_importance_pass[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
double danger_importance_distance[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
double danger_importance_goal[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
for (int i = 0; i <= world.numO; i++)
{

for (int j = i + 1; j < world.numO; j++)
{
space = true;
roboto2roboto = Line::makeLineFromTwoPoints(world.robotO[i].position, world.robotO[j].position);
for (int z =0 ; z < world.numT; z++)
{
robot = Circle(world.robotT[z].position, ROBOT_RADIUS);
roboto2roboto.getCircleIntersectionPoints(robot, &q1, &q2);
if (q1 != q2)
{
space = false;
}
}
if (space)
{
danger_importance_angle[i] = 1;
danger_importance_angle[j] = 1;
DrawShape::DrawLine(world.robotO[i].position, world.robotO[j].position);
}
else
{
danger_importance_angle[i] = 0;
danger_importance_angle[j] = 0;
}
}



space = true;
roboto2goal = Line::makeLineFromTwoPoints(world.robotO[i].position, Field::getGoalMidP());
for (int z = 0; z < world.numT; z++)
{
robot = Circle(world.robotT[z].position, ROBOT_RADIUS);
roboto2goal.getCircleIntersectionPoints(robot, &q1, &q2);
if (q1 != q2)
{
space = false;
}
}
if (space)
{
danger_importance_goal[i] = 1;
DrawShape::DrawLine(world.robotO[i].position, Field::getGoalMidP());
}
else
{
}
}
*/

//***************************************************MohammadHossein Z************************************//









//////estimation next position
//void HighLevel::EstimationPosition(World &world)
//{
//	int id;
//	VecPosition VT[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD][Max_Time_Estimation];
//	VecPosition AT[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD][Max_Time_Estimation];
//	for (id = 0; id <= MAX_ROBOTS_PER_TEAM_IN_THE_FIELD; id++)
//	{
//		//		VT[id][time] = world.robotT[id].velocity;
//		//		AT[id][time - 1] = VT[id][time] - VT[id][time - 1];
//	}
//
//
//}
//

///////learning about location oppoment robot
/*
void HighLevel::Scoring_Situations(World &world)
{
int id;
int longtest;
Cone::Hole_Type c;

if (world.robotO[id].position.getX > 0)
{
Cone test(world.robotO[id].position, Field::getUpBarP(), Field::getDownBarP());
//	test.Get_Free_Space_In_Cone(world, test, , , );
//	longtest.point1
}





//	if (time > t)
//	t++;
//	if( oppoment keap the ball)
for (id = 0; id < MAX_ROBOT_O; id++)
{
if (world.robotO[id].position.getX > 0 && world.robotO[id].position.getY > 0)
UpR_Atack[int(world.robotO[id].position.getX / (FieldLength / Length_division))][int(world.robotO[id].position.getY / (FieldWidth / Width_division))]++;
else if (world.robotO[id].position.getX < 0 && world.robotO[id].position.getY > 0)
UpL_Atack[int(-1 * world.robotO[id].position.getX / (FieldLength / Length_division))][int(world.robotO[id].position.getY / (FieldWidth / Width_division))]++;
else if (world.robotO[id].position.getX > 0 && world.robotO[id].position.getY < 0)
DownR_Atack[int(world.robotO[id].position.getX / (FieldLength / Length_division))][int(-1 * world.robotO[id].position.getY / (FieldWidth / Width_division))]++;
else if (world.robotO[id].position.getX < 0 && world.robotO[id].position.getY < 0)
UpR_Atack[int(-1 * world.robotO[id].position.getX / (FieldLength / Length_division))][int(-1 * world.robotO[id].position.getY / (FieldWidth / Width_division))]++;
}
//	else
for (id = 0; id < MAX_ROBOT_O; id++)
{
if (world.robotO[id].position.getX > 0 && world.robotO[id].position.getY > 0)
UpR_Defence[int(world.robotO[id].position.getX / (FieldLength / Length_division))][int(world.robotO[id].position.getY / (FieldWidth / Width_division))]++;
else if (world.robotO[id].position.getX < 0 && world.robotO[id].position.getY > 0)
UpL_Defence[int(-1 * world.robotO[id].position.getX / (FieldLength / Length_division))][int(world.robotO[id].position.getY / (FieldWidth / Width_division))]++;
else if (world.robotO[id].position.getX > 0 && world.robotO[id].position.getY < 0)
DownR_Defence[int(world.robotO[id].position.getX / (FieldLength / Length_division))][int(-1 * world.robotO[id].position.getY / (FieldWidth / Width_division))]++;
else if (world.robotO[id].position.getX < 0 && world.robotO[id].position.getY < 0)
UpR_Defence[int(-1 * world.robotO[id].position.getX / (FieldLength / Length_division))][int(-1 * world.robotO[id].position.getY / (FieldWidth / Width_division))]++;
}
//	endif
}


*/
//////////////////////////////////


////// SMMSS


//void HighLevel::Overlap() {
//
//	int cursor=-1;
//	int overlapper_1 = -1;
//	int overlapper_2 = -1;
//	int overlapper_main = -1;
//
//	VecPosition overlap_pos_1=0;
//	VecPosition overlap_pos_2 = 0;
//
//	cursor = find_robot_have_ball('T');
//
//	overlap_pos_1 = world.robotT[cursor].position.getY()+0.05*FieldWidth;
//	overlap_pos_2 = world.robotT[cursor].position.getY()-0.05*FieldWidth;
//
//	overlapper_1 = nearest_robot_to_point('T', overlap_pos_1 );
//	overlapper_2 = nearest_robot_to_point('T', overlap_pos_2);
//
//	overlapper_main = overlapper_1;
//
//	if (overlapper_2 > overlapper_1) {
//
//		overlapper_main = overlapper_2;
//
//	}
//
//	if (cursor != -1) {
//
//	//	world
//
//	}


//}