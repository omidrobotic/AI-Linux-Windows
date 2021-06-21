#include "world.h"
#include "RRT.h"
#include "MotionControl.h"
#include "Switches.h"
#include "HighLevel.h"
int y = 0;
//Strategy
int indexOfNearestRobot;
void produceRobotsDestinations()
{
#if GAME_MODE_ROBOCUP_2021==1

    Balk::set_balks_in_world_object();
	//auto start_timeM = std::chrono::high_resolution_clock::now();
	///set robots to line
	//DrawShape::ClearCircles();
	//DrawShape::ClearLines();
	//for (int i = 0; i < world.numT; i++)
	//{		
	//	world.robotT[i].destination_position = VecPosition(world.mouseX, world.mouseY);// + (pow(-1,i)*floor((i+1)/2) * 500) );
	//	world.robotT[i].destination_angle = world.robotT[i].position.getAngleToward(VecPosition(world.mouseX, world.mouseY));
	//	//cout << world.robotT[i].destination_angle << endl;
	//	//world.robotT[i].shoot_or_chip = 1;
	//	//world.robotT[i].kick_power = 7;
	//	//DrawShape::DrawDot(world.robotT[i].destination_position);
	//	//DrawShape::DrawDot(world.robotT[i].position,20,0,0,255);
	//	//DrawShape::DrawParaline(world.robotT[i].position,Line::makeLineFromPositionAndAngle(world.robotT[i].position,Rad2Deg( world.robotT[i].angle)).getPointOnLineClosestTo(VecPosition(world.mouseX, world.mouseY)),0,0,255);
	//	//cout << world.robotT[i].angle << endl;
	//}
	//world.robotT[world.getIndexForRobotTNumber(2)].destination_position = VecPosition(world.mouseX, world.mouseY);






	for (int i = 0; i < MAX_ROBOTS_PER_TEAM_IN_THE_FIELD; i++)
	{
		world.robotT[i].destination_set = false;
		//world.robotT[i].destination_position = world.robotT[i].position;
	}
	for (int i = 0; i <= world.numT; i++)
	{
		VecPosition robot1_to_robot2 = world.ball.getCurrentBallPosition() - world.robotT[i].position;
		double angle_robot_to_ball = -((robot1_to_robot2).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[i].position.getY() - world.ball.getCurrentBallPosition().getY());
		if (angle_robot_to_ball - world.robotT[i].angle<10 * CORRECTION_FACTOR && angle_robot_to_ball - world.robotT[i].angle>-10 * CORRECTION_FACTOR)
			world.robotT[i].ballBalkMode = Ball::BalkMode::notBalk;
		else
			world.robotT[i].ballBalkMode = Ball::BalkMode::balk;
		world.robotT[i].spinBack = false;

	}

//HighLevel::move_ball_to_position(1,VecPosition(1000,1000));
	  


	//HighLevel::DefenceCutShoot(3, x);
//	HighLevel::GoaliHoleCover();
	//HighLevel::BlockOponent(1, x, 1);
	//HighLevel::BlockOponent(2, x,2);
	//	world.robotT[0].destination_position = world.robotT[0].position;
	//world.robotT[1].destination_position = world.robotT[1].position;




//	HighLevel::several_position_Line(VecPosition(((FieldLength / 2) - (PenaltyAreaWidth)), 0), ((world.numT - (int)((world.numT - 2) / 3)) - 1), x);
	//HighLevel::catch_the_ball_2point(world, VecPosition(0, 0), 0, 1);
	//if(HighLevel::find_robot_have_ball('T')!=-1)
	//HighLevel::catch_the_ball_2point(world, VecPosition(0, 0), 0, 1);
	/*HighLevel::Shoot(0);
	world.robotT[0].shoot_or_chip = 0;
	world.robotT[0].kick_power = 1;*/
//	HighLevel::plan_scor(4);
//	HighLevel::defence_scor2(5);
//	HighLevel::GoaliHoleCover();
	//HighLevel::catch_the_ball_2point(world, VecPosition (0,0), 0, 1);
	//HighLevel::plan_scor(4);
//	DrawShape::DrawDot(Field::getDownParaline_LeftPenaltyArea(), 70, 255, 0, 0);
	//HighLevel::GoaliHoleCover();
	//
	//HighLevel::Ready(world, 0);
	//HighLevel::plan_scor(3);

	//HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));
	//else
	//	HighLevel::Pass(0, 1);
	//HighLevel::Shoot(0);
	/*world.robotT[0].destination_angle = M_PI;
	world.robotT[0].destination_position = VecPosition(0, FieldWidth / 4);
	world.robotT[0].shoot_or_chip = 1;
	world.robotT[0].kick_power = 2;*/
	//DrawShape::ClearCircles();
	//HighLevel::Block(3, x);
	//HighLevel::Block(world, 3);
	//	HighLevel::StopSurrounding(world,1,2);




    // Begin Farhan Daemi

    //HighLevel::turn_all_spinbacks_on();

//    HighLevel::goalKeeper_defend_and_pass(GOALI_NUMBER);
//    HighLevel::forward_formation(4);
//    HighLevel::defence_formation(2);
//	HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));




        // End Farhan Daemi



//	HighLevel::GoaliHoleCover();
//	HighLevel::plan_scor(4);
//	HighLevel::defence_scor2(2);
//	HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));
	//HighLevel::defence_format(8);

    for (int i = 0; i <= world.numT; ++i) {
        if (world.robotT[i].kick_power>0)
        {
           world.robotT[i].spinBack= false;
        } else
        {
            world.robotT[i].spinBack= true;
        }
    }




/*	switch (world.playMode)
	{
	    case mode_State::ballPlacement:
	        indexOfNearestRobot = HighLevel::nearest_robot_to_ball('T');
            for (int i = 0; i <= world.numT; i++)
            {
                if (indexOfNearestRobot==i)
                    HighLevel::move_ball_to_position(world.getRobotTNumberForIndex(indexOfNearestRobot),
                                                     world.team_T.Set_Refree_Ball_Position);
                else
                    world.robotT[i].destination_position=world.robotT[i].position;
            }
            world.robotT[indexOfNearestRobot].kick_power=0;
	        break;
    case mode_State::Stop:
        switch (world.kickMode)
        {
        case mode_State::KickMode::NoKickMode:
            HighLevel::GoaliHoleCover();
            HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
            HighLevel::defence_scor2((world.numT - 2)-int(PRESENT_OF_ATTACKER*(world.numT - 2)));
            HighLevel::find_roboto_pass(int(PRESENT_OF_ATTACKER*(world.numT - 2)));
            break;

		case mode_State::KickMode::KickOffOPrepare:
			HighLevel::start_robotT_format_NoKickMode("KickOffOPrepare");
			break;

		case mode_State::KickMode::KickOffTPrepare:
			HighLevel::start_robotT_format_NoKickMode("KickOffTPrepare");
			HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));


			break;

		case mode_State::KickMode::PenaltyTPrepare:
			HighLevel::start_robotT_format_penaltyt(world,1);
			break;

		case mode_State::KickMode::PenaltyOPrepare:
			HighLevel::start_robotT_format_penaltyo(world);
			break;
		default:
			break;
		}
		break;


	case mode_State::Wait:
		switch (world.kickMode)
		{
		case mode_State::KickMode::KickOffO:
			HighLevel::start_robotT_format_NoKickMode("KickOffO");
			break;

		case mode_State::KickMode::KickOffT:
			HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));
		//	HighLevel::start_robotT_format_NoKickMode("KickOffT");
			break;

		case mode_State::KickMode::PenaltyO:
			HighLevel::start_robotT_format_penaltyo(world);
			break;

		case mode_State::KickMode::PenaltyT:
			HighLevel::Shoot(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
			HighLevel::start_robotT_format_penaltyt(world, 1);
			break;

		*//*case mode_State::KickMode::NoKickMode:
			HighLevel::start_robotT_format_NoKickMode("NoKickMode");
			break;*//*

		case mode_State::KickMode::DirectFreeKickT:
			HighLevel::GoaliHoleCover();
			HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
			HighLevel::defence_scor2((world.numT - 2) - int((world.numT - 2)*PRESENT_OF_ATTACKER));
			HighLevel::direct_free_kick(int(PRESENT_OF_ATTACKER*(world.numT - 2))+1, 0);
			break;

		case mode_State::KickMode::DirectFreeKickO:
			HighLevel::GoaliHoleCover();
			//HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
			HighLevel::defence_scor2((world.numT - 2)-int(PRESENT_OF_ATTACKER*(world.numT - 2)));
			HighLevel::find_roboto_pass(int(PRESENT_OF_ATTACKER*(world.numT - 2)));
			break;

		case mode_State::KickMode::IndirectFreeKickT:
			HighLevel::GoaliHoleCover();
			HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
			HighLevel::defence_scor2((world.numT - 2) - int((world.numT - 2)*PRESENT_OF_ATTACKER));
			HighLevel::direct_free_kick(int(PRESENT_OF_ATTACKER*(world.numT - 2)) + 1, 0);
			break;

		case mode_State::KickMode::IndirectFreeKickO:
			HighLevel::GoaliHoleCover();
			HighLevel::find_roboto_pass(int(PRESENT_OF_ATTACKER*(world.numT - 2)));
			HighLevel::defence_scor2((world.numT - 2)- int(PRESENT_OF_ATTACKER*(world.numT - 2)));
			//HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
			break;
		default:
			break;
		}
		break;

	case mode_State::Play:
		HighLevel::ownership_ball();
		//world.setKickMode(mode_State::KickMode::NoKickMode);
	//	if (HighLevel::play_mode == teammate)
	//	{

	//	}
		*//*else if (HighLevel::play_mode== opponent)
		{
			HighLevel::GoaliHoleCover();
			HighLevel::defence_hol_robotO(1, HighLevel::find_robot_have_ball('O'));
			HighLevel::defence_scor2(int((world.numT-2)*0.67));
			HighLevel::find_roboto_pass(world.numT-(int((world.numT - 2) * 0.67)+2));

		}
		else
		{
			HighLevel::GoaliHoleCover();
			HighLevel::defence_hol_robotO(-1,1);
			HighLevel::defence_scor2(int((world.numT - 2) * 0.67));
			HighLevel::find_roboto_pass(world.numT - (int((world.numT - 2) * 0.67) + 2));
		}*//*
		switch (world.kickMode)
		{
		case mode_State::KickMode::KickOffO:
			HighLevel::start_robotT_format_NoKickMode("KickOffO");
			break;

		case mode_State::KickMode::KickOffT:
			HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
			break;

		case mode_State::KickMode::PenaltyO:
			break;

		case mode_State::KickMode::PenaltyT:
			break;

		case mode_State::KickMode::NoKickMode:

			*//*HighLevel::GoaliHoleCover();
			HighLevel::plan_scor(4);
			HighLevel::defence_scor2(2);
			HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));*//*
            HighLevel::GoaliHoleCover();
                HighLevel::plan_scor(int(PRESENT_OF_ATTACKER*(world.numT - 2)));

                if(world.ball.getCurrentBallPosition().getDistanceTo(Field::getGoalMidO())<2500 && HighLevel::find_robot_have_ball('O')==-1)
            {
                HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
            } else {
                HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));
            }
            //cout << world.getRobotTNumberForIndex(uyu) << endl;
            HighLevel::defence_scor2((world.numT - 2) - int(PRESENT_OF_ATTACKER*(world.numT - 2)));

			break;

		case mode_State::KickMode::DirectFreeKickT:

			HighLevel::start_robotT_format_NoKickMode("DirectFreeKickT");
			HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
			break;

		case mode_State::KickMode::DirectFreeKickO:
			HighLevel::start_robotT_format_NoKickMode("DirectFreeKickO");
			break;

		case mode_State::KickMode::IndirectFreeKickT:
			HighLevel::start_robotT_format_NoKickMode("IndirectFreeKickT");
			HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
			break;

		case mode_State::KickMode::IndirectFreeKickO:
			HighLevel::start_robotT_format_NoKickMode("IndirectFreeKickO");
			break;

		default:
			break;
		}
		break;


	case mode_State::Halt:
		for (int i = 0; i < MAX_ROBOTS_PER_TEAM_IN_THE_FIELD; i++)
		{
			world.robotT[i].velocityToGo = VecPosition(0, 0);
			world.robotT[i].wToGo = 0;
		}
		break;

	case mode_State::Timeout:
		HighLevel::time_out();
		break;

		default:
        break;
	}*/

	//set last destination
	HighLevel::set_last_destination_set();

	//auto end_timeM = std::chrono::high_resolution_clock::now();
	//auto timeM = end_timeM - start_timeM;
	//cout << timeM.count() / 1000000.0 << endl;
#elif GAME_MODE_ROBOCUP_2021==2
#endif
    }
	

//RRT
void producePathsToDestinations()
{
	
	for (int i = 0; i < world.numT; i++)
	{	
		if (rrt.MakeRRT(world.robotT[i].id, world.robotT[i].destination_position, world.robotT[i].pathToDestination, world.robotT[i].sizeOfPathToDestination, world.robotT[i].isRobotTBalk, world.robotT[i].isRobotOBalk, ((world.robotT[i].id == world.team_T.Goalie) ? false : true), world.robotT[i].ballBalkMode) == RRT::RRT_result::path_not_found);
		//	cout << "path for robot with id " << world.robotT[i].id << " not found" << endl;
			
	}
}

//Motion Conctrol
void produceSpeedOfRobots()
{

#if SEND_COMMANDS_TO_ROBOTS == 1
	SpeedDiagram sd;
	int rnfi;
	for (int i = 0; i < world.numT; i++)
	{
		rnfi = world.getRobotTNumberForIndex(i);
		world.robotT[i].velocityToGo = sd.genVecVel_forNrf(rnfi, world.robotT[i].pathToDestination[world.robotT[i].sizeOfPathToDestination - 1]);
		world.robotT[i].wToGo = sd.genVecW_forNrf(rnfi, world.robotT[i].destination_angle);
		//cout << world.robotT[i].wToGo << endl;
		
	}
#elif SEND_COMMANDS_TO_ROBOTS == 0
	SpeedDiagram sd;
	int rnfi;
	for (int i = 0; i < world.numT; i++)
	{
		rnfi = world.getRobotTNumberForIndex(i);
		world.robotT[i].velocityToGo = sd.genVecVel_forGrsim(rnfi, world.robotT[i].pathToDestination[world.robotT[i].sizeOfPathToDestination-1]);
		world.robotT[i].wToGo = sd.genVecW_forGrsim(rnfi, world.robotT[i].destination_angle);
	}
#elif SEND_COMMANDS_TO_ROBOTS==2
    SpeedDiagram sd;
    int rnfi;
    for (int i = 0; i < world.numT; i++)
    {
        rnfi = world.getRobotTNumberForIndex(i);
        world.robotT[i].velocityToGo = sd.genVecVel_forGrsim(rnfi, world.robotT[i].pathToDestination[world.robotT[i].sizeOfPathToDestination-1]);
        world.robotT[i].wToGo = sd.genVecW_forGrsim(rnfi, world.robotT[i].destination_angle);        //cout << world.robotT[i].wToGo << endl;

    }
#endif
}