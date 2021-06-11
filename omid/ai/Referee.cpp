#include "Referee.h"
#include <iostream>
#include "Switches.h"


Refree::Refree(void)
{

}
void Refree::recive_Init(void)
{
	
	refree_udp.Init_Socket_Client(GROUP_ADDR_Refree, PORT_NUM_Refree);
	
}
void Refree::Refree_parser(World &world)
{
	
	///get information from udp and save it into buffer_recive of packet
	///return size of data
	int ct = refree_udp.recive();
	//std::cout << "\n refree ln:" << ct;
	//std::cout << "\n refree \n" << refree_udp.buffer_recive;

	/// Parse a protocol buffer(buffer_recive) contained in an array of bytes.
	if (packet.ParseFromArray(refree_udp.buffer_recive, ct) < 0)
	{
		std::cout << "????refree????";
		return;
	}
	
	if (world.getInstance().team_color ==TC_Blue)///_____make it in world 
	{
		world.team_T.Goalie = packet.blue().goalie();
		world.team_T.Yellow_Cards = packet.blue().yellow_cards();
		world.team_T.Yellow_Cards_Times_Size = packet.blue().yellow_card_times_size();
		for (int i = 0; packet.blue().yellow_card_times_size() > i;i++)
		{
			/// The amount of time (in microseconds) left on each yellow card issued to the team.
			/// If no yellow cards are issued, this array has no elements.
			/// Otherwise, times are ordered from smallest to largest.
			world.team_T.Yellow_Card_Times[i] = packet.blue().yellow_card_times(i);
			/*std::cout << "\n " << i << ":" << Blue_Team_Info.Yellow_Card_Times[i];*/
		}
		world.team_T.Red_Cards = packet.blue().red_cards();
		world.team_T.Timeouts = packet.blue().timeouts();
		world.team_T.Timeout_Time = packet.blue().timeout_time();
		world.team_T.Score = packet.blue().score();


		world.team_O.Goalie = packet.yellow().goalie();
		world.team_O.Yellow_Cards = packet.yellow().yellow_cards();
		world.team_O.Yellow_Cards_Times_Size = packet.yellow().yellow_card_times_size();
		for (int i = 0;packet.yellow().yellow_card_times_size() > i;i++)
		{
			world.team_O.Yellow_Card_Times[i] = packet.yellow().yellow_card_times(i);
			/*std::cout << "\n " << i << ":" << Yellow_Team_Info.Yellow_Card_Times[i];*/
		}
		world.team_O.Red_Cards = packet.yellow().red_cards();
		world.team_O.Timeouts = packet.yellow().timeouts();
		world.team_O.Timeout_Time = packet.yellow().timeout_time();
		world.team_O.Score = packet.yellow().score();
	}
	else if(world.getInstance().team_color == TC_Yellow)
	{
		world.team_O.Goalie = packet.blue().goalie();
		world.team_O.Yellow_Cards = packet.blue().yellow_cards();
		world.team_O.Yellow_Cards_Times_Size = packet.blue().yellow_card_times_size();
		for (int i = 0;packet.blue().yellow_card_times_size() > i;i++)
		{
			world.team_O.Yellow_Card_Times[i] = packet.blue().yellow_card_times(i);
			/*	std::cout << "\n " << i << ":" << Blue_Team_Info.Yellow_Card_Times[i];*/
		}
		world.team_O.Red_Cards = packet.blue().red_cards();
		world.team_O.Timeouts = packet.blue().timeouts();
		world.team_O.Timeout_Time = packet.blue().timeout_time();
		world.team_O.Score = packet.blue().score();


		world.team_T.Goalie = packet.yellow().goalie();
		world.team_T.Yellow_Cards = packet.yellow().yellow_cards();
		world.team_T.Yellow_Cards_Times_Size = packet.yellow().yellow_card_times_size();
		for (int i = 0;packet.yellow().yellow_card_times_size() > i;i++)
		{
			world.team_T.Yellow_Card_Times[i] = packet.yellow().yellow_card_times(i);
			/*std::cout << "\n " << i << ":" << Yellow_Team_Info.Yellow_Card_Times[i];*/
		}
		world.team_T.Red_Cards = packet.yellow().red_cards();
		world.team_T.Timeouts = packet.yellow().timeouts();
		world.team_T.Timeout_Time = packet.yellow().timeout_time();
		world.team_T.Score = packet.yellow().score();
	}

	/// The number of microseconds left in the stage.
	/// The following stages have this value; the rest do not:
	/// NORMAL_FIRST_HALF
	/// NORMAL_HALF_TIME
	/// NORMAL_SECOND_HALF
	/// EXTRA_TIME_BREAK
	/// EXTRA_FIRST_HALF
	/// EXTRA_HALF_TIME
	/// EXTRA_SECOND_HALF
	/// PENALTY_SHOOTOUT_BREAK
	///
	/// If the stage runs over its specified time, this value
	/// becomes negative.
	stage_time_left = packet.stage_time_left();

	//// TODO: any use for the timestamps? dont need 

	/// packet.command_counter() == The number of commands issued since startup (mod 2^32).
	///this part sets play mode and kick mode of game according to command
	if (m_counter != packet.command_counter())
	{
		m_counter = packet.command_counter();
		switch (packet.command()) {
		case SSL_Referee::HALT:
			world.setPlayMode(mode_State::PlayMode::Halt);
			world.setKickMode(mode_State::KickMode::NoKickMode);
			break;

		case SSL_Referee::STOP:
			world.setPlayMode(mode_State::PlayMode::Stop);
			world.setKickMode(mode_State::KickMode::NoKickMode);
			break;

		case SSL_Referee::NORMAL_START:
			world.setPlayMode(mode_State::PlayMode::Wait);
			///////////////!  return_referee_inputstate : it does not get set anywhere
			//switch (world.kickMode)
			//{
			//case Game_State::State::KickoffYellowPrepare:
			//	if (World::getInstance().team_color == TC_Blue)
			//	{
			//		world.setPlayMode(mode_State::PlayMode::Wait);
			//		world.setKickMode(mode_State::KickMode::KickOffO);
			//	}
			//	else if (World::getInstance().team_color == TC_Yellow)
			//	{
			//		world.setPlayMode(mode_State::PlayMode::Wait);
			//		world.setKickMode(mode_State::KickMode::KickOffT);
			//	}
			//	break;
			//case Game_State::State::KickoffBluePrepare:
			//	if (World::getInstance().team_color == TC_Blue)
			//	{
			//		world.setPlayMode(mode_State::PlayMode::Wait);
			//		world.setKickMode(mode_State::KickMode::KickOffT);
			//	}
			//	else if (World::getInstance().team_color == TC_Yellow)
			//	{
			//		world.setPlayMode(mode_State::PlayMode::Wait);
			//		world.setKickMode(mode_State::KickMode::KickOffO);
			//	}
			//	break;
			//case Game_State::State::PenaltyYellowPrepare:
			//	if (World::getInstance().team_color == TC_Blue)
			//	{
			//		world.setPlayMode(mode_State::PlayMode::Wait);
			//		world.setKickMode(mode_State::KickMode::PenaltyO);
			//	}
			//	else if (World::getInstance().team_color == TC_Yellow)
			//	{
			//		world.setPlayMode(mode_State::PlayMode::Wait);
			//		world.setKickMode(mode_State::KickMode::PenaltyT);
			//	}
			//	break;
			//case Game_State::State::PenaltyBluePrepare:
			//	if (World::getInstance().team_color == TC_Blue)
			//	{
			//		world.setPlayMode(mode_State::PlayMode::Wait);
			//		world.setKickMode(mode_State::KickMode::PenaltyT);
			//	}
			//	else if (World::getInstance().team_color == TC_Yellow)
			//	{
			//		world.setPlayMode(mode_State::PlayMode::Wait);
			//		world.setKickMode(mode_State::KickMode::PenaltyO);
			//	}
			//	break;
			//default:
			//	// silently ignore start command
			//	break;
			//}
			//break;
			switch (world.kickMode)
			{
			case mode_State::KickOffTPrepare:
				world.setKickMode(mode_State::KickMode::KickOffT);
				break;
			case mode_State::KickOffOPrepare:
				world.setKickMode(mode_State::KickMode::KickOffO);
				break;
			case mode_State::PenaltyTPrepare:
				world.setKickMode(mode_State::KickMode::PenaltyT);
				break;
			case mode_State::PenaltyOPrepare:
				world.setKickMode(mode_State::KickMode::PenaltyO);
				break;
			default:
				// silently ignore start command
				break;
			}
			break;

		case SSL_Referee::FORCE_START:
			world.setPlayMode(mode_State::PlayMode::/*Wait*/Play);
			world.setKickMode(mode_State::KickMode::NoKickMode);
			break;

		case SSL_Referee::PREPARE_KICKOFF_YELLOW:
			if (World::getInstance().team_color == TC_Blue)
			{
				world.setPlayMode(mode_State::PlayMode::Stop);
				world.setKickMode(mode_State::KickMode::KickOffOPrepare);
			}
			else if (World::getInstance().team_color == TC_Yellow)
			{
				world.setPlayMode(mode_State::PlayMode::Stop);
				world.setKickMode(mode_State::KickMode::KickOffTPrepare);
			}
			break;

		case SSL_Referee::PREPARE_KICKOFF_BLUE:
			if (World::getInstance().team_color == TC_Blue)
			{
				world.setPlayMode(mode_State::PlayMode::Stop);
				world.setKickMode(mode_State::KickMode::KickOffTPrepare);
			}
			else if (World::getInstance().team_color == TC_Yellow)
			{
				world.setPlayMode(mode_State::PlayMode::Stop);
				world.setKickMode(mode_State::KickMode::KickOffOPrepare);
			}
			break;

		case SSL_Referee::PREPARE_PENALTY_YELLOW:
			if (World::getInstance().team_color == TC_Blue)
			{
				world.setPlayMode(mode_State::PlayMode::Stop);
				world.setKickMode(mode_State::KickMode::PenaltyOPrepare);
			}
			else if (World::getInstance().team_color == TC_Yellow)
			{
				world.setPlayMode(mode_State::PlayMode::Stop);
				world.setKickMode(mode_State::KickMode::PenaltyTPrepare);
			}
			break;

		case SSL_Referee::PREPARE_PENALTY_BLUE:
			if (World::getInstance().team_color == TC_Blue)
			{
				world.setPlayMode(mode_State::PlayMode::Stop);
				world.setKickMode(mode_State::KickMode::PenaltyTPrepare);
			}
			else if (World::getInstance().team_color == TC_Yellow)
			{
				world.setPlayMode(mode_State::PlayMode::Stop);
				world.setKickMode(mode_State::KickMode::PenaltyOPrepare);
			}
			break;

		case SSL_Referee::DIRECT_FREE_YELLOW:
			if (World::getInstance().team_color == TC_Blue)
			{
				world.setPlayMode(mode_State::PlayMode::Wait);
				world.setKickMode(mode_State::KickMode::DirectFreeKickO);
			}
			else if (World::getInstance().team_color == TC_Yellow)
			{
				world.setPlayMode(mode_State::PlayMode::Wait);
				world.setKickMode(mode_State::KickMode::DirectFreeKickT);
			}
			break;

		case SSL_Referee::DIRECT_FREE_BLUE:
			if (World::getInstance().team_color == TC_Blue)
			{
				world.setPlayMode(mode_State::PlayMode::Wait);
				world.setKickMode(mode_State::KickMode::DirectFreeKickT);
			}
			else if (World::getInstance().team_color == TC_Yellow)
			{
				world.setPlayMode(mode_State::PlayMode::Wait);
				world.setKickMode(mode_State::KickMode::DirectFreeKickO);
			}
			break;

		case SSL_Referee::INDIRECT_FREE_YELLOW:
			if (World::getInstance().team_color == TC_Blue)
			{
				world.setPlayMode(mode_State::PlayMode::Wait);
				world.setKickMode(mode_State::KickMode::IndirectFreeKickO);
			}
			else if (World::getInstance().team_color == TC_Yellow)
			{
				world.setPlayMode(mode_State::PlayMode::Wait);
				world.setKickMode(mode_State::KickMode::IndirectFreeKickT);
			}
			break;

		case SSL_Referee::INDIRECT_FREE_BLUE:
			if (World::getInstance().team_color == TC_Blue)
			{
				world.setPlayMode(mode_State::PlayMode::Wait);
				world.setKickMode(mode_State::KickMode::IndirectFreeKickT);
			}
			else if (World::getInstance().team_color == TC_Yellow)
			{
				world.setPlayMode(mode_State::PlayMode::Wait);
				world.setKickMode(mode_State::KickMode::IndirectFreeKickO);
			}
			break;

		case SSL_Referee::TIMEOUT_YELLOW:
			world.setPlayMode(mode_State::PlayMode::Timeout);
			world.setKickMode(mode_State::KickMode::NoKickMode);
			break;

		case SSL_Referee::TIMEOUT_BLUE:
			world.setPlayMode(mode_State::PlayMode::Timeout);
			world.setKickMode(mode_State::KickMode::NoKickMode);
			break;

		case SSL_Referee::GOAL_YELLOW:
			world.setPlayMode(mode_State::PlayMode::Stop);
			world.setKickMode(mode_State::KickMode::NoKickMode);
			break;

		case SSL_Referee::GOAL_BLUE:
			world.setPlayMode(mode_State::PlayMode::Stop);
			world.setKickMode(mode_State::KickMode::NoKickMode);
			break;

		case SSL_Referee::BALL_PLACEMENT_YELLOW:
			world.setKickMode(mode_State::KickMode::NoKickMode);
			if (World::getInstance().team_color == TC_Blue)
				world.setPlayMode(mode_State::PlayMode::Wait);
			else if (World::getInstance().team_color == TC_Yellow)
				world.setPlayMode(mode_State::PlayMode::Stop);
			break;

		case SSL_Referee::BALL_PLACEMENT_BLUE:
			world.setKickMode(mode_State::KickMode::NoKickMode);
			if (World::getInstance().team_color == TC_Blue)
				world.setPlayMode(mode_State::PlayMode::Wait);
			else if (World::getInstance().team_color == TC_Yellow)
				world.setPlayMode(mode_State::PlayMode::Stop);
			break;

		}
	}

	world.ballPlacementPosition.setX(packet.designated_position().x());
	world.ballPlacementPosition.setY(packet.designated_position().y());
	
	///add stage : OK
	///add ball placement position	: OK

	switch (packet.stage())
	{
	case mode_State::StageMode::NORMAL_FIRST_HALF_PRE:
		world.setStageMode(mode_State::StageMode::NORMAL_FIRST_HALF_PRE);
		break;
	case mode_State::StageMode::NORMAL_FIRST_HALF:
		world.setStageMode(mode_State::StageMode::NORMAL_FIRST_HALF);
		break;
	case mode_State::StageMode::NORMAL_HALF_TIME:
		world.setStageMode(mode_State::StageMode::NORMAL_HALF_TIME);
		break;
	case mode_State::StageMode::NORMAL_SECOND_HALF_PRE:
		world.setStageMode(mode_State::StageMode::NORMAL_SECOND_HALF_PRE);
		break;
	case mode_State::StageMode::NORMAL_SECOND_HALF:
		world.setStageMode(mode_State::StageMode::NORMAL_SECOND_HALF);
		break;
	case mode_State::StageMode::EXTRA_TIME_BREAK:
		world.setStageMode(mode_State::StageMode::EXTRA_TIME_BREAK);
		break;
	case mode_State::StageMode::EXTRA_FIRST_HALF_PRE:
		world.setStageMode(mode_State::StageMode::EXTRA_FIRST_HALF_PRE);
		break;
	case mode_State::StageMode::EXTRA_FIRST_HALF:
		world.setStageMode(mode_State::StageMode::EXTRA_FIRST_HALF);
		break;
	case mode_State::StageMode::EXTRA_HALF_TIME:
		world.setStageMode(mode_State::StageMode::EXTRA_HALF_TIME);
		break;
	case mode_State::StageMode::EXTRA_SECOND_HALF_PRE:
		world.setStageMode(mode_State::StageMode::EXTRA_SECOND_HALF_PRE);
		break;
	case mode_State::StageMode::EXTRA_SECOND_HALF:
		world.setStageMode(mode_State::StageMode::EXTRA_SECOND_HALF);
		break;
	case mode_State::StageMode::PENALTY_SHOOTOUT_BREAK:
		world.setStageMode(mode_State::StageMode::PENALTY_SHOOTOUT_BREAK);
		break;
	case mode_State::StageMode::PENALTY_SHOOTOUT:
		world.setStageMode(mode_State::StageMode::PENALTY_SHOOTOUT);
		break;
	case mode_State::StageMode::POST_GAME:
		world.setStageMode(mode_State::StageMode::POST_GAME);
		break;
	}

	
}

