#ifndef GAME_STATE_H
#define GAME_STATE_H

//class Game_State {
//public:
//	enum State {
//		Halt = 0,
//		Stop = 1,
//		Game = 2,
//		GameForce = 3,
//
//		KickoffYellowPrepare = 5,
//		KickoffYellow = 6,
//		PenaltyYellowPrepare = 7,
//		PenaltyYellow = 8,
//		DirectYellow = 9,
//		IndirectYellow = 10,
//
//		KickoffBluePrepare = 11,
//		KickoffBlue = 12,
//		PenaltyBluePrepare = 13,
//		PenaltyBlue = 14,
//		DirectBlue = 15,
//		IndirectBlue = 16,
//		
//		TimeoutYellow = 17,
//		TimeoutBlue = 18,
//	};
//	const char* getStateName( State stat)
//	{
//		switch (stat)
//		{
//		case Halt:return "Halt";
//		case Stop:return "Stop";
//		case Game:return "Game";
//		case GameForce:return "GameForce";
//		case KickoffYellowPrepare:return "KickoffYellowPrepare";
//		case KickoffYellow:return "KickoffYellow";
//		case PenaltyYellowPrepare:return "PenaltyYellowPrepare";
//		case PenaltyYellow:return "PenaltyYellow";
//		case DirectYellow:return "DirectYellow";
//		case IndirectYellow:return "IndirectYellow";
//		case KickoffBluePrepare:return "KickoffBluePrepare";
//		case KickoffBlue:return "KickoffBlue";
//		case PenaltyBluePrepare:return "PenaltyBluePrepare";
//		case PenaltyBlue:return "PenaltyBlue";
//		case DirectBlue:return "DirectBlue";
//		case IndirectBlue:return "IndirectBlue";
//		case TimeoutYellow:return "TimeoutYellow";
//		case TimeoutBlue:return "TimeoutBlue";
//		}
//	}
//};

///*mode: PlayMode / KickMode
class mode_State
{
public:
	enum PlayMode {
		Halt,
		Timeout,
		Stop,
		Wait,
		Play,
		ballPlacement
	};
	const char* getPlayModeName( PlayMode playMode)
	{
		switch (playMode)
		{
		case Halt:return "Halt";
		case Stop:return "Stop";
		case Wait:return "Wait";
		case Play:return "Play";
		case Timeout:return "Timeout";
		}
	}

	enum KickMode {
		NoKickMode ,
		KickOffTPrepare,
		KickOffOPrepare,
		KickOffT,
		KickOffO,
		DirectFreeKickT,
		DirectFreeKickO,
		IndirectFreeKickT,
		IndirectFreeKickO,
		PenaltyTPrepare,
		PenaltyOPrepare,
		PenaltyT,
		PenaltyO,
	};
	const char* getKickModeName( KickMode kickMode)
	{
		switch (kickMode)
		{
		case KickOffTPrepare:return "KickOffTPrepare";
		case KickOffOPrepare:return "KickOffOPrepare";
		case KickOffT:return "KickOffT";
		case KickOffO:return "KickOffO";
		case DirectFreeKickT:return "DirectFreeKickT";
		case DirectFreeKickO:return "DirectFreeKickO";
		case IndirectFreeKickT:return "IndirectFreeKickT";
		case IndirectFreeKickO:return "IndirectFreeKickO";
		case PenaltyTPrepare:return "PenaltyTPrepare";
		case PenaltyOPrepare:return "PenaltyOPrepare";
		case PenaltyT:return "PenaltyT";
		case PenaltyO:return "PenaltyO";
		case NoKickMode:return "NoKickMode";
		}
	}

	enum StageMode {
		// The first half is about to start.
		// A kickoff is called within this stage.
		// This stage ends with the NORMAL_START.
		NORMAL_FIRST_HALF_PRE = 0,
		// The first half of the normal game, before half time.
		NORMAL_FIRST_HALF = 1,
		// Half time between first and second halves.
		NORMAL_HALF_TIME = 2,
		// The second half is about to start.
		// A kickoff is called within this stage.
		// This stage ends with the NORMAL_START.
		NORMAL_SECOND_HALF_PRE = 3,
		// The second half of the normal game, after half time.
		NORMAL_SECOND_HALF = 4,
		// The break before extra time.
		EXTRA_TIME_BREAK = 5,
		// The first half of extra time is about to start.
		// A kickoff is called within this stage.
		// This stage ends with the NORMAL_START.
		EXTRA_FIRST_HALF_PRE = 6,
		// The first half of extra time.
		EXTRA_FIRST_HALF = 7,
		// Half time between first and second extra halves.
		EXTRA_HALF_TIME = 8,
		// The second half of extra time is about to start.
		// A kickoff is called within this stage.
		// This stage ends with the NORMAL_START.
		EXTRA_SECOND_HALF_PRE = 9,
		// The second half of extra time.
		EXTRA_SECOND_HALF = 10,
		// The break before penalty shootout.
		PENALTY_SHOOTOUT_BREAK = 11,
		// The penalty shootout.
		PENALTY_SHOOTOUT = 12,
		// The game is over.
		POST_GAME = 13
	};
	const char* getStageName(StageMode stag)
	{
		switch (stag)
		{
		case NORMAL_FIRST_HALF_PRE: return "NORMAL_FIRST_HALF_PRE";
		case NORMAL_FIRST_HALF: return "NORMAL_FIRST_HALF";
		case NORMAL_HALF_TIME: return "NORMAL_HALF_TIME";
		case NORMAL_SECOND_HALF_PRE: return "NORMAL_SECOND_HALF_PRE";
		case NORMAL_SECOND_HALF: return "NORMAL_SECOND_HALF";
		case EXTRA_TIME_BREAK: return "EXTRA_TIME_BREAK";
		case EXTRA_FIRST_HALF_PRE: return "EXTRA_FIRST_HALF_PRE";
		case EXTRA_FIRST_HALF: return "EXTRA_FIRST_HALF";
		case EXTRA_HALF_TIME: return "EXTRA_HALF_TIME";
		case EXTRA_SECOND_HALF_PRE: return "EXTRA_SECOND_HALF_PRE";
		case EXTRA_SECOND_HALF: return "EXTRA_SECOND_HALF";
		case PENALTY_SHOOTOUT_BREAK: return "PENALTY_SHOOTOUT_BREAK";
		case PENALTY_SHOOTOUT: return "PENALTY_SHOOTOUT";
		case POST_GAME: return "POST_GAME";
		}
	}
};


#endif // GAME_STATE_H