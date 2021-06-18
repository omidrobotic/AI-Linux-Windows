#ifndef STRATEGYGENERALFUNCTIOS_H 
#define STRATEGYGENERALFUNCTIOS_H
#include "estimation.h"
///mohammadhossein
#define Max_Time_Estimation 100
#define Length_division 10
#define Width_division 10
///end mohammadhossein
class VecPosition;

//#define POSITIONS_FOR_MAKE_SITUATION 100//not used
enum condition_pass
{
	submit = 0,
	expectation = 1,
	receive = 2
};
enum ownership
{
	opponent = 0,
	teammate = 1,
	ex = 2
};
class HighLevel
{
public:
	//////////////////////zolfaghari
	static condition_pass pass_mode;
	static ownership play_mode;
	/////////////////////end
	//static void SetPrimitiveArrangement();
	//static int GetNearestRobotTo(VecPosition point);
	//static void SetGoaliPrimitiveArrangement();

	//bool Has50cmDistanceFromBall(int id);
	//void Get50cmDistanceFromBall(int id);

	//static void StartO(World &world);
	//static void StartT(World &world);

    ////Begin Farhan Demi
    static void gotoXY(int robotIndex, VecPosition target);
    static void lookAt(int robotIndex, float angle);
    static bool move_ball_to_position(int robotIndex, VecPosition target_pos);
    static void goalKeeper_defend_and_pass(int goalKeeperIndex);
    static bool ball_is_in_penalty_area(char team);
    static void defence_formation(int number_of_defender);
    static int  nearest_robot_to_point_except_goali(VecPosition postion);
    static int  oponent_is_shooting_index();
    static void forward_formation(int number_of_forwards);
    static void turn_all_spinbacks_on();
    static void turn_all_spinbacks_off();
    static void turn_spinbacks_on(int robotIndex);
    static void turn_spinbacks_off(int robotIndex);


    ////End Farhan Demi

	////mohammad hossin
	static void EstimationPosition(World &world);
	static void Scoring_Situations(World &world);
	////end mohammadhossein
	static  void ReadyForKick(int index);
	static	void GoaliHoleCover();
	static  void GoaliCutShoot(World &world, int id);
	static  void DefenceCutShoot(int NumofRobot, int index_robotT[]);
	static  void Block(int NumofRobot, int index_robotT[]);
	static  void BlockKicker(int NumofRobot, int index_robotT[]);
	static  void Banish(int index_robotT);
	static  void Shoot(int index_robotT);
	static  void StopSurrounding(int NumofRobot, int index_robotT[]);
	static  void AntiOneTouch_positionball(World &world, int id);
	static  void AntiOneTouch(World &world, int NumofRobot, int index_robotT[]);
	static  void CatchPenalty(World &world, int index);
	static  void BlockOponent(int NumofRobot, int index_robotT[], int dangerer_robotO_index);
	static void several_position_Line(VecPosition dest, int NumofRobot, int index_robott[]);
	////////mohammadhossein_zolfaghari
	//static double max(double x, double y);
	static void find_roboto_pass(int number_of_robott_block_roboto);
	static void direct_free_kick(int number_of_attacker, int sender_index);
	static void defence_scor2(int number_of_defender);
	static double rel(double x, double min, double max);
	static int find_robot_have_ball(char team);
	static int find_best_robot_pass(int index_robotT);
	static int nearest_robot_to_ball(char Team);
	static void start_robotT_format_penaltyo(World &world);
	static int nearest_robot_to_point(char Team, VecPosition postion);
	static void start_robotT_format_NoKickMode(string playmode);
	static  void Ready(World &world, int index);
	static void catch_the_ball_2point(World &world, VecPosition dest, int index1, int index2);
	static  void Pass(int index1, int index2);
	static void start_robotT_format_penaltyt(World &world, int index);
	static void ready_for_penaltyt(World &world, int index);
	static void catch_the_ball_2point_whithspeenback(World &world, int index, VecPosition dest);
	static void defence_format();
	static void time_out();
	static void plan_scor(int number_of_attacker);
	static void DefenceHoleCover(int number_robotT, int index_robot[]);
	static void defence_scor(int number_of_defender);
	static void set_last_destination_set();
	static void check_last_destination_set(int index_robotT, VecPosition destination);
	static void ownership_ball();
	static int block_the_ball_to_point(VecPosition send, VecPosition recive);
	static void defence_hol_robotO(int robotO_index, int number_of_defender);
	static void robot_t_can_pass(int robot_hav_ball);
	////
	static  VecPosition Block_get_target();
	//// SMMSS
	static void Overlap();
private:
	static int onces_program;
	//	static  VecPosition dest;
	static  VecPosition Block_get_special_target(int my_rank, int num_of_block_robot, VecPosition general_target, int direction);

	static  int BlockKicker_Is_Moving(VecPosition newIntersection[]);
	static  VecPosition StopSurrounding_get_general_target();
	static  VecPosition StopSurrounding_get_special_target(int my_rank, int num_of_ready_robot, double general_angle, VecPosition ball_position);
	static  VecPosition BlockOponent_get_general_target(int dangerer_robotO_index);
};

#endif // HIGHLEVEL_H

