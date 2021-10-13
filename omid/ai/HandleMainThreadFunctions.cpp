#include "world.h"
#include "RRT.h"
#include "MotionControl.h"
#include "Switches.h"
#include "HighLevel.h"
int y = 0;
int stage = -1, cnt =0;
int challengeNumber = 6;
VecPosition dest;
bool ch3= false;
bool shooted= true;
bool passi= false;
//Strategy
int indexOfNearestRobot;
int _a=1;

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

    for (int i = 0; i < world.numT; i++) {
        int index = world.getIndexForRobotTNumber(i);
        if(index != -1) {
            if(world.getRobotTNumberForIndex(index) == GOALIE_NUM) world.robotT[index].role = "Goalie";
            else                    world.robotT[index].role = "None";
        }
    }
//    for (int i = 0; i < world.numT; ++i) {
//    for (int i = 0; i < world.numT; ++i) {
//        if (world.robotT[i].kick_power>0)
//        {
//            world.robotT[i].spinBack= false;
//        } else
//        {
//            world.robotT[i].spinBack= true;
//        }
//    }




    if(DIVISION==1)
    {
    if (world.numT>2) {
//        HighLevel::GoaliHoleCover();
        HighLevel::GoalieDefend(0);

       // HighLevel::defence_scor2(((world.numT - 2) - int(PRESENT_OF_ATTACKER * (world.numT - 2))));

        switch (world.playMode) {


            case mode_State::ballPlacement:
                /*  indexOfNearestRobot = HighLevel::nearest_robot_to_ball('T');
                  for (int i = 0; i < world.numT; i++)
                  {
                      if (indexOfNearestRobot==i)
                          HighLevel::move_ball_to_position(world.getRobotTNumberForIndex(indexOfNearestRobot),
                                                           world.team_T.Set_Refree_Ball_Position);
                      else
                          world.robotT[i].destination_position=world.robotT[i].position;
                  }*/

                world.robotT[indexOfNearestRobot].kick_power = 0;
                break;
            case mode_State::Stop:
                switch (world.kickMode) {
                    case mode_State::KickMode::NoKickMode:
                     //   HighLevel::GoaliHoleCover();
                        HighLevel::ReadyForKick(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                      //  HighLevel::defence_scor2(((world.numT - 2) - int(PRESENT_OF_ATTACKER * (world.numT - 2))));
                        HighLevel::find_roboto_pass(int(PRESENT_OF_ATTACKER * (world.numT - 2)));
                        break;

                    case mode_State::KickMode::KickOffOPrepare:
                        HighLevel::start_robotT_format_NoKickMode("KickOffOPrepare");
                        break;

                    case mode_State::KickMode::KickOffTPrepare:
                        HighLevel::start_robotT_format_NoKickMode("KickOffTPrepare");
                        HighLevel::ReadyForKick(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));


                        break;

                    case mode_State::KickMode::PenaltyTPrepare:
                        HighLevel::start_robotT_format_penaltyt(world, 1);
                        break;

                    case mode_State::KickMode::PenaltyOPrepare:
                        HighLevel::start_robotT_format_penaltyo(world);
                        break;
                    default:
                        break;
                }
                break;


            case mode_State::Wait:
                switch (world.kickMode) {
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

                    case mode_State::KickMode::NoKickMode:
                        HighLevel::start_robotT_format_NoKickMode("NoKickMode");
                        break;


                    case mode_State::KickMode::DirectFreeKickT:
                 //       HighLevel::GoaliHoleCover();
                        HighLevel::find_best_robot_pass(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                       // HighLevel::defence_scor2((world.numT - 2) - int((world.numT - 2) * PRESENT_OF_ATTACKER));
                        HighLevel::direct_free_kick(int(PRESENT_OF_ATTACKER * (world.numT - 2)) + 1, 0);
                        break;

                    case mode_State::KickMode::DirectFreeKickO:
                  //      HighLevel::GoaliHoleCover();
                        //HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                     //   HighLevel::defence_scor2(int(PRESENT_OF_ATTACKER * (world.numT - 2)));
                        HighLevel::find_roboto_pass(int(PRESENT_OF_ATTACKER * (world.numT - 2)));
                        break;

                    case mode_State::KickMode::IndirectFreeKickT:
                   //     HighLevel::GoaliHoleCover();
                        HighLevel::find_best_robot_pass(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                       // HighLevel::defence_scor2((world.numT - 2) - int((world.numT - 2) * PRESENT_OF_ATTACKER));
                        HighLevel::direct_free_kick(int(PRESENT_OF_ATTACKER * (world.numT - 2)) + 1, 0);
                        break;

                    case mode_State::KickMode::IndirectFreeKickO:
                   //     HighLevel::GoaliHoleCover();
                        HighLevel::find_roboto_pass(int(PRESENT_OF_ATTACKER * (world.numT - 2)));
                   //     HighLevel::defence_scor2(((world.numT - 2) - int(PRESENT_OF_ATTACKER * (world.numT - 2))));
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
                //else if (HighLevel::play_mode== opponent)
                /*     {
                         HighLevel::GoaliHoleCover();
                         HighLevel::defence_hol_robotO(1, HighLevel::find_robot_have_ball('O'));
                         HighLevel::defence_scor2(int((world.numT-2)*0.67));
                         HighLevel::find_roboto_pass(world.numT-(int((world.numT - 2) * 0.67)+2));
                     }*/
                /*else
                {
                    HighLevel::GoaliHoleCover();
                    HighLevel::defence_hol_robotO(-1,1);
                    HighLevel::defence_scor2(int((world.numT - 2) * 0.67));
                    HighLevel::find_roboto_pass(world.numT - (int((world.numT - 2) * 0.67) + 2));
                }*/

                switch (world.kickMode) {
                    case mode_State::KickMode::KickOffO:
                        HighLevel::start_robotT_format_NoKickMode("KickOffO");
                        break;

                    case mode_State::KickMode::KickOffT:
                        HighLevel::ReadyForKick(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        break;

                    case mode_State::KickMode::PenaltyO:
                        break;

                    case mode_State::KickMode::PenaltyT:
                        break;

                    case mode_State::KickMode::NoKickMode:
//                        cout<< world.team_side << '\n';


                    /* AI for Robocup 2021
                    //    HighLevel::GoaliHoleCover();
                        HighLevel::plan_scor(int(PRESENT_OF_ATTACKER * (world.numT - 2)));

                        if (world.ball.getCurrentBallPosition().getDistanceTo(Field::getGoalMidO()) < 5000 &&
                            HighLevel::find_robot_have_ball('O') == -1) {
                            HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
                        } else {
                            HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));
                        }
                        // cout<<"shoot  :"<<world.robotT[HighLevel::nearest_robot_to_ball('T')].kick_power<<'\n';
                        //cout << world.getRobotTNumberForIndex(uyu) << endl;
//                        HighLevel::defence_scor2(((world.numT - 2) - int(PRESENT_OF_ATTACKER * (world.numT - 2))));
                    */

                        break;

                    case mode_State::KickMode::DirectFreeKickT:

                        HighLevel::start_robotT_format_NoKickMode("DirectFreeKickT");
                        HighLevel::ReadyForKick(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        break;

                    case mode_State::KickMode::DirectFreeKickO:
                        HighLevel::start_robotT_format_NoKickMode("DirectFreeKickO");
                        break;

                    case mode_State::KickMode::IndirectFreeKickT:
                        HighLevel::start_robotT_format_NoKickMode("IndirectFreeKickT");
                        HighLevel::ReadyForKick(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        break;

                    case mode_State::KickMode::IndirectFreeKickO:
                        HighLevel::start_robotT_format_NoKickMode("IndirectFreeKickO");
                        break;

                    default:
                        break;
                }
                break;


            case mode_State::Halt:
                for (int i = 0; i < MAX_ROBOTS_PER_TEAM_IN_THE_FIELD; i++) {
                    world.robotT[i].velocityToGo = VecPosition(0, 0);
                    world.robotT[i].wToGo = 0;
                }
                break;

            case mode_State::Timeout:
                HighLevel::time_out();
                break;

            default:
                break;
        }
    }
    }
    else if(DIVISION==2)
    {
    //    int itis[1];
    //    itis[0]=HighLevel::nearest_robot_to_ball('T');
    //    HighLevel::BlockKicker(1, itis);
    HighLevel::GoalieDefend(GOALIE_NUM);
    if(world.numT>2)
    {
        int min_robot=((world.numT - 2) - int(PRESENT_OF_ATTACKER * (world.numT - 2)));
        int max_robot=int(PRESENT_OF_ATTACKER * (world.numT - 2));

        switch (world.playMode) {
            case mode_State::ballPlacement:
                  indexOfNearestRobot = HighLevel::nearest_robot_to_ball('T');
                  for (int i = 0; i < world.numT; i++)
                  {
                      HighLevel::move_ball_to_position(world.getRobotTNumberForIndex(indexOfNearestRobot),
                                                       world.team_T.Set_Refree_Ball_Position);
                  /*    if (indexOfNearestRobot==i)
                          HighLevel::move_ball_to_position(world.getRobotTNumberForIndex(indexOfNearestRobot),
                                                           world.team_T.Set_Refree_Ball_Position);
                      else
                          world.robotT[i].destination_position=world.robotT[i].position;*/
                  }

                world.robotT[indexOfNearestRobot].kick_power = 0;
                break;
            case mode_State::Stop:
                switch (world.kickMode) {
                    case mode_State::KickMode::NoKickMode:

                        HighLevel::GoaliHoleCover();
                        HighLevel::go_back_ball(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition())) ;
                     // if(min_robot>0)HighLevel::find_roboto_pass(min_robot);
                        if(max_robot>0)HighLevel::defence_scor2(max_robot);
                        break;

                    case mode_State::KickMode::KickOffOPrepare:
                        HighLevel::start_robotT_format_NoKickMode("KickOffOPrepare");
                        break;

                    case mode_State::KickMode::KickOffTPrepare:
                        HighLevel::start_robotT_format_NoKickMode("KickOffTPrepare");
                        HighLevel::ReadyForKick(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));


                        break;

                    case mode_State::KickMode::PenaltyTPrepare:
                        HighLevel::start_robotT_format_penaltyt(world, 1);
                        break;

                    case mode_State::KickMode::PenaltyOPrepare:
                        HighLevel::start_robotT_format_penaltyo(world);
                        break;
                    default:
                        break;
                }
                break;


            case mode_State::Wait:
                switch (world.kickMode) {
                    case mode_State::KickMode::KickOffO:
                        HighLevel::start_robotT_format_NoKickMode("KickOffO");
                        break;

                    case mode_State::KickMode::KickOffT:
                        HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
                        break;

                    case mode_State::KickMode::PenaltyO:
                        HighLevel::start_robotT_format_penaltyo(world);
                        break;

                    case mode_State::KickMode::PenaltyT:
                        HighLevel::Shoot(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        HighLevel::start_robotT_format_penaltyt(world, 1);
                        break;

                    case mode_State::KickMode::NoKickMode:
                        HighLevel::start_robotT_format_NoKickMode("NoKickMode");
                        break;


                    case mode_State::KickMode::DirectFreeKickT:
                        HighLevel::GoaliHoleCover();
//                        HighLevel::find_best_robot_pass(
//                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
                        HighLevel::defence_scor2(min_robot);
                        HighLevel::direct_free_kick(max_robot + 1, 0);
//                        world.playMode = mode_State::Play;
                        break;

                    case mode_State::KickMode::DirectFreeKickO:
                        HighLevel::GoaliHoleCover();
                        HighLevel::go_back_ball(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition())) ;

                        /// HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        HighLevel::defence_scor2(max_robot);
                        HighLevel::find_roboto_pass(min_robot);
//                        world.playMode = mode_State::Play;
                        break;

                    case mode_State::KickMode::IndirectFreeKickT:
                        HighLevel::GoaliHoleCover();
                        HighLevel::find_best_robot_pass(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        HighLevel::defence_scor2(max_robot + 1);
                        HighLevel::direct_free_kick(min_robot, 0);
//                        world.playMode = mode_State::Play;
                        break;

                    case mode_State::KickMode::IndirectFreeKickO:
                        HighLevel::GoaliHoleCover();
                        HighLevel::find_roboto_pass(min_robot);
                        HighLevel::defence_scor2(max_robot);
                        HighLevel::go_back_ball(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition())) ;
//                        world.playMode = mode_State::Play;
                        //HighLevel::ReadyForKick(HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        break;
                    default:
                        break;
                }
                break;

            case mode_State::Play:
//                HighLevel::RobotFormation();
//                HighLevel::ownership_ball();
                HighLevel::defence_scor2(1);
                HighLevel::plan_scor(4);
                HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));
                HighLevel::GoalieDefend(GOALIE_NUM);
                //world.setKickMode(mode_State::KickMode::NoKickMode);
                //	if (HighLevel::play_mode == teammate)
                //	{

                //	}
                //else if (HighLevel::play_mode== opponent)
                    /* {
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
                }*/

                switch (world.kickMode) {
<<<<<<< HEAD
//                    case mode_State::KickMode::KickOffO:
//                        HighLevel::start_robotT_format_NoKickMode("KickOffO");
//                        break;
//
//                    case mode_State::KickMode::KickOffT:
//                        HighLevel::GoaliHoleCover();
//                        HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
//                        HighLevel::defence_scor2(max_robot);
////                        HighLevel::ReadyForKick(
////                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
//                        break;
//
//                    case mode_State::KickMode::PenaltyO:
//                        HighLevel::GoaliHoleCover();
//                        break;
//
//                    case mode_State::KickMode::PenaltyT:
//                        HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
//                        break;

//                    case mode_State::KickMode::NoKickMode:
//                       /// HighLevel::GoalieDefend(world.getIndexForRobotTNumber(world.team_T.Goalie));
//                        if(HighLevel::find_robot_have_ball('T')!=-1) {
//
//                            HighLevel::GoaliHoleCover();
//                           /// HighLevel::go_to_ball(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));
//                            HighLevel::defence_scor2(max_robot);
//                            HighLevel::plan_scor(min_robot);
//                            if (world.ball.getCurrentBallPosition().getDistanceTo(Field::getGoalMidO()) < 7000 &&
//                                HighLevel::find_robot_have_ball('O') == -1) {
//                                HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
//                            } else {
//                                HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));
//                            }
//                        } else if(HighLevel::find_robot_have_ball('O')!=-1)
//                        {
//                            HighLevel::GoaliHoleCover();
//                            HighLevel::go_to_ball(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));
//                            HighLevel::defence_scor2(max_robot);
//                            HighLevel::go_back_ball(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));
//                        } else
//                        {
//                            HighLevel::GoaliHoleCover();
//                            HighLevel::go_to_ball(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));
//                            HighLevel::defence_scor2(max_robot);
//                            HighLevel::plan_scor(min_robot);
//                        }
//                        break;
=======
                    case mode_State::KickMode::KickOffO:
                        HighLevel::start_robotT_format_NoKickMode("KickOffO");
                        break;

                    case mode_State::KickMode::KickOffT:
                        HighLevel::ReadyForKick(
                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        break;

                    case mode_State::KickMode::PenaltyO:
                        HighLevel::GoaliHoleCover();
                        break;

                    case mode_State::KickMode::PenaltyT:
                        HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
                        break;

                    case mode_State::KickMode::NoKickMode:
                        HighLevel::GoalieDefend(world.getIndexForRobotTNumber(world.team_T.Goalie));
                        if(HighLevel::find_robot_have_ball('T')!=-1) {


                            HighLevel::GoaliHoleCover();
                            HighLevel::defence_scor2(max_robot);
                            HighLevel::plan_scor(min_robot);
                            if (world.ball.getCurrentBallPosition().getDistanceTo(Field::getGoalMidO()) < 7000 &&
                                HighLevel::find_robot_have_ball('O') == -1) {
                                HighLevel::Shoot(HighLevel::nearest_robot_to_ball('T'));
                            } else {
                                HighLevel::find_best_robot_pass(HighLevel::nearest_robot_to_ball('T'));
                            }
                        } else
                        {
                            int itis2[1];
                            itis2[0]=HighLevel::nearest_robot_to_ball('T');
                     //       HighLevel::StopSurrounding(1, itis2);
                        }
                        break;
>>>>>>> parent of 352a90e (first test on robocup server with div B)

                    case mode_State::KickMode::DirectFreeKickT:
                        HighLevel::Shoot(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));
                        HighLevel::start_robotT_format_NoKickMode("DirectFreeKickT");
                       // HighLevel::ReadyForKick(
                       //         HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        break;

                    case mode_State::KickMode::DirectFreeKickO:
                        HighLevel::GoaliHoleCover();
                        HighLevel::go_back_ball(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));

                        /// HighLevel::go_to_ball(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));
                        HighLevel::defence_scor2(max_robot);
                        //HighLevel::start_robotT_format_NoKickMode("DirectFreeKickO");
                        break;

                    case mode_State::KickMode::IndirectFreeKickT:
                        HighLevel::start_robotT_format_NoKickMode("IndirectFreeKickT");
                        HighLevel::Shoot(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));
//                        HighLevel::ReadyForKick(
//                                HighLevel::nearest_robot_to_point('T', world.ball.getCurrentBallPosition()));
                        break;

                    case mode_State::KickMode::IndirectFreeKickO:
                       // HighLevel::start_robotT_format_NoKickMode("IndirectFreeKickO");
                        HighLevel::GoaliHoleCover();
                        HighLevel::go_back_ball(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));

                        /// HighLevel::go_to_ball(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));
                        HighLevel::defence_scor2(max_robot);
                       break;

                    default:
                        break;
                }
                break;


            case mode_State::Halt:
                for (int i = 0; i < MAX_ROBOTS_PER_TEAM_IN_THE_FIELD; i++) {
                    world.robotT[i].velocityToGo = VecPosition(0, 0);
                    world.robotT[i].wToGo = 0;
                }
                break;

            case mode_State::Timeout:
                HighLevel::time_out();
                break;

            default:
                break;
        }
    }
}

    //set last destination
    HighLevel::set_last_destination_set();

    //auto end_timeM = std::chrono::high_resolution_clock::now();
    //auto timeM = end_timeM - start_timeM;
    //cout << timeM.count() / 1000000.0 << endl;
#elif GAME_MODE_ROBOCUP_2021==2
#elif GAME_MODE_ROBOCUP_2021==3
    int attackerNum =world.getIndexForRobotTNumber(5);
    int defenderNum = world.getIndexForRobotTNumber(7);
  /*  if(world.playMode==mode_State::Stop ||  world.playMode==mode_State::Halt) {
        world.robotT[attackerNum].destination_position=world.robotT[attackerNum].position;
        world.robotT[defenderNum].destination_position=world.robotT[defenderNum].position;
    } else*/// {
  Circle robots[MAX_ROBOTS_IN_THE_FIELD];
    for (int i = 0; i < world.numT; ++i) {
        if(i!=attackerNum)
        {
            robots[i]=Circle(world.robotT[i].position, ROBOT_RADIUS);
        }
    }

        if (world.numT >= 1) {
            /*  for (int i = 0; i < 7; ++i) {
                  world.robotT[i].kick_power=2;
              }*/
            if (challengeNumber == 1) {
                Cone::Hole_Type Longest_Hole1;
                Cone::Hole_Type Longest_Hole2;
                Cone::Hole_Type Longest_Hole3;

                Circle robots[MAX_ROBOTS_IN_THE_FIELD];
                double pos1 = 0;
                double pos2 = 0;
                double pos3 = 0;
                for (int t = 0; t < world.numT; t++) {
                    if (t != defenderNum)
                        robots[t] = Circle(world.robotT[t].position, ROBOT_RADIUS);
                }
                if (stage == -1) {
                    Cone BallToGoal1(VecPosition(-2000, 0), Field::getUpBarO(), Field::getDownBarO());
                    if ((BallToGoal1.Get_Free_Space_In_Cone(robots, world.numT, Longest_Hole1) > 0)) {
                        pos1 = HighLevel::rel((Longest_Hole1.Point_1 - VecPosition(-2000, 0)).AngleBetween(
                                Longest_Hole1.Point_2 - VecPosition(-2000, 0)), 0, 3.14 / 2);;
                    }
                    Cone BallToGoal2(VecPosition(-2000, 1000), Field::getUpBarO(), Field::getDownBarO());
                    if ((BallToGoal2.Get_Free_Space_In_Cone(robots, world.numT, Longest_Hole2) > 0)) {
                        pos2 = HighLevel::rel((Longest_Hole2.Point_1 - VecPosition(-2000, 1000)).AngleBetween(
                                Longest_Hole2.Point_2 - VecPosition(-2000, 1000)), 0, 3.14 / 2);;
                    }
                    Cone BallToGoal3(VecPosition(-2000, -1000), Field::getUpBarO(), Field::getDownBarO());
                    if ((BallToGoal3.Get_Free_Space_In_Cone(robots, world.numT, Longest_Hole3) > 0)) {
                        pos3 = HighLevel::rel((Longest_Hole3.Point_1 - VecPosition(-2000, -1000)).AngleBetween(
                                Longest_Hole3.Point_2 - VecPosition(-2000, -1000)), 0, 3.14 / 2);;
                    }


                    if (pos2 >= pos1 && pos2 >= pos3)
                        dest = VecPosition(-2000, 1000);


                    else if (pos3 >= pos2 && pos3 >= pos1)
                        dest = VecPosition(-2000, -1000);


                    else if (pos1 >= pos2 && pos1 >= pos3)
                        dest = VecPosition(-2000, 0);


                    stage = 2;
                }
                if (stage == 2) {
                    if (world.robotT[defenderNum].position.getDistanceTo(dest) > 200) {
                        world.robotT[defenderNum].destination_position = dest;
                        HighLevel::lookAtPos(defenderNum, Field::getGoalMidO());
                    } else {
                        world.robotT[defenderNum].destination_position = world.robotT[defenderNum].position;
                        HighLevel::lookAtPos(defenderNum, Field::getGoalMidO());
                    }


                    if (shooted == true) {
                        shooted = HighLevel::go_back_ball(attackerNum, world.robotT[defenderNum].position);

                    } else {
                        HighLevel::go_back_ball(attackerNum, world.robotT[defenderNum].position);
                        if (world.ball.getCurrentBallPosition().getDistanceTo(world.robotT[attackerNum].position) >
                            world.ball.getCurrentBallPosition().getDistanceTo(world.robotT[defenderNum].position))
                            stage = 3;
                    }
                }
                if (stage == 3) {
                    HighLevel::Shoot(attackerNum);
                    world.robotT[defenderNum].destination_position = VecPosition(0, 0);
                }
            }

            if (challengeNumber == 2) {
                VecPosition intersection1, intersection2;
                Line ball_to_robotO = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(),
                                                                  Field::getGoalMidO());
                Circle roboto = Circle(world.ball.getCurrentBallPosition(), 2 * ROBOT_RADIUS);
                ball_to_robotO.getCircleIntersectionPoints(roboto, &intersection1, &intersection2);
                if (Field::getGoalMidO().getDistanceTo(intersection1) >
                    Field::getGoalMidO().getDistanceTo(intersection2)) {
                    if (world.robotT[attackerNum].position.getDistanceTo(intersection1) > 100) {
                        world.robotT[attackerNum].destination_position = intersection1;
                    } else {
                        //world.robotT[index].destination_position = world.robotT[index].position;

                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) <
                            50) {
                            if (world.robotT[attackerNum].position.getDistanceTo(VecPosition(-2000, 1000)) > 100)
                                world.robotT[attackerNum].destination_position = VecPosition(-2000, 1000);
                            else
                                HighLevel::Shoot(attackerNum);
                        } else {
                            world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();
                        }
                    }
                } else {
                    if (world.robotT[attackerNum].position.getDistanceTo(intersection2) > 100) {
                        world.robotT[attackerNum].destination_position = intersection2;
                    } else {
                        //world.robotT[index].destination_position = world.robotT[index].position;

                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) <
                            50) {
                            if (world.robotT[attackerNum].position.getDistanceTo(VecPosition(-2000, 1000)) > 100)
                                world.robotT[attackerNum].destination_position = VecPosition(-2000, 1000);
                            else
                                HighLevel::Shoot(attackerNum);
                        } else {
                            world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();
                        }
                    }
                }
            }

            if (challengeNumber == 3) {
                int playingRobotNum = 5;
                if (world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].position.getDistanceTo(
                        world.ball.getCurrentBallPosition()) < 50 || ch3) {
                    ch3 = true;
                    VecPosition robot_pos = world.robotT[world.getIndexForRobotTNumber(playingRobotNum)].position;
                    VecPosition ball_pos = world.ball.getCurrentBallPosition();
                    VecPosition dest;
                    VecPosition *other_robots_pos;
                    VecPosition other_robots[world.numT - 1];
                    int last_robot_y = -FieldWidth / 2;
                    for (int i = 0; i < world.numT - 1; ++i) {
                        VecPosition robot_pos;
                        int min_y = 99999;
                        int min_i;
                        for (int j = 0; j < world.numT; ++j) {
                            if (world.getRobotTNumberForIndex(j) != playingRobotNum) {
                                VecPosition robot_pos_check = world.robotT[j].position;
                                if (robot_pos_check.getY() > last_robot_y && robot_pos_check.getY() < min_y) {
                                    min_y = robot_pos_check.getY();
                                    robot_pos = robot_pos_check;
                                    min_i = world.getRobotTNumberForIndex(j);
                                }
                            }
                        }
                        other_robots[i] = robot_pos;
                        last_robot_y = robot_pos.getY() + 50;
                    }
                    if (stage == -1) {
                        dest = ball_pos;
                        dest.setY(ball_pos.getY() - ROBOT_RADIUS);
                        HighLevel::lookAtPos(playingRobotNum, dest);
                        HighLevel::gotoXY(playingRobotNum, dest);
                        if (HighLevel::arivedToPos(playingRobotNum, dest)) stage++;
                    } else if (stage < world.numT - 1) {
                        dest = other_robots[stage];
                        if (stage % 2 == 0) dest.setX(dest.getX() + 500);
                        else dest.setX(dest.getX() - 500);
                        HighLevel::lookAtPos(playingRobotNum, dest);
                        HighLevel::gotoXY(playingRobotNum, dest);
                        if (HighLevel::arivedToPos(playingRobotNum, dest)) stage++;
                    } else HighLevel::gotoXY(playingRobotNum, robot_pos);
                } else {
                    world.robotT[world.getIndexForRobotTNumber(
                            playingRobotNum)].destination_position = world.ball.getCurrentBallPosition();
                }
            }

            if (challengeNumber == 4) {
                VecPosition robot_to_mid_bigest_hollVec = world.robotT[defenderNum].position - world.robotT[attackerNum].position;
                //DrawShape::DrawDot(mid_bigest_holl, 100, 255, 0, 0);
                //	DrawShape::DrawParaline(mid_bigest_holl, world.robotT[index_robotT].position);
                double angle_robot_to_mid_bigest_holl = -((robot_to_mid_bigest_hollVec).AngleBetween(VecPosition(1, 0)))*sign(world.robotT[attackerNum].position.getY() - world.robotT[defenderNum].position.getY());
                world.robotT[attackerNum].destination_angle=angle_robot_to_mid_bigest_holl;
                world.robotT[defenderNum].destination_position = VecPosition(-2000, 0);
                VecPosition intersection1, intersection2;
                Line ball_to_robotO = Line::makeLineFromTwoPoints(world.ball.getCurrentBallPosition(),
                                                                  world.robotT[defenderNum].position);
                Circle roboto = Circle(world.ball.getCurrentBallPosition(), 2 * ROBOT_RADIUS);
                ball_to_robotO.getCircleIntersectionPoints(roboto, &intersection1, &intersection2);
                if (world.robotT[defenderNum].position.getDistanceTo(intersection1) >
                    world.robotT[defenderNum].position.getDistanceTo(intersection2)) {
                    if (world.robotT[attackerNum].position.getDistanceTo(intersection1) > 40) {
                        world.robotT[attackerNum].destination_position = intersection1;
                    } else {
                        //world.robotT[index].destination_position = world.robotT[index].position;
                        world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();
                        world.robotT[attackerNum].shoot_or_chip = 1;
                        world.robotT[attackerNum].kick_power = 2;
                        if(world.robotT[defenderNum].position.getDistanceTo(world.ball.getCurrentBallPosition())<100)
                        {
                            passi= true;
                        }

                    }
                    //HighLevel::Pass(HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()),HighLevel::nearest_robot_to_point('T',world.ball.getCurrentBallPosition()));


                } else {
                    if (world.robotT[attackerNum].position.getDistanceTo(intersection2) > 40) {
                        world.robotT[attackerNum].destination_position = intersection2;
                    } else {
                        //world.robotT[index].destination_position = world.robotT[index].position;
                        world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();
                        world.robotT[attackerNum].shoot_or_chip = 1;
                        world.robotT[attackerNum].kick_power = 2;
                        if(world.robotT[defenderNum].position.getDistanceTo(world.ball.getCurrentBallPosition())<100)
                        {
                            passi= true;
                        }

                    }
                }
            }
            if (challengeNumber == 5) {

                world.robotT[world.getIndexForRobotTNumber(7)].destination_angle = M_PI / 4;
                world.robotT[world.getIndexForRobotTNumber(
                        7)].destination_position = world.robotT[world.getIndexForRobotTNumber(7)].position;

            }


           /* if(world.playMode==mode_State::Play) {
cout <<"stage"<<world.robotT[world.getIndexForRobotTNumber(7)].position.getDistanceTo(world.ball.getCurrentBallPosition())<<'\n';

                if (challengeNumber == 6) {
                    if (stage == -1) {
                        VecPosition robot_to_mid_bigest_hollVec =
                                world.ball.getCurrentBallPosition() - world.robotT[attackerNum].position;
                        double angle_robot_to_mid_bigest_holl =
                                -((robot_to_mid_bigest_hollVec).AngleBetween(VecPosition(1, 0))) *
                                sign(world.robotT[attackerNum].position.getY() -
                                     world.ball.getCurrentBallPosition().getY());
                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) <
                            300) {
                            if (abs(angle_robot_to_mid_bigest_holl - world.robotT[attackerNum].angle) <
                                20.0 * M_PI / 180) {
                                world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();
                            } else {
                                world.robotT[attackerNum].destination_position = world.robotT[attackerNum].position;
                            }
                        } else if (
                                world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) >=
                                300)
                            world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();

                        world.robotT[attackerNum].destination_angle = angle_robot_to_mid_bigest_holl;
                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) <
                            75) {
                            stage = 0;
                        }
                    }
                    if (stage == 0) {
                        world.robotT[attackerNum].destination_position = VecPosition( -1475.96, -21.7726 );
                        VecPosition robot_to_mid_bigest_hollVec1 =
                                Field::getGoalMidO() - world.robotT[attackerNum].position;
                        double angle_robot_to_mid_bigest_holl_goal =
                                -((robot_to_mid_bigest_hollVec1).AngleBetween(VecPosition(1, 0))) *
                                sign(world.robotT[attackerNum].position.getY() - Field::getGoalMidO().getY());
                        world.robotT[attackerNum].destination_angle = angle_robot_to_mid_bigest_holl_goal;
                        if (VecPosition( -1475.96, -21.7726 ).getDistanceTo(world.robotT[attackerNum].position) < 100) {
                            stage = 1;
                        }
                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) >
                            200) {
                            stage = 0;
                        }
                    }
                    if (stage == 1) {
                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) >
                            200) {
                            stage = 0;
                        }
                        VecPosition mid_bigest_holl;
                        Cone::Hole_Type Longest_Hole;
                        Cone BallToGoal(world.ball.getCurrentBallPosition(), Field::getUpBarO(), Field::getDownBarO());
                        VecPosition robot_to_mid_bigest_hollVec =
                                world.ball.getCurrentBallPosition() - world.robotT[attackerNum].position;
                        double angle_robot_to_mid_bigest_holl =
                                -((robot_to_mid_bigest_hollVec).AngleBetween(VecPosition(1, 0))) *
                                sign(world.robotT[attackerNum].position.getY() - mid_bigest_holl.getY());
                        if (BallToGoal.Get_Free_Space_In_Cone(robots, world.numT, Longest_Hole) > 0) {
                            mid_bigest_holl.setX((Longest_Hole.Point_1.getX() + Longest_Hole.Point_2.getX()) / 2);
                            mid_bigest_holl.setY((Longest_Hole.Point_1.getY() + Longest_Hole.Point_2.getY()) / 2);
                            if (abs(angle_robot_to_mid_bigest_holl - world.robotT[attackerNum].angle) <
                                20.0 * M_PI / 180) {
                                world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();
                                world.robotT[attackerNum].shoot_or_chip = true;
                                world.robotT[attackerNum].kick_power = 2;
                            } else {
                                world.robotT[attackerNum].destination_position = world.robotT[attackerNum].position;
                                world.robotT[attackerNum].kick_power = 0;
                            }
                            world.robotT[attackerNum].destination_angle = angle_robot_to_mid_bigest_holl;
                        }
                    }

                }

            }
            else */
           if(world.playMode==mode_State::ballPlacement)
            {
                if (challengeNumber == 6) {
                    if (stage == -1) {
                        VecPosition robot_to_mid_bigest_hollVec =
                                world.ball.getCurrentBallPosition() - world.robotT[attackerNum].position;
                        double angle_robot_to_mid_bigest_holl =
                                -((robot_to_mid_bigest_hollVec).AngleBetween(VecPosition(1, 0))) *
                                sign(world.robotT[attackerNum].position.getY() -
                                     world.ball.getCurrentBallPosition().getY());
                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) <
                            300) {
                            if (abs(angle_robot_to_mid_bigest_holl - world.robotT[attackerNum].angle) <
                                20.0 * M_PI / 180) {
                                world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();
                            } else {
                                world.robotT[attackerNum].destination_position = world.robotT[attackerNum].position;
                            }
                        } else if (
                                world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) >=
                                300)
                            world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();

                        world.robotT[attackerNum].destination_angle = angle_robot_to_mid_bigest_holl;
                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) <
                            60) {
                            stage = 0;
                        }
                    }
                    if (stage == 0) {
                        world.robotT[attackerNum].destination_position = world.ballPlacementPosition;
                        VecPosition robot_to_mid_bigest_hollVec1 =
                                Field::getGoalMidO() - world.robotT[attackerNum].position;
                        double angle_robot_to_mid_bigest_holl_goal =
                                -((robot_to_mid_bigest_hollVec1).AngleBetween(VecPosition(1, 0))) *
                                sign(world.robotT[attackerNum].position.getY() - Field::getGoalMidO().getY());
                        world.robotT[attackerNum].destination_angle = angle_robot_to_mid_bigest_holl_goal;
                        if (world.ballPlacementPosition.getDistanceTo(world.robotT[attackerNum].position) < 100) {
                            stage = 1;
                        }
                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) >
                            130) {
                            //stage = 0;
                        }
                    }
                    if (stage == 1) {
                        if (world.robotT[attackerNum].position.getDistanceTo(world.ball.getCurrentBallPosition()) >
                            200) {
                            stage = 0;
                        }
                        VecPosition mid_bigest_holl;
                        Cone::Hole_Type Longest_Hole;
                        Cone BallToGoal(world.ball.getCurrentBallPosition(), Field::getUpBarO(), Field::getDownBarO());
                        VecPosition robot_to_mid_bigest_hollVec =
                                world.ball.getCurrentBallPosition() - world.robotT[attackerNum].position;
                        double angle_robot_to_mid_bigest_holl =
                                -((robot_to_mid_bigest_hollVec).AngleBetween(VecPosition(1, 0))) *
                                sign(world.robotT[attackerNum].position.getY() - mid_bigest_holl.getY());
                        if (BallToGoal.Get_Free_Space_In_Cone(robots, world.numT, Longest_Hole) > 0) {
                            mid_bigest_holl.setX((Longest_Hole.Point_1.getX() + Longest_Hole.Point_2.getX()) / 2);
                            mid_bigest_holl.setY((Longest_Hole.Point_1.getY() + Longest_Hole.Point_2.getY()) / 2);
                            if (abs(angle_robot_to_mid_bigest_holl - world.robotT[attackerNum].angle) <
                                20.0 * M_PI / 180) {
                                world.robotT[attackerNum].destination_position = world.ball.getCurrentBallPosition();
                                world.robotT[attackerNum].shoot_or_chip = true;
                                world.robotT[attackerNum].kick_power = 4;
                            } else {
                                world.robotT[attackerNum].destination_position = world.robotT[attackerNum].position;
                                world.robotT[attackerNum].kick_power = 0;
                            }
                            world.robotT[attackerNum].destination_angle = angle_robot_to_mid_bigest_holl;
                        }
                    }

                }
            }
            else
            {
                world.robotT[attackerNum].destination_position=world.robotT[attackerNum].position;
                world.robotT[attackerNum].destination_angle=world.robotT[attackerNum].angle;
            }

        }
    //}
   // world.robotT[world.getIndexForRobotTNumber(4)].velocityToGo=VecPosition(800,0);
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