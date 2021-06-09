 #ifndef ESTIMATION_H
#define ESTIMATION_H
#include "world.h"
#include "timer.h"
#include "geometry.h"
#include "matrix.h"
#include "filter.h"
#include "Switches.h"
enum Ownership
{
    NoOwnership = 0,
    Opponent = 1,
    Teamate = 2
} ;

#define OWNER_LIMIT 500 //90 in the own robot
#define COUNTER_LIMIT 400 //Equal about 3 sec

const int VeloNum = 5;

class Estimation
{
public:
    //static bool             is_ball_static ;
    static Line             ballLine ;
    //static VecPosition      ballDirection ;
    //static VecPosition      ballVelocity ;
    static double           ballAcceleration ;
    static Ownership        ownership ;
    static int              ownership_index ;
    double           ownershipCounter  ;
    double           counter_step;
    double           counter_toword;
    double           distance_limit_t;
    double           distance_limit_o;

    Timer es_timr;

    VecPosition robotTvelocity[MAX_ROBOTS_IN_THE_FIELD];
    VecPosition robotTlastPos[MAX_ROBOTS_IN_THE_FIELD][100 /*VeloNum + 1*/ ];
    double robotTlastTime[MAX_ROBOTS_IN_THE_FIELD][ 100 /*VeloNum + 1*/ ];
    int robtoTNumOfSaved[MAX_ROBOTS_IN_THE_FIELD];

    void updateRobotsVelocities(World &world)
    {
        for ( int i = 0; i < world.numT; i++ )
        {
            for ( int j = robtoTNumOfSaved[i]; j >= 0 ; j-- )
            {
                robotTlastPos[ i ][ j ] = robotTlastPos[ i ][ j - 1 ];
                //logStatus("lastpos="+string::number(robotTlastPos[ i ][0 ].getMagnitude()));
                robotTlastTime[ i ][ j ] = robotTlastTime[ i ][ j - 1 ];
            }

            robotTlastPos[ i ][ 0 ] = world.robotT[ i ].position ;
            robotTlastTime[ i ][ 0 ] = world.robotT[ i ].timeCaptured ;

            if ( robtoTNumOfSaved[i] < VeloNum )
            {
                robtoTNumOfSaved[i]++;
            }

            world.robotT[ i ].velocity = ( ( robotTlastPos[ i ][ 0 ] - robotTlastPos[ i ][ robtoTNumOfSaved[i] - 1 ] ) /
                                           ( robotTlastTime[ i ][ 0 ] - robotTlastTime[ i ][ robtoTNumOfSaved[i] - 1 ] ) );

//            if ( i == 0 )
//            logStatus( string::number(world.robotT[ i ].velocity.getMagnitude()), QColor("blue") );
            if ( world.robotT[ i ].velocity.isNan() )
            {
                world.robotT[ i ].velocity = VecPosition( 0, 0 );
               // cout<<"\n :| :| nan";
            }

            world.robotT[ i ].velocity.setMagnitude( (double) ( world.robotT[ i ].velocity.getMagnitude() ) * ( 900.011f / 843.011f ) );

        }
        //GLFrame::addTextToPainting( VecPosition( (FieldLength/2), 2075 ), string::number( world.robotT [ Estimation::getrobotTIndex( 1 ) ].velocity.getMagnitude() ) ) ;



        es_timr.start();
    }


    static int robotT_index[MAX_ROBOTS_IN_THE_FIELD];
    static int robotO_index[MAX_ROBOTS_IN_THE_FIELD];
	void look_up_table_index(World &world)
	{
		int i;
		for (i = 0; i < world.numT; i++)
			robotT_index[world.robotT[i].id] = i;

		for (i = 0; i < world.numO; i++)
			robotO_index[world.robotO[i].id] = i;
	}

    Timer time ;
    double oldTime[MAX_BALLS + 1]  ;
	void UpdateBallstatus(World &world)
	{
		//    VecPosition vel;
		//    for(int i=BALL_OLD_POS_NUM - 1;i>=0;i--)
		//    {
		//        oldTime[i+1] = oldTime[i];
		//    }
		//    double duration = time.stop();
		//    oldTime[0] = duration;
		//    for(int i=0;i<BALL_OLD_POS_NUM - 1;i++)
		//    {
		//        vel += (world.ball.getOldBallPosition(i) - world.ball.getOldBallPosition(i+1))/oldTime[i];
		//    }

		//ballVelocity = (world.ball.getOldBallPosition(0) - world.ball.getOldBallPosition(BALL_OLD_POS_NUM - 1)) / (world.ball.getOldBallTime(0) - world.ball.getOldBallTime(BALL_OLD_POS_NUM - 1));
		ballLine = Line::makeLineFromPositionAndAngle(world.ball.getCurrentBallPosition(), Rad2Deg(world.ball.velocity.Angle()));

	}



    static int getOwnership_team() ;
    static int getOwnership_robot_index() ;
    void UpdateOwnership(World &world) ;
    double getFarTimeToBall(VecPosition Me, World &world);
    int set_ball_area(World&world);
    Estimation()
    {
        for ( int i = 0; i < MAX_ROBOTS_IN_THE_FIELD; i++ )
            robtoTNumOfSaved[i] = 0;
        ownershipCounter = COUNTER_LIMIT / 2 ;
        //is_ball_static = true ;
        ballLine = Line(0,0,0) ;
        //ballDirection = VecPosition(0,0) ;
    }

    static int Partition(double* _array1, int* _array2, int* _array3, int _start, int _end, bool is_decending = false);
    static void QuickSort(double* _array1, int* _array2, int* _array3, int _start, int _end, bool is_decending = false);

    static int get_nearest_T(World &world,VecPosition point) ;
    static int get_nearest_O(World &world,VecPosition point) ;
    static int TimeToBall(Robot robot,double v,World &world)  ;
    static int FastestO(double v,World &world) ;
    static int FastestP(double v,World &world) ;
    static VecPosition ReciveBallPoint(Robot robot,double v,World &world)  ;

    static void GetDistacetoPoint(World &world, VecPosition point ,double distace[MAX_ROBOTS_IN_THE_FIELD + MAX_ROBOTS_IN_THE_FIELD],int ids[MAX_ROBOTS_IN_THE_FIELD + MAX_ROBOTS_IN_THE_FIELD],char TeamateOrOpponet[MAX_ROBOTS_IN_THE_FIELD + MAX_ROBOTS_IN_THE_FIELD]) ;
    static void GetDistaceTtoPoint(World &world,VecPosition point ,double distace[MAX_ROBOTS_IN_THE_FIELD],int ids[MAX_ROBOTS_IN_THE_FIELD],int index[MAX_ROBOTS_IN_THE_FIELD], bool is_decending = false );
    static void GetDistaceOtoPoint(World &world,VecPosition point ,double distace[MAX_ROBOTS_IN_THE_FIELD],int ids[MAX_ROBOTS_IN_THE_FIELD],int index[MAX_ROBOTS_IN_THE_FIELD]) ;

    static void GetDistaceTtoLine(World &world,Line target ,double distace[MAX_ROBOTS_IN_THE_FIELD],int ids[MAX_ROBOTS_IN_THE_FIELD],int index[MAX_ROBOTS_IN_THE_FIELD]) ;

    static int getrobotTIndex(int _id) ;
    static int getRobotOIndex(int _id) ;
    static bool isDirectLineBetween(World &world, VecPosition vec1, VecPosition vec2, double minimum_distance = 100) ;
    static bool isBallGettingCloserToPoint(World &world, VecPosition target) ;   ///// static nemishe chikaresh konim
    static  bool isBallGettingCloserToLine(World &world,Line target) ;
    VecPosition getBallDirection(World &world,Line &ballLine) ;



    /// Marking Methods
    static double MM_angle_robot_to_bigest_hole(World &world, VecPosition target, Cone::Hole_Type bigest_hole, bool our_treat = false) ;
    static double MM_angle_between_robot_to_ball_and_robot_to_goal(World &world, VecPosition target, bool our_treat = false) ;
    static double MM_distance_to_ball(World &world, VecPosition target, bool our_treat = false) ;
    static double MM_nearest_obs_to_Line_ball_to_robot(World &world, VecPosition target, int decider_index, bool our_treat = false) ;
    static double MM_nearest_obs_to_Line_robot_to_goal(World &world, VecPosition target, int decider_index, Cone::Hole_Type bigest_hole, bool our_treat = false, int team_index1 = -1, int team_index2 = -1) ;
    static double MM_distance_to_goal(World &world, VecPosition target, bool our_treat = false) ;
   /* static double MM_distance_to_nearest_make_situation_NEGATIVE(World &world, VecPosition target, int decider_index, PlayerDecider *deciders, bool our_treat = false) ;
*/

    static double getMarkOfMathematicalEquation( double *range, double *mark, int num, double input );
    static double getMarkOfMathematicalEquation( double sRange, double eRange, double sMark, double eMark, double input );


    void RUN(World &world) ;
} ;




#endif // ESTIMATION_H
