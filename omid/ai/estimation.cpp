#include "estimation.h"
#include "matrix.h"
#include "filter.h"
#include "Switches.h"
//bool Estimation::is_ball_static ;
Line Estimation::ballLine;
//VecPosition Estimation::ballDirection;
Ownership Estimation::ownership;
int Estimation::ownership_index ;
//VecPosition Estimation::ballVelocity ;

int Estimation::robotT_index[MAX_ROBOTS_IN_THE_FIELD];
int Estimation::robotO_index[MAX_ROBOTS_IN_THE_FIELD];



/*
 This method calculate the ball line and direction,
 Ball direction will return in the method,
 Ball line will place at the input argument as refrence -> (Line &ballLine)
 */

VecPosition Estimation::getBallDirection(World &world, Line &ballLine)
{
    //Regression Algorithm
    ballLine = Line(0,0,0);
    int n = MAX_BALLS - 1;
    double b = 0;//,a = 0;
    double sigmaX=0,sigmaY=0,sigmaXX=0,sigmaXY=0 ; //,distance=0;

    for(int i=0;i<n;i++)
    {
        sigmaX += world.ball.getOldBallPosition(i).getX()/1000;
        sigmaY += world.ball.getOldBallPosition(i).getY()/1000;
        sigmaXX += (world.ball.getOldBallPosition(i).getX()/1000)*(world.ball.getOldBallPosition(i).getX()/1000);
        sigmaXY += (world.ball.getOldBallPosition(i).getX()/1000)*(world.ball.getOldBallPosition(i).getY()/1000);
    }

    VecPosition average = VecPosition(sigmaX * 1000 /n, sigmaY * 1000 /n) ;
    bool is_static = true ;
    for(int i = 0; i < n; i++)
        is_static = (is_static) && (average.getDistanceTo(world.ball.getOldBallPosition(i)) < 15 ) ;
//    if(is_static)
//    {
//        //is_ball_static = true ;
////        ballVelocity = VecPosition(0, 0) ;
//        //ballDirection = VecPosition(0, 0) ;
//        return VecPosition(0, 0) ;
//    }
//    else
//        is_ball_static = false ;

    b = ((sigmaXY) - (sigmaX*sigmaY)/n)/(sigmaXX-(sigmaX*sigmaX)/n);
    //a = (sigmaY-(b*sigmaX))/(float)n; // Dont Need
    //The equation is y=bx+a
    //Support::setStrings(7,0);
    double angDeg = (Rad2Deg(atan(b)) > 0) ? Rad2Deg(atan(b)) : 180 + Rad2Deg(atan(b));
    //Support::setStrings(6,angDeg);

    //ballLine = Line::makeLineFromPositionAndAngle(world.ball.getCurrentBallPosition() ,angDeg);
    ballLine = Line::makeLineFromPositionAndAngle(average ,angDeg);

    Line tempLine = Line::makeLineFromPositionAndAngle(VecPosition(0,0),angDeg);

    Circle c;
    c.setCircle(VecPosition(0,0),1000);
    VecPosition intersect0(0,0),intersect1(0,0);
    tempLine.getCircleIntersectionPoints(c,&intersect0,&intersect1);
    //Support::setVecPositions(0,intersect1);
    VecPosition ballDirection = world.ball.getCurrentBallPosition() - world.ball.getOldBallPosition(n) ;
    double angle0,angle1;
    angle0 = ballDirection.AngleBetween(intersect0);
    angle1 = ballDirection.AngleBetween(intersect1);

    //Support::setStrings(2,angle0);
    //Support::setStrings(3,angle1);
    // Support::setVecPositions(0,intersect1);
    if(angle0 < angle1)
    {
        ballDirection = intersect0.normalize() ;
//        ballVelocity = ballDirection * ballVelocity.getMagnitude() ;
        return intersect0;
    }
    else
    {
        ballDirection = intersect1.normalize() ;
//        ballVelocity = ballDirection * ballVelocity.getMagnitude() ;
        return intersect1;
    }


}

int Estimation::get_nearest_T(World &world,VecPosition point) // NOTE: The out put is the index of the robot
{
    float _temp , min = 100000;
    int out = 0;
    for ( int i = 0 ; i < world.numT ; i++ )
    {
        _temp = point.getDistanceTo(world.robotT[i].position);
        if ( _temp < min )
        {
            min = _temp;
            out = i;
        }
    }
    return out;
}

int Estimation::get_nearest_O(World &world,VecPosition point) // NOTE: The out put is the index of the robot
{
    float _temp , min = 100000;
    int out = 0;
    for ( int i = 0 ; i < world.numO ; i++ )
    {
        _temp = point.getDistanceTo(world.robotO[i].position);
        if ( _temp < min )
        {
            min = _temp;
            out = i;
        }
    }
    return out;
}



//int Estimation::NearestO(World &world)
//{
//    VecPosition v1 = world.ball.position ;
//    VecPosition v2 = world.robotO[0].position ;
//    double d = v1.getDistanceTo(v2) ;
//    double d1;
//    int i=0;
//    for(int j=1;world.numO;j++)
//    {
//        d1 = v1.getDistanceTo(world.robotO[j].position) ;
//        if(d1<d)
//        {
//            d = d1 ;
//            i = j ;
//        }
//    }
//    return i ;
//}
//int Estimation::NearestP(World &world)
//{
//    VecPosition v1 = world.ball.position ;
//    VecPosition v2 = world.robotT[0].position ;
//    double d = v1.getDistanceTo(v2) ;
//    double d1 ;
//    int i=0;
//    for(int j=0;world.numT;j++)
//    {
//        d1 = v1.getDistanceTo(world.robotT[j].position) ;
//        if(d1<d)
//        {
//            d = d1 ;
//            i = j ;
//        }
//    }
//    return i ;
//}

//int Estimation::TimeToBall(Robot robot, double v , World &world)
//{

//    VecPosition P = robot.position;
//    double t1=0, t2=0 ;
//    VecPosition v1 = world.ball.position;
//    VecPosition v2 = world.ball.velocity;
//    if(pow(v1.getMagnitude()*cos(v2.AngleBetween((P-v1))),2)
//        -pow(v2.getMagnitude(),2)+pow(v,2) > 0)
//        {
//        if(v2.getMagnitude() == v)
//        {}
//        else
//        {
//            t1 = (P-v1).getMagnitude()*0.5*(-2*v2.getMagnitude()*
//                                            cos(v2.AngleBetween(P-v1))+2*
//                                            sqrt(pow(v2.getMagnitude()*cos(v2.AngleBetween(P-v1)),2)-
//                                                 pow(v2.getMagnitude(),2)+pow(v,2)))
//                 /(pow(v,2)-pow(v2.getMagnitude(),2));

//            t2 = (P-v1).getMagnitude()*0.5*(-2*v2.getMagnitude()*
//                                            cos(v2.AngleBetween(P-v1))-2*
//                                            sqrt(pow(v2.getMagnitude()*cos(v2.AngleBetween(P-v1)),2)-
//                                                 pow(v2.getMagnitude(),2)+pow(v,2)))
//                 /(pow(v,2)-pow(v2.getMagnitude(),2));
//        }
//        if(t1 > 0 && t2 < 0) return t1;
//        if(t1 < 0 && t2 > 0) return t2;
//        if(t1 > 0 && t2 > 0 && t1 <= t2) return t1 ;
//        if(t1 > 0 && t2 > 0 && t1 > t2) return t2;
//    }
//    return 1000;
//}

//int Estimation::FastestO(double v,World &world)
//{
//    int i=0 ;
//    double t = TimeToBall(world.robotO[0],v,world) ;
//    double t1 ;
//    for (int j=1;world.numO;j++)
//    {
//        t1 = TimeToBall(world.robotO[j],v,world);
//        if(t1 < t)
//        {
//            t = t1 ;
//            i = j ;
//        }
//        return i ;
//    }
//}

//int Estimation::FastestP(double v, World &world)
//{
//    int i=0 ;
//    double t = TimeToBall(world.robotT[0],v,world) ;
//    double t1 ;
//    for (int j=1;world.numT;j++)
//    {
//        t1 = TimeToBall(world.robotT[j],v,world);
//        if(t1 < t)
//        {
//            t = t1 ;
//            i = j ;
//        }
//        return i ;
//    }
//}

//VecPosition Estimation::ReciveBallPoint(Robot robot, double v, World &world)
//{
//    //return world.ball.velocity * TimeToBall(robot,v,world) + world.ball.position ;
//}

//void Estimation::GetDistacetoPoint(World &world, VecPosition point ,double distace[MAX_OPPONENT + MAX_ROBOTS_IN_THE_FIELD],int ids[MAX_OPPONENT + MAX_ROBOTS_IN_THE_FIELD],char TeamateOrOpponet[MAX_OPPONENT + MAX_ROBOTS_IN_THE_FIELD])
//{
//    double temp = 0;
//    char tempc;
//    int i;
//    for(i=0;i < world.numT;i++)
//    {
//        ids[i] = world.robotT[i].id;
//        distace[i] = point.getDistanceTo(world.robotT[i].position);
//        TeamateOrOpponet[i] = 'T';
//    }
//    for(int j=0;j < world.numO ;j++,i++)
//    {
//        ids[i] = world.robotO[j].id;
//        distace[i] = point.getDistanceTo(world.robotO[j].position);
//        TeamateOrOpponet[i] = 'O';
//    }
//    for(int k=0;k < i ;k++)
//        for(int j=k+1;j < i ;j++)
//            if(distace[k] > distace[j])
//            {
//                temp = distace[k];
//                distace[k] = distace[j];
//                distace[j] = temp;
//                temp = ids[k];
//                ids[k] = ids[j];
//                ids[j] = temp;
//                tempc = TeamateOrOpponet[k];
//                TeamateOrOpponet[k] = TeamateOrOpponet[j];
//                TeamateOrOpponet[j] = tempc;
//            }
//}

int Estimation::Partition(double *_array1, int *_array2, int *_array3, int _start, int _end, bool is_decending)
{
    int i,j;
    float x = _array1[_end];
    i = _start - 1;
    for( j = _start ; j < _end ; j++ )
        if ( (!is_decending && (_array1[j] < x )) || (is_decending && (_array1[j] > x )) )
        {
        i++;
        swap(_array1[i],_array1[j]);
        swap(_array2[i],_array2[j]);
        swap(_array3[i],_array3[j]);
    }
    if(_array1[++i] != _array1[_end])
    {
        swap(_array1[i],_array1[_end]);
        swap(_array2[i],_array2[_end]);
        swap(_array3[i],_array3[_end]);
    }
    return i;
}

void Estimation::QuickSort(double *_array1, int *_array2, int *_array3, int _start, int _end, bool is_decending)
{
    int _pivot;
    if(_start < _end)
    {
        _pivot = Partition(_array1,_array2,_array3,_start,_end, is_decending);
        QuickSort(_array1,_array2,_array3,_start,_pivot - 1, is_decending);
        QuickSort(_array1,_array2,_array3,_pivot + 1,_end, is_decending);
    }

}

void Estimation::GetDistaceTtoPoint(World &world,VecPosition point ,double distace[MAX_ROBOTS_IN_THE_FIELD],int ids[MAX_ROBOTS_IN_THE_FIELD],int index[MAX_ROBOTS_IN_THE_FIELD], bool is_decending )
{
    for ( int i = 0 ; i < world.numT ; i++ )
    {
        distace[i] = point.getDistanceTo( world.robotT[i].position ) ;
        ids[i] = world.robotT[i].id ;
        index[i] = i ;
    }
    QuickSort(distace, ids, index, 0, world.numT-1 ,is_decending);
}

void Estimation::GetDistaceTtoLine(World &world,Line target ,double distace[MAX_ROBOTS_IN_THE_FIELD],int ids[MAX_ROBOTS_IN_THE_FIELD],int index[MAX_ROBOTS_IN_THE_FIELD])
{
    for ( int i = 0 ; i < world.numT ; i++ )
    {
        distace[i] = target.getDistanceWithPoint( world.robotT[i].position ) ;
        ids[i] = world.robotT[i].id ;
        index[i] = i ;
    }
    QuickSort(distace, ids, index, 0, world.numT-1 );
}

void Estimation::GetDistaceOtoPoint(World &world,VecPosition point ,double distace[MAX_ROBOTS_IN_THE_FIELD],int ids[MAX_ROBOTS_IN_THE_FIELD], int index[MAX_ROBOTS_IN_THE_FIELD] = 0 )
{
    //double temp = 0;
    for(int i=0;i < world.numO;i++)
    {
        index[i] = i ;
        ids[i] = world.robotO[i].id;
        distace[i] = point.getDistanceTo(world.robotO[i].position);
    }
//    for(int i=0;i<world.numO;i++)
//        for(int j=i+1;j<world.numO;j++)
//            if(distace[i] > distace[j])
//            {
//                temp = distace[i];
//                distace[i] = distace[j];
//                distace[j] = temp;
//                temp = ids[i];
//                ids[i] = ids[j];
//                ids[j] = temp;
//            }
    QuickSort(distace, ids, index, 0, world.numO-1 );
}

//int Estimation::NthNearestP(World &world, VecPosition point, int n)
//{
//    double a[MAX_ROBOTS_IN_THE_FIELD] ;
//    int b[MAX_ROBOTS_IN_THE_FIELD] ;
//    int index[MAX_ROBOTS_IN_THE_FIELD] ;
//    GetDistaceTtoPoint(world,point , a , b,index);
//    n = ( n >= world.numT ? world.numT - 1 : n ) ;
//    return b[n];
//}

//int Estimation::NthNearestO(World &world, VecPosition point, int n)
//{
//    double a[MAX_OPPONENT] ;
//    int b[MAX_OPPONENT] ;
//    int index[MAX_ROBOTS_IN_THE_FIELD] ;
//    GetDistaceOtoPoint(world,point,a,b);
//    n = (n >= world.numO ? world.numO - 1 : n );
//    return b[n];
//}

int Estimation::getrobotTIndex(int _id)
{
    return robotT_index[_id];
}

int Estimation::getRobotOIndex(int _id)
{
    return robotO_index[_id];
}

bool Estimation::isDirectLineBetween(World &world, VecPosition vec1, VecPosition vec2, double minimum_distance)
{
    int output = true ;
    Line my_line = Line::makeLineFromTwoPoints(vec1, vec2) ;

    for(int i = 0; i < world.numO; i++)
        if( (my_line.getDistanceWithPoint(world.robotO[i].position) < minimum_distance) && (world.robotO[i].position.isBetween(vec1, vec2)) && (world.robotO[i].position.getDistanceTo(vec1) > 100) && (world.robotO[i].position.getDistanceTo(vec2) > 100) )
            output = false ;

    for(int i = 0; i < world.numT; i++)
        if( (my_line.getDistanceWithPoint(world.robotT[i].position) < minimum_distance) && (world.robotT[i].position.isBetween(vec1, vec2))  && (world.robotT[i].position.getDistanceTo(vec1) > 100) && (world.robotT[i].position.getDistanceTo(vec2) > 100) )
            output = false ;

    return output ;
}

bool Estimation::isBallGettingCloserToPoint(World &world, VecPosition target)
{
    int n = MAX_BALLS - 1;
    if(world.ball.getCurrentBallPosition().getDistanceTo(target) < world.ball.getOldBallPosition(n).getDistanceTo(target))
        return true;
    return false;
}

bool Estimation::isBallGettingCloserToLine(World &world, Line target)
{
    int n = MAX_BALLS-1;
    if(target.getDistanceWithPoint(world.ball.getCurrentBallPosition()) < target.getDistanceWithPoint(world.ball.getOldBallPosition(n)))
        return true;
    return false;
}






int Estimation::getOwnership_team()
{
    return ownership;
}
int Estimation::getOwnership_robot_index()
{
    return ownership_index;
}

enum Field_Area
{
    fa_teamate      ,
    fa_midle        ,
    fa_opponent     ,

    fa_none
}ball_area ;

int Estimation::set_ball_area(World&world)
{
    int border = 0 ;
    Field_Area up_area  = fa_none ;
    Field_Area down_area = fa_none ;

    int a = world.ball.getCurrentBallPosition().getX() - (int)(FieldLength / 6) ;
    int b = world.ball.getCurrentBallPosition().getX() + (int)(FieldLength / 6) ;

    if(fabs(a) < 250)
    {
        border = (int)(FieldLength / 6) ;
        up_area = fa_teamate ;
        down_area = fa_midle ;
    }else if(fabs(b) < 250)
    {
        border = (int)(-FieldLength / 6) ;
        up_area = fa_midle ;
        down_area = fa_opponent ;
    }

    if( (border == 0) || ( (ball_area != up_area) && (ball_area != down_area) ) )
    {
        if(a > 0)
            ball_area = fa_teamate ;
        else if( (a < 0) && (b > 0) )
            ball_area = fa_midle ;
        else if(b < 0)
            ball_area = fa_opponent ;
    }
    return 0;

}


void Estimation::UpdateOwnership(World &world)
{
    double far_time_from_ball_to_nearer_opponent_robot = getFarTimeToBall(world.robotO[0].position, world) ;
    int nearer_opponent_index = 0;
    if(world.numO > 1)
    {
        for(int i = 1; i < world.numO; i++)
        {
            if( far_time_from_ball_to_nearer_opponent_robot > getFarTimeToBall(world.robotO[i].position, world))
            {
                far_time_from_ball_to_nearer_opponent_robot = getFarTimeToBall(world.robotO[i].position, world) ;
                nearer_opponent_index = i ;
            }
        }
    }

    double far_time_from_ball_to_nearer_teamate_robot = getFarTimeToBall(world.robotT[0].position, world) ;
    int nearer_teamate_index = 0;
    if(world.numT > 1)
    {
        for(int i = 0; i < world.numT; i++)
        {
            if( far_time_from_ball_to_nearer_teamate_robot > getFarTimeToBall(world.robotT[i].position, world) )
            {
                far_time_from_ball_to_nearer_teamate_robot = getFarTimeToBall(world.robotT[i].position, world) ;
                nearer_teamate_index = i ;
            }
        }
    }
    set_ball_area(world);

    if(ball_area == fa_teamate)
    {
        distance_limit_t = 90 + 300;
        distance_limit_o = 90 + 500;

        counter_step = 1;
    }
    if(ball_area == fa_midle)
    {
        distance_limit_t = 90 + 400;
        distance_limit_o = 90 + 400;

        counter_step = 1.5;
    }
    if(ball_area == fa_opponent)
    {
        distance_limit_t = 90 + 400;
        distance_limit_o = 90 + 500;

        counter_step = 2;
    }

    if(far_time_from_ball_to_nearer_opponent_robot < distance_limit_o )
    {
        counter_toword = -1;

    }
    else if(far_time_from_ball_to_nearer_teamate_robot < distance_limit_t )
    {
        counter_toword = 1;
    }
    else
    {
        counter_toword = 0;
    }


    if(counter_toword == 0)
    {
        if(ownershipCounter > COUNTER_LIMIT/2)
            ownershipCounter -= counter_step ;
        else if(ownershipCounter < COUNTER_LIMIT/2)
            ownershipCounter += counter_step ;
    }
    else if(counter_toword == 1)
    {
        if(ownershipCounter < COUNTER_LIMIT)
            ownershipCounter += counter_step ;
    }
    else if(counter_toword == -1)
    {
        if(ownershipCounter > 0)
            ownershipCounter -= counter_step ;
    }


    if(ownershipCounter < COUNTER_LIMIT*2/5)
    {
        ownership = Opponent;
        if(counter_toword == -1)
            ownership_index = nearer_opponent_index;
    }
    else if(ownershipCounter > COUNTER_LIMIT*3/5)
    {
        ownership = Teamate;
        if(counter_toword == 1)
            ownership_index = nearer_teamate_index;
    }
    else
    {
        ownership = NoOwnership;
        ownership_index = -1;
    }

}
double Estimation::getFarTimeToBall(VecPosition Me, World &world)
{

    if(world.ball.getVelocity().getMagnitude() < 500)
        return world.ball.getCurrentBallPosition().getDistanceTo(Me) ;

    VecPosition closest_to_me = ballLine.getPointOnLineClosestTo(Me) ;
    if( world.ball.getVelocity().AngleBetween(closest_to_me - world.ball.getCurrentBallPosition()) < M_PI/18 )
        return sqrt( pow(Me.getDistanceTo(closest_to_me), 2) + pow( world.ball.getCurrentBallPosition().getDistanceTo(closest_to_me) / ( world.ball.getVelocity().getMagnitude() ), 2) );
    else
        return sqrt( pow(Me.getDistanceTo(closest_to_me), 2) + pow( world.ball.getCurrentBallPosition().getDistanceTo(closest_to_me) * ( world.ball.getVelocity().getMagnitude() ), 2) );

}




double Estimation::MM_angle_robot_to_bigest_hole(World &, VecPosition target, Cone::Hole_Type bigest_hole, bool our_treat)
{
    if(bigest_hole.Point_1 == VecPosition(0,0) && bigest_hole.Point_2 == VecPosition(0,0))
        return 0;
    AngDeg _tem_ang = Rad2Deg((bigest_hole.Point_1 - target).AngleBetween(bigest_hole.Point_2 - target)) ;
    if(_tem_ang < 5)
        return 0;
    if(_tem_ang < 20)
        return ((_tem_ang - 5) / (float)(20 - 5)) * 100;
    if(_tem_ang < 40)
        return ((40 - _tem_ang) / (float)(40 - 20)) * 100;
    return 0;
}

double Estimation::MM_angle_between_robot_to_ball_and_robot_to_goal(World &world, VecPosition target, bool our_treat)
{

    AngDeg _tem_ang = Rad2Deg(((our_treat ? Field::getGoalMidP() : Field::getGoalMidO()) - target).AngleBetween(world.ball.getCurrentBallPosition() - target)) ;
    if(_tem_ang < 15)
        return 0;
    if(_tem_ang < 40)
        return ((_tem_ang - 15) / (float)(40 - 15)) * 100;
    if(_tem_ang < 90)
        return ((90 - _tem_ang) / (float)(90 - 40)) * 100;
    return ((90 - _tem_ang) / (float)(180 - 90)) * 100;
}
double Estimation::MM_distance_to_ball(World &world, VecPosition target, bool our_treat)
{
    double _tem_dist = world.ball.getCurrentBallPosition().getDistanceTo(target);
    double max_distance = 2000;

    if(_tem_dist < 500)
        return 0;
    if(_tem_dist < max_distance)
        return ((_tem_dist - 500) / (float)(max_distance - 500)) * 100;
    if(_tem_dist < 5000)
        return ((5000 - _tem_dist) / (float)(5000 - max_distance)) * 100;
    return 0;
}
double Estimation::MM_nearest_obs_to_Line_ball_to_robot(World &world, VecPosition target, int decider_index, bool our_treat)
{
    Paraline ball_to_robot = Paraline(target, world.ball.getCurrentBallPosition()) ;
    double distance = 10101010 ;
    double temp ;
    int nearest_t_to_ball_index = Estimation::get_nearest_T(world,world.ball.getCurrentBallPosition());

    for(int i = 0; i < world.numO; i++)
    {
        if((i == decider_index) && our_treat) continue ;
        if(! world.robotO[i].position.isBetween(target, world.ball.getCurrentBallPosition()) ) continue ;
        temp = ball_to_robot.getDistanceTo(world.robotO[i].position) ;
        if(temp < distance)
            distance = temp ;
    }
    for(int i = 0; i < world.numT; i++)
    {
        if((i == decider_index || i == nearest_t_to_ball_index) && !our_treat) continue ;
        if(! world.robotT[i].position.isBetween(target, world.ball.getCurrentBallPosition()) ) continue ;
        temp = ball_to_robot.getDistanceTo(world.robotT[i].position) ;
        if(temp < distance)
            distance = temp ;
    }

    double min = 150.0 ;
    double max = 500.0 ;
    if(distance < min)
        return 0 ;
    if(distance < max)
        return ( (distance - min) / (float)( max - min ) ) * 100 ;
    return 100 ;
}
double Estimation::MM_nearest_obs_to_Line_robot_to_goal(World &world, VecPosition target, int decider_index, Cone::Hole_Type bigest_hole, bool our_treat, int team_index1, int team_index2)
{
    if(bigest_hole.Point_1 == VecPosition(0,0) && bigest_hole.Point_2 == VecPosition(0,0)) return 0;
    Paraline ball_to_robot = Paraline(target, (bigest_hole.Point_1 + bigest_hole.Point_2 ) / 2.0 ) ;
    double distance = 10101010 ;
    double temp ;

    int index_goali_opp = Estimation::get_nearest_O(world,Field::getGoalMidO());
    int index_goali_t = Estimation::get_nearest_T(world,Field::getGoalMidP());
    for(int i = 0; i < world.numO; i++)
    {
        if((i == decider_index) && our_treat) continue ;
        if((i == index_goali_opp) && !our_treat)continue ;
        if(! world.robotO[i].position.isBetween(target, (bigest_hole.Point_1 + bigest_hole.Point_2 ) / 2.0 ) ) continue ;
        temp = ball_to_robot.getDistanceTo(world.robotO[i].position) ;
        if(temp < distance)
            distance = temp ;
    }
    for(int i = 0; i < world.numT; i++)
    {
        if ( ( (i == team_index1) || (i == team_index2) )&& our_treat) continue ;
        if((i == decider_index) && !our_treat) continue ;
        if((i == index_goali_t) && our_treat)continue ;
        if(! world.robotT[i].position.isBetween(target, (bigest_hole.Point_1 + bigest_hole.Point_2 ) / 2.0 ) ) continue ;
        temp = ball_to_robot.getDistanceTo(world.robotT[i].position) ;
        if(temp < distance)
            distance = temp ;
    }

    double min = 150.0 ;
    double max = 200.0 ;
    if(distance < min)
        return 0 ;
    if(distance < max)
        return ( (distance - min) / (float)( max - min ) ) * 100 ;
    return 100 ;
}

double Estimation::MM_distance_to_goal(World &world, VecPosition target, bool our_treat )
{
    double dist = target.getDistanceTo((our_treat ? Field::getGoalMidP() : Field::getGoalMidO()));
    double range[6] , mark [6];
//    range[ 0 ] = 500; mark[ 0 ] = 100;
//    range[ 1 ] = 2500;mark[ 1 ] = 70;
//    range[ 2 ] = 5000;mark[ 2 ] = 0;
//    range[ 3 ] = 6500;mark[ 3 ] = -100;

    range[ 0 ] = 550    ; mark[ 0 ] = 0   ;
    range[ 1 ] = 600    ; mark[ 1 ] = 60   ;
    range[ 2 ] = 1000   ; mark[ 2 ] = 100   ;
    range[ 3 ] = 2000   ; mark[ 3 ] = 60 ;
    range[ 4 ] = 4500   ; mark[ 4 ] = 0 ;
    range[ 5 ] = 6500   ; mark[ 5 ] = -100;

    return getMarkOfMathematicalEquation( range, mark, 6, dist );
}

//double Estimation::MM_distance_to_nearest_make_situation_NEGATIVE(World &world, VecPosition target, int decider_index, PlayerDecider *deciders, bool our_treat)
//{
//    double distance = 1000000;
//    double _temp ;
//    for(int i = 0; i < world.numT; i++)
//    {
//        if(deciders[i].index == decider_index) continue;
//        if(deciders[i].currentHighLevel == "MakeSituation")
//        {
//            _temp = world.robotT[deciders[i].index].position.getDistanceTo(target) ;
//            if(_temp < distance)
//                distance = _temp ;
//        }
//    }
//    if(distance < 700)
//        return -1000;
//    if(distance < 2000)
//        return -(( 2000 - distance )/(float)(2000 - 700)) * 100;
//    return 0;
//}

double Estimation::getMarkOfMathematicalEquation(double *range, double *mark, int num, double input)
{
    if ( input < range[ 0 ] ) return mark[ 0 ] ;
    if ( input > range[ num - 1 ] ) return mark[ num - 1 ] ;

    for( int i = 1; i < num; i++ )
        if( input <= range[i] )
            return getMarkOfMathematicalEquation( range[ i - 1 ], range[i], mark[ i - 1 ], mark[ i ], input ) ;
}

double Estimation::getMarkOfMathematicalEquation(double sRange, double eRange, double sMark, double eMark, double input)
{
    if ( input < sRange ) return sMark;
    if ( input > eRange ) return eMark;

    return ( ( ( input - sRange ) * ( eMark - sMark ) ) / ( double ) ( eRange - sRange ) ) + sMark;
}


void Estimation::RUN(World &world)
{

    this->UpdateBallstatus(world);
    //ali khanjani
//    VecPosition ballDirection;
//    ballDirection = this->getBallDirection(world,ballLine) ;
//    double a[2];
//    a[0]=ballDirection.getX();
//    a[1]=ballDirection.getY();
//    logStatus(string::number(Rad2Deg(atan(a[1]/a[0]))),"brown");
    //ali khanjani
    this->UpdateOwnership(world) ;
    look_up_table_index(world) ;
    updateRobotsVelocities( world );
}
