#include "Vision.h"
#include "Switches.h"

void Vision::ProcessBalls ( World &world )
{

	//First we have to extract the balls!
	balls_num = ExtractBalls ( );
	//Now lets merge them!
	balls_num = MergeBalls ( balls_num );
	//parser
	balls_num_TrackingMode=Ball_Parser(world,balls_num);
	//The most important part, The Kalman Filter!


	
	
}
int Vision::ExtractBalls ( void )
{
	int ans = 0;
	for ( int i = 0 ; i < CAM_COUNT ; i ++ )
	{
		
		if (  use_camera[i] )
		{
			for ( int j = 0 ;(j < frame[i].balls_size ( ) && j<MAX_BALLS*CAM_COUNT); j ++ )
			{
				ball[ans] = frame[i].balls ( j );
				time_capture_ball[ans] = frame[i].t_capture();
				ans ++;
			}
		}
	}
	return ans;
}
int Vision::MergeBalls ( int num )
{
	int balls_num = 0;
	for ( int i = 0 ; i < num ; i ++ )
	{
		for ( int j = i + 1 ; j < num ; j ++ )
		{
			if ( sqrtf(powf(ball[i].x() - ball[j].x(), 2) + powf(ball[i].y() - ball[j].y(), 2)) < MERGE_DISTANCE )
			{
				ball[i].set_x ( ( ball[i].x ( ) + ball[j].x ( ) ) / (float)2.0 );
				ball[i].set_y ( ( ball[i].y ( ) + ball[j].y ( ) ) / (float)2.0 );
				if ( ball[i].has_z ( ) )
					ball[i].set_z ( ( ball[i].z ( ) + ball[j].z ( ) ) / (float)2.0 );

				ball[j] = ball[num-1];
				num --;

				j --;
			}
		}
		balls_num ++;
	}

	return balls_num;
}
int Vision::Ball_Parser(World &world, int num)
{
	if (TrackingMode)
	{
		for (int i = 0; i < num; i++)
		{
			VecPosition current_ball_pos = VecPosition(ball[i].x(), ball[i].y());
			
			if (ssl_lastball_n == 0)
				ssl_lastball_detection[ssl_lastball_n++] = current_ball_pos;
			for (int j = 0; j < ssl_lastball_n; j++)
			{
				if (current_ball_pos.getDistanceTo(ssl_lastball_detection[j]) < MERGE_DISTANCE)
					ssl_lastball_detection[j] = current_ball_pos;
				else
				{
					if (ssl_lastball_n < MAX_BALLS*CAM_COUNT)
						ssl_lastball_detection[ssl_lastball_n++] = current_ball_pos;
				}
			}
		}

	}
	else
	{
		ssl_lastball_n = 0;
		if (num > 0)
		{
			ssl_lastball_detection[0] = VecPosition(ball[0].x(), ball[0].y());
		
		}
	}
		//for (int i = 0;i < ssl_lastball_n;i++)
		//{
			if (world.team_side == TS_RightSide)
			{
				VecPosition temp_pos, temp_vel;
				BKF[0].Update(time_capture_ball[0], ssl_lastball_detection[0], temp_pos, temp_vel);
				world.ball.setCurrentBallPosition(temp_pos, time_capture_ball[0]);
				world.ball.setVelocity(temp_vel * 1000);
			}

			else if (world.team_side == TS_LeftSide)
			{
				VecPosition temp_pos, temp_vel;
				BKF[0].Update(time_capture_ball[0], -ssl_lastball_detection[0], temp_pos, temp_vel);
				world.ball.setCurrentBallPosition(temp_pos, time_capture_ball[0]);
				world.ball.setVelocity(temp_vel * 1000);
			}
		//}
	return (ssl_lastball_n);
}
void Vision::Nearer_Ball(VecPosition pos)
{

	int _index = 0;
	double Nearer_distanse = pos.getDistanceTo(ssl_lastball_detection[0]);
	for (int j = 1; j < ssl_lastball_n; j++)
	{
		if (Nearer_distanse > pos.getDistanceTo(ssl_lastball_detection[j]))
		{
			_index = j;
			Nearer_distanse = pos.getDistanceTo(ssl_lastball_detection[j]);
		}
	}

	if (_index != -1)
	{
		Current_ball_index = _index;
	}
}