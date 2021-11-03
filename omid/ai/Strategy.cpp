///to be changed

#include "Strategy.h"
#include "HighLevel.h"
#include <iostream>
using namespace std;
void example::choose_defends(int percent)
{

 SortByindex distance_robot[5];

for(int i=0 ;i<world.numT;i++)
{
    if(i!=3)
    {
    distance_robot[i].distance=world.robotT[world.getIndexForRobotTNumber(i)].position.getDistanceTo(world.robotT[3].position);
    distance_robot[i].index_robot=i;

    }

}

//sort
for(int j=1;j<world.numT;j++)
{
    double key =distance_robot[j].distance;
    int i=j-1;
    while((i>-1)&&(distance_robot[i].distance))
    {
        distance_robot[i+1]=distance_robot[i];
        i--;
    }
    distance_robot[i+1].distance=key;
}

}










//#include "Strategy.h"
//#include "GameState.h"
//#include "world.h"
//#include "HighLevel.h"
//
//
//void Strategy::DecideStrategy()
//{
//	switch (World::playMode)
//	{
//		case mode_State::PlayMode::Halt :
//			for (int i = 0; i < world.numT; i++)
//			{
//				
//			}
//		
//			break;
//		case mode_State::PlayMode::Stop :
//			if (world.time_period == TP_StartOfGame)
//			{
//				
//			}
//
//		default:
//			break;
//	}
//
//
//}
