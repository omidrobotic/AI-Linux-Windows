#include "Vision.h"

using namespace std;


Vision::~Vision()
{
}
void Vision::ProcessVision ( World &world)
{

	ProcessBalls (world);
	ProcessRobots (world);	
}
