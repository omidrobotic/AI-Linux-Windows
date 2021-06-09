#include "world.h"
#include "graphical/glframe.h"
#include "geometry.h"
#include "Switches.h"
#include <string.h>


/*! ROBOT METHODS */
void Robot::operator = (const Robot &r)
{
	id = r.id;
	position = r.position;
	angle = r.angle;
	wheelAngleForward = r.wheelAngleForward;
	wheelAngleBack = r.wheelAngleBack;
	radius = r.radius;
	timeCaptured = r.timeCaptured;
	timeSent = r.timeSent;
	timeRecived = r.timeRecived;
		
	destination_position = r.destination_position;
	destination_angle = r.destination_angle;

	memcpy(pathToDestination, r.pathToDestination, r.sizeOfPathToDestination);
	sizeOfPathToDestination = r.sizeOfPathToDestination;

	velocityToGo = r.velocityToGo;
	wToGo = r.wToGo;

	send_command = r.send_command;

	destination_set = r.destination_set;
}
/*! ROBOT METHODS */


/*! BALL METHODS */
bool Ball::setCurrentBallPosition(VecPosition _pos, double time_captured)
{
	//    if(_pos.getX()!=-1 && _pos.getY()!=-1 ) // && _pos.getX()!= oldPositions[0].getX() && _pos.getY()!= oldPositions[0].getY())
	//    {
	for (int i = MAX_BALLS - 2; i >= 0; i--)
	{
		oldPositions[i + 1] = oldPositions[i];
		oldTime[i + 1] = oldTime[i];
	}
	oldPositions[0] = _pos;
	oldTime[0] = time_captured;
	return true;
	//    }
	//    return false;
}

VecPosition Ball::getCurrentBallPosition()
{
	return oldPositions[0];
}

VecPosition Ball::getOldBallPosition(int _oldFrame)
{
	return oldPositions[_oldFrame];
}

double Ball::getOldBallTime(int _oldFrame)
{
	return oldTime[_oldFrame];
}

VecPosition Ball::getVelocity()
{
	return velocity;
}

bool Ball::setVelocity(VecPosition _vel)
{
	velocity = _vel;
	return true;
}

/*! BALL METHODS */


/*! FIELD METHODS */
VecPosition Field::getGoalMidO()
{
	return VecPosition(-(FieldLength / 2), 0);
}

VecPosition Field::getGoalMidP()
{
	return VecPosition((FieldLength / 2), 0);
}

VecPosition Field::getUpLeftPoint()
{
	return VecPosition(-(FieldLength / 2), FieldWidth / 2);
}

VecPosition Field::getUpRightPoint()
{
	return VecPosition((FieldLength / 2), FieldWidth / 2);
}

VecPosition Field::getDownLeftPoint()
{
	return VecPosition(-(FieldLength / 2), -FieldWidth / 2);
}

VecPosition Field::getDownRightPoint()
{
	return VecPosition((FieldLength / 2), -FieldWidth / 2);
}

VecPosition Field::getSurroundFieldUpLeftPoint()
{
	return VecPosition(-(SurroundFieldLength / 2), SurroundFieldWidth / 2);
}

VecPosition Field::getSurroundFieldUpRightPoint()
{
	return VecPosition((SurroundFieldLength / 2), SurroundFieldWidth / 2);
}

VecPosition Field::getSurroundFieldDownLeftPoint()
{
	return VecPosition(-(SurroundFieldLength / 2), -SurroundFieldWidth / 2);
}

VecPosition Field::getSurroundFieldDownRightPoint()
{
	return VecPosition((SurroundFieldLength / 2), -SurroundFieldWidth / 2);
}

VecPosition Field::getUpMidPoint()
{
	return VecPosition(0, FieldWidth / 2);
}

VecPosition Field::getDownMidPoint()
{
	return VecPosition(0, -FieldWidth / 2);
}

VecPosition Field::getMidPoint()
{
	return VecPosition(0, 0);
}

VecPosition Field::getUpBarP()
{
	return VecPosition((FieldLength / 2), GoalLength / 2);
}

VecPosition Field::getDownBarP()
{
	return VecPosition((FieldLength / 2), -GoalLength / 2);
}

VecPosition Field::getUpBarO()
{
	return VecPosition(-(FieldLength / 2), GoalLength / 2);
}

VecPosition Field::getDownBarO()
{
	return VecPosition(-(FieldLength / 2), -GoalLength / 2);
}

VecPosition Field::getUpLeftCornel()
{
	return VecPosition(-CornelPoint, FieldWidth / 2);
}

VecPosition Field::getDownLeftCornel()
{
	return VecPosition(-CornelPoint, -FieldWidth / 2);
}

VecPosition Field::getUpRightCornel()
{
	return VecPosition(CornelPoint, FieldWidth / 2);
}

VecPosition Field::getDownRightCornel()
{
	return VecPosition(CornelPoint, -FieldWidth / 2);
}

VecPosition Field::getUpLeft_LeftPenaltyArea()
{
	return VecPosition(-FieldLength / 2, PenaltyAreaLength / 2);
}

VecPosition Field::getDownLeft_LeftPenaltyArea()
{
	return VecPosition(-FieldLength / 2, -PenaltyAreaLength / 2);
}

VecPosition Field::getUpRight_LeftPenaltyArea()
{
	return VecPosition((-FieldLength / 2) + PenaltyAreaWidth, PenaltyAreaLength / 2);
}

VecPosition Field::getDownRight_LeftPenaltyArea()
{
	return VecPosition((-FieldLength / 2) + PenaltyAreaWidth, -PenaltyAreaLength / 2);
}

VecPosition Field::getUpLeft_RightPenaltyArea()
{
	return VecPosition((FieldLength / 2) - PenaltyAreaWidth, PenaltyAreaLength / 2);
}

VecPosition Field::getDownLeft_RightPenaltyArea()
{
	return VecPosition((FieldLength / 2) - PenaltyAreaWidth, -PenaltyAreaLength / 2);
}

VecPosition Field::getUpRight_RightPenaltyArea()
{
	return VecPosition(FieldLength / 2, PenaltyAreaLength / 2);
}

VecPosition Field::getDownRight_RightPenaltyArea()
{
	return VecPosition(FieldLength / 2, -PenaltyAreaLength / 2);
}

VecPosition Field::getPenaltyPointP()
{
	return VecPosition((FieldLength / 2) - PenaltyPoint, 0);
}

VecPosition Field::getPenaltyPointO()
{
	return VecPosition(-(FieldLength / 2) + PenaltyPoint, 0);
}

VecPosition Field::getRightGoal_UpGoalPost_RightPoint()
{
	return VecPosition(FieldLength/2 + GoalPostLength,GoalLength/2);
}

VecPosition Field::getRightGoal_DownGoalPost_RightPoint()
{
	return VecPosition(FieldLength / 2 + GoalPostLength, -GoalLength / 2);
}

VecPosition Field::getRightGoal_UpGoalPost_LeftPoint()
{
	return VecPosition(FieldLength / 2, GoalLength / 2);
}

VecPosition Field::getRightGoal_DownGoalPost_LeftPoint()
{
	return VecPosition(FieldLength / 2, -GoalLength / 2);
}

VecPosition Field::getLeftGoal_UpGoalPost_RightPoint()
{
	return VecPosition(-FieldLength / 2, GoalLength / 2);
}

VecPosition Field::getLeftGoal_DownGoalPost_RightPoint()
{
	return VecPosition(-FieldLength / 2, -GoalLength / 2);
}

VecPosition Field::getLeftGoal_UpGoalPost_LeftPoint()
{
	return VecPosition(-FieldLength / 2 - GoalPostLength, GoalLength / 2);
}

VecPosition Field::getLeftGoal_DownGoalPost_LeftPoint()
{
	return VecPosition(-FieldLength / 2 - GoalPostLength, -GoalLength / 2);
}

Paraline Field::getUpParaline_LeftPenaltyArea()
{
    Paraline temp = Paraline(getUpLeft_LeftPenaltyArea(), getUpRight_LeftPenaltyArea());
	return temp;
}

Paraline Field::getRightParaline_LeftPenaltyArea()
{
	return Paraline(getUpRight_LeftPenaltyArea(), getDownRight_LeftPenaltyArea());
}

Paraline Field::getDownParaline_LeftPenaltyArea()
{
	return Paraline(getDownLeft_LeftPenaltyArea(), getDownRight_LeftPenaltyArea());
}

Paraline Field::getLeftParaline_LeftPenaltyArea()
{
	return Paraline(getUpLeft_LeftPenaltyArea(), getDownLeft_LeftPenaltyArea());
}

Paraline Field::getUpParaline_RightPenaltyArea()
{
	return Paraline(getUpLeft_RightPenaltyArea(), getUpRight_RightPenaltyArea());
}

Paraline Field::getRightParaline_RightPenaltyArea()
{
	return Paraline(getUpRight_RightPenaltyArea(), getDownRight_RightPenaltyArea());
}

Paraline Field::getDownParaline_RightPenaltyArea()
{
	return Paraline(getDownLeft_RightPenaltyArea(), getDownRight_RightPenaltyArea());
}

Paraline Field::getLeftParaline_RightPenaltyArea()
{
	return Paraline(getUpLeft_RightPenaltyArea(), getDownLeft_RightPenaltyArea());
}

Paraline Field::getRightGoal_UpGoalPost_CountinuedToEndOfField()
{
	return Paraline(getRightGoal_UpGoalPost_LeftPoint(), VecPosition(SurroundFieldLength / 2, GoalLength / 2));
}

Paraline Field::getRightGoal_DownGoalPost_CountinuedToEndOfField()
{
	return Paraline(getRightGoal_DownGoalPost_LeftPoint(), VecPosition(SurroundFieldLength / 2, -GoalLength / 2));
}

Paraline Field::getLeftGoal_UpGoalPost_CountinuedToEndOfField()
{
	return Paraline(getLeftGoal_UpGoalPost_RightPoint(), VecPosition(-SurroundFieldLength / 2, GoalLength / 2));
}

Paraline Field::getLeftGoal_DownGoalPost_CountinuedToEndOfField()
{
	return Paraline(getLeftGoal_DownGoalPost_RightPoint(), VecPosition(-SurroundFieldLength / 2, -GoalLength / 2));
}

Line Field::getGoalLineP()
{
	Line goalLineP = Line::makeLineFromTwoPoints(Field::getUpBarP(), Field::getDownBarP());
	return goalLineP;
}

Line Field::getGoalLineO()
{
	Line goalLineO = Line::makeLineFromTwoPoints(Field::getUpBarO(), Field::getDownBarO());
	return goalLineO;
}

Rect Field::getFieldRectangle()//build the ground
{
	Rect FieldRectangle;
	FieldRectangle.setRectanglePoints(getUpLeftPoint(), getDownRightPoint());;
	return FieldRectangle;
}

Rect Field::getSurroundFieldRectangle()
{
	Rect SurrondFieldRectangle;
	SurrondFieldRectangle.setRectanglePoints(getSurroundFieldUpLeftPoint(), getSurroundFieldDownRightPoint());;
	return SurrondFieldRectangle;
}

Rect Field::getMidAreaRect()
{
	Rect FieldRectangle;
	FieldRectangle.setRectanglePoints(VecPosition(-(FieldLength / 4), (FieldWidth / 2)), VecPosition((FieldLength / 4), -(FieldWidth / 2)));;
	return FieldRectangle;
}

Rect Field::getTeamAreaRect()
{
	Rect FieldRectangle;
	FieldRectangle.setRectanglePoints(VecPosition(0, (FieldWidth / 2)), getDownRightPoint());
	return FieldRectangle;
}

Rect Field::getOppAreaonetouchRectup()
{
	Rect FieldRectangle;
	FieldRectangle.setRectanglePoints(VecPosition(0, (FieldWidth / 2)), VecPosition(-(FieldLength / 4), 0));
	return FieldRectangle;
}

Rect Field::getOppAreaonetouchRectdown()
{
	Rect FieldRectangle;
	FieldRectangle.setRectanglePoints(VecPosition(0, -(FieldWidth / 2)), VecPosition(-(FieldLength / 4), 0));
	return FieldRectangle;
}

Rect Field::getOppAreaRect()
{
	Rect FieldRectangle;
	FieldRectangle.setRectanglePoints(getUpLeftPoint(), VecPosition(0, -(FieldWidth / 2)));
	return FieldRectangle;
}

#if RECTANGULAR_PENALTY_AREA != 0
Rect Field::getLeftPenaltyArea()
{
	Rect LeftPenaltyAreaRectangle(getUpLeft_LeftPenaltyArea(), getDownRight_LeftPenaltyArea());
	return LeftPenaltyAreaRectangle;
}

Rect Field::getRightPenaltyArea()
{
	Rect RightPenaltyAreaRectangle(getUpLeft_RightPenaltyArea(), getDownRight_RightPenaltyArea());
	return RightPenaltyAreaRectangle;
}
#endif
bool Field::isInField(const VecPosition &v)
{
	return getFieldRectangle().isInside(v);
}

bool Field::isInSurroundField(const VecPosition &v)
{
	return (getSurroundFieldRectangle().isInside(v));
}

bool Field::isInSurroundField_notInBehindGoal(const VecPosition &v)
{
	return (isInSurroundField(v) && !isInBehindGoal(v));
}

bool Field::isInBehindGoal(const VecPosition &v)
{
	return (Rect(getLeftGoal_UpGoalPost_LeftPoint(), VecPosition(-SurroundFieldLength / 2, -GoalLength / 2)).isInside(v) ||
			Rect(VecPosition(SurroundFieldLength / 2, GoalLength / 2), getRightGoal_DownGoalPost_RightPoint()).isInside(v));
}

short int Field::detect_airt(VecPosition position)
{
	if (position.getX() >= 0 && position.getY() >= 0)
		return 1;
	else if (position.getX() <= 0 && position.getY() >= 0)
		return 2;
	else if (position.getX() <= 0 && position.getY() <= 0)
		return 3;
	else
		return 4;
}

/*! FIELD METHODS */


/**! WORLD'S STATIC VARIABLES **/

World						World::instance;
TeamColor					World::team_color;
TeamSide					World::team_side;
Timer						World::glTimer;
mode_State::PlayMode        World::playMode;
mode_State::KickMode        World::kickMode;
mode_State::StageMode		World::stageMode;
VecPosition                 World::ballPlacementPosition;


	/* ! RRT VARIABLES*/



#if RECTANGULAR_PENALTY_AREA == 0
const Circle World::penalty_area_circle_balk[4] = { Circle(VecPosition(-4500, 250),1000 + ROBOT_RADIUS), Circle(VecPosition(-4500, -250),1000 + ROBOT_RADIUS), Circle(VecPosition(4500, 250),1000+ROBOT_RADIUS), Circle(VecPosition(4500, -250),1000+ROBOT_RADIUS) };
Paraline World::penalty_area_line_balk[2] = { Paraline(VecPosition(PALU.getX()+ROBOT_RADIUS,PALU.getY()),VecPosition(PALD.getX() + ROBOT_RADIUS,PALD.getY())),Paraline(VecPosition(PARU.getX() - ROBOT_RADIUS,PARU.getY()),VecPosition(PARD.getX() - ROBOT_RADIUS,PARD.getY()) )};
#endif
//double Bar::line_slope(Cartesian_Coordinates start, Cartesian_Coordinates end)
//{
//	if ((end.x - start.x) != 0)
//		return (end.y - start.y) / (end.x - start.x);
//	else
//		return std::numeric_limits<float>::infinity();
//}

//double Bar::perp_slope(double slope)
//{
//	if (slope != 0)
//		return -1 / slope;
//	else
//		return std::numeric_limits<float>::infinity();
//}

///find point with having slope and distance 
//Cartesian_Coordinates Bar::find_Cartesian_Coordinates(Cartesian_Coordinates start, double slope, double distance)		//distance > 0 ->  x end be rast miravad	//distance < 0 ->  x end be chap miravad	//slope > 0 ->  y end be bala miravad	//slope < 0 ->  y end be paein miravad
//{
//	Cartesian_Coordinates end;
//
//	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
//	end.x = start.x + sign(distance)*x;
//	end.y = start.y + sign(distance)*slope*x;
//
//	/*if (airt == 1)
//	{
//	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
//	end.x = start.x + sign(distance)*x;
//	end.y = start.y + sign(distance)*slope*x;
//	}
//	else if (airt == 2)
//	{
//	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
//	end.x = start.x + sign(distance)*x;
//	end.y = start.y + sign(distance)*slope*x;
//	}
//	else if (airt == 3)
//	{
//	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
//	end.x = start.x - sign(distance)*x;
//	end.y = start.y - sign(distance)*slope*x;
//	}
//	else
//	{
//	double x = sqrt(pow(distance, 2) / (1 + pow(slope, 2)));
//	end.x = start.x - sign(distance)*x;
//	end.y = start.y - sign(distance)*slope*x;
//	}*/
//
//	return end;
//}

//int World::num_of_active_line_balks = 0;
//int World::num_of_active_circle_balks = 0;
//Circle World::circle_balk[World::num_of_allowed_circle_balks];
//Circle World::robotT_circle_balk[num_of_allowed_robotT_circle_balks];
//Circle World::robotO_circle_balk[num_of_allowed_robotO_circle_balks];
//Bar_Segment World::line_balk[World::num_of_allowed_line_balks];
//int World::index_of_active_circle_balks[World::num_of_allowed_circle_balks];
//int World::index_of_active_line_balks[World::num_of_allowed_circle_balks];
	/* ! RRT VARIABLES*/


#pragma region "Chart Variables"
#if DRAW_CHART == 1
	/* ! Chart VARIABLES*/
double World::AI_speeds[number_of_speeds][2];
double World::ROBOT_speeds[number_of_speeds][2];
//double World::ROBOT_avarage_location_x = 0;
//double World::ROBOT_avarage_location_y = 0;
VecPosition World::previous_robot_speed;
VecPosition World::previous_robot_location;
double World::previous_vision_time;
VecPosition World::robot_speed;
//bool World::draw_chart = true;
	/* ! Chart VARIABLES*/
#endif
#pragma endregion



#if USE_FEEDFORWARD == 1
VecPosition World::summation_of_movement_seen_by_vision_0to15[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
VecPosition World::summation_of_movement_seen_by_vision_mines15to0[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
VecPosition World::summation_of_movement_calculated_by_commands_15to30[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
VecPosition World::summation_of_movement_calculated_by_commands_0to15[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
#endif


/**! WORLD'S STATIC VARIABLES **/


/*! WORLD METHODS */
World::World()
{
}

World &World::getInstance()
{
	return instance;
}

void World::setStageMode(mode_State::StageMode _sm)
{
	stageMode = _sm;
}

void World::setPlayMode(mode_State::PlayMode _pm)
{
	playMode = _pm;
}

void World::setKickMode(mode_State::KickMode _pm)
{
	kickMode = _pm;
}

void World::setTeamColor(TeamColor _cl)
{
	team_color = _cl;
}

void World::setTeamSide(TeamSide _s)
{
	team_side = _s;
}

int World::getIndexForRobotTNumber(const int &robot_number) const
{
	for (int i = 0; i < numT; i++)
	{
		if (robotT[i].id == robot_number)
		{
			return i;
		}
	}
	cout << "error on getting Index for robot number "<< robot_number << endl;
	return -1;
}

int World::getRobotTNumberForIndex(const int &robot_index) const
{
	return world.robotT[robot_index].id;
}

int World::getIndexForRobotONumber(const int &robot_number) const
{
	for (int i = 0; i < numO; i++)
	{
		if (robotO[i].id == robot_number)
		{
			return i;
		}
	}
	cout << "error on getting Index for robot number " << robot_number << endl;
	return -1;
}

int World::getRobotONumberForIndex(const int &robot_index) const
{
	return world.robotO[robot_index].id;
}

VecPosition World::get_robotT_position(const int &robot_number) const
{
	for (int i = 0; i < numT; i++)
	{
		if (robotT[i].id == robot_number)
		{
			return robotT[i].position;
		}
	}
	return false;
}

#if USE_FEEDFORWARD == 1
void World::shift_robot_positions_seen_by_vision()
{
	for (int i = 0; i < world.numT; i++)
	{
		for (int j = World::feedforward_number_of_speeds_to_consider - 2; j >= 0; j--)
		{
			world.robot_movement_seen_by_vision[i][j+1] = world.robot_movement_seen_by_vision[i][j];
		}
	}
}

void World::shift_robot_positions_calculated_by_commands()
{
	for (int i = 0; i < world.numT; i++)
	{
		for (int j = World::feedforward_number_of_speeds_to_consider - 2; j >=0; j--)
		{
			world.robot_movement_calculated_by_commands[i][j + 1] = world.robot_movement_calculated_by_commands[i][j];
		}
	}
}

#endif

void World::exactSleep(double desired_delay)	//in milliseconds
{
	auto start_time = std::chrono::high_resolution_clock::now();
	auto end_time = std::chrono::high_resolution_clock::now();
	double time = std::chrono::duration_cast<std::chrono::microseconds>(start_time - end_time).count();
	while (time < desired_delay * 1000)
	{
		end_time = std::chrono::high_resolution_clock::now();
		time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
	}
	return;
}

/*! WORLD METHODS */


/*! DrawObjects METHODS */

//void DrawShape::DrawDot(VecPosition point, double radius, double red, double green, double blue)
//{
//	Circle c(point, radius);
//	GLFrame::addCirlceToPainting(c, red, green, blue,true);
//	//_sleep(300);
//}
//
//void DrawShape::DrawCircle(Circle circle, double red, double green, double blue, bool is_fill)
//{
//	GLFrame::addCirlceToPainting(circle, red, green, blue,is_fill);
//}
//
//void DrawShape::DrawParaline(VecPosition start, VecPosition end, double red, double green, double blue)
//{
//	GLFrame::addLineToPainting(start, end, red, green, blue);
//}
//
//void DrawShape::DrawParaline(Paraline pl, double red, double green, double blue)
//{
//	GLFrame::addLineToPainting(pl.getFirstPoint(),pl.getSecondPoint(), red, green, blue);
//}
//
//void DrawShape::ClearCircles()
//{
//	GLFrame::resetCirclePaintings();
//}
//
//void DrawShape::ClearLines()
//{
//	GLFrame::resetLinePaintings();
//}

/*! DrawObjects METHODS */


/*! Tools METHODS */
int Tools::u(double t)
{
	if (t >= 0)
		return 1;
	else
		return 0;
}
/*! Tools METHODS */
