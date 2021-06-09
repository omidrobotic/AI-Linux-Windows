#include "RRT.h"
#include "FieldGeometry.h"
//#include <conio.h>
#include "Switches.h"
/*! CLASS GRAPH */
RRT rrt;

Graph::Graph()
{
	p = 0;
	q = 0;
}
VecPosition Graph::nearest_vertex(const VecPosition &crd)
{
	double max_distance = numeric_limits<double>::infinity();
	double shortest_distance = max_distance;
	double distance;

	VecPosition nearest_vertex;
	for (int i = 0; i <p; i++)
	{
		if (i == 1)
			continue;
		distance = crd.getDistanceTo(vertex[i]);
		if (distance < shortest_distance && crd.getX() != vertex[i].getX() && crd.getY() != vertex[i].getY())
		{
			shortest_distance = distance;
			nearest_vertex = vertex[i];
		}
	}

	return nearest_vertex;
}
void Graph::add_vertex(const VecPosition &crd,const int &parrent)
{
	p++;
	vertex[p - 1] = crd;
	vertex[p - 1].parent = parrent;
	vertex[p - 1].number = p - 1;
}
void Graph::add_edge(const VecPosition &start_crd, const VecPosition &end_crd)
{
	q++;
	edge[q - 1].start_vertex = start_crd;
	edge[q - 1].end_vertex = end_crd;
}
void Graph::remove_vertex(int number)
{

}
//VecPosition Graph::calculate_vertex(const VecPosition &start, const VecPosition &destination, const double &distance)
//{
//	if (start.getDistanceTo(destination) > distance)
//	{
//		/*return start.getPositionFromSlopeAndDistance(Bar::line_slope(start, destination), sign(destination.getX() - start.getX()) * distance);*/
//		double m = Bar::line_slope(start, destination);
//		double alfa = tanh(m);
//		if (destination.getX() < start.getX())
//			alfa += M_PI;
//		return VecPosition((distance*cos(alfa) + start.getX()), (distance*sin(alfa) + start.getY()));
//	}
//	else
//	{
//		return destination;
//	}
//}
void Graph::find_pass(const VecPosition &start, const VecPosition &end, VecPosition* pass, int &size_of_pass)
{
	/*int index_of_pass_vertex = 0;
	//Cartesian_Coordinates checked_vertices[World::num_of_vertices];
	int index_of_checking_edge = 0;
	int previous_index_of_checking_edge = 0;
	Cartesian_Coordinates Intended_vertex = end;		//vertex that looking for nearest vertex
	Edge b;
	b.start_vertex.x = NAN;
	b.start_vertex.y = NAN;
	b.end_vertex.x = NAN;
	b.end_vertex.y = NAN;

	bool found_pass = false;
	bool found_vertex = false;

	pass[0] = end;
	index_of_pass_vertex++;

	int test = q;
	while (true)
	{
	for (int i=q-1;i>=0;i--)
	{
	if (i == index_of_checking_edge)
	continue;
	if (edge[i].end_vertex.x == Intended_vertex.x && edge[i].end_vertex.y == Intended_vertex.y)
	{
	if (edge[i].start_vertex.x == start.x && edge[i].start_vertex.y == start.y)
	{
	pass[index_of_pass_vertex] = edge[i].start_vertex;
	found_pass = true;
	found_vertex = true;
	break;
	}
	pass[index_of_pass_vertex] = edge[i].start_vertex;
	Intended_vertex = edge[i].start_vertex;
	previous_index_of_checking_edge = index_of_checking_edge;
	index_of_checking_edge = i;
	index_of_pass_vertex++;
	i = q;
	}

	else if (edge[i].start_vertex.x == Intended_vertex.x && edge[i].start_vertex.y == Intended_vertex.y)
	{
	if (edge[i].end_vertex.x == start.x && edge[i].end_vertex.y == start.y)
	{
	pass[index_of_pass_vertex] = edge[i].end_vertex;
	found_pass = true;
	found_vertex = true;
	break;
	}
	pass[index_of_pass_vertex] = edge[i].end_vertex;
	Intended_vertex = edge[i].end_vertex;
	previous_index_of_checking_edge = index_of_checking_edge;
	index_of_checking_edge = i;
	index_of_pass_vertex++;
	i = q;
	}
	}

	if (!found_vertex)
	{
	//edge[index_of_checking_edge].end_vertex.x == Intended_vertex.x
	//&& edge[index_of_checking_edge].end_vertex.y == Intended_vertex.y
	if (edge[previous_index_of_checking_edge].end_vertex.x == edge[index_of_checking_edge].start_vertex.x
	&& edge[previous_index_of_checking_edge].end_vertex.y == edge[index_of_checking_edge].start_vertex.y
	||
	edge[previous_index_of_checking_edge].end_vertex.x == edge[index_of_checking_edge].end_vertex.x
	&& edge[previous_index_of_checking_edge].end_vertex.y == edge[index_of_checking_edge].end_vertex.y)
	{
	Intended_vertex = edge[previous_index_of_checking_edge].end_vertex;
	}

	else if (edge[previous_index_of_checking_edge].start_vertex.x == edge[index_of_checking_edge].end_vertex.x
	&& edge[previous_index_of_checking_edge].start_vertex.y == edge[index_of_checking_edge].end_vertex.y
	||
	edge[previous_index_of_checking_edge].start_vertex.x == edge[index_of_checking_edge].start_vertex.x
	&& edge[previous_index_of_checking_edge].start_vertex.y == edge[index_of_checking_edge].start_vertex.y)
	{
	Intended_vertex = edge[previous_index_of_checking_edge].start_vertex;
	}

	edge[index_of_checking_edge] = b;
	pass[index_of_pass_vertex].x = NAN;
	pass[index_of_pass_vertex].y = NAN;
	index_of_pass_vertex--;
	cout << "";

	/*edge[index_of_checking_edge].start_vertex.x = NULL;
	edge[index_of_checking_edge].start_vertex.y = NULL;
	edge[index_of_checking_edge].end_vertex.x = NULL;
	edge[index_of_checking_edge].end_vertex.y = NULL;
	}

	if (found_pass)
	return pass;
	}*/

	pass[0] = end;
	for (int i = 1; i<World::num_of_vertices; i++)
	{
		pass[i] = vertex[pass[i - 1].parent];

		if (pass[i].getX() == start.getX()   &&   pass[i].getY() == start.getY())
		{
			size_of_pass = i/*+1*/;
			break;
		}
	}
}
void Graph::optimize_pass(const VecPosition* const pass, VecPosition* const optimized_pass, const int &size_of_pass, int &size_of_optimized_pass, const short int &robot_index, const bool &robotT_is_balk, const bool &robotO_is_balk, const bool &penalty_area_is_balk, const Ball::BalkMode &ballBalkMode)
{
	int i , j ;
	optimized_pass[0] = pass[0];		//destination
	size_of_optimized_pass = 1;
	for (i = 0, j = 0; i + 2 < size_of_pass+1; i++)
	{
		//Balk::check_collision(optimized_pass[j], pass[i + 2]) || (penalty_area_is_balk && Balk::check_collision_with_penalty_area(optimized_pass[j], pass[i + 2]))
		if (Balk::check_collision_with_(optimized_pass[j], pass[i + 2], robot_index, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, ballBalkMode))
		{
			//Balk::check_collision(optimized_pass[j], pass[i + 2]);
			j++;
			optimized_pass[j] = pass[i + 1];
			size_of_optimized_pass++;
		}
	}

	optimized_pass[size_of_optimized_pass] = pass[size_of_pass/*+1*/];
	//size_of_optimized_pass++;
}
bool Graph::is_in_field(const VecPosition &point)
{
	if (point.getX() > (FieldLength / 2) || point.getX() < -(FieldLength / 2)
		|| point.getY() >(FieldWidth / 2) || point.getY() < -(FieldWidth / 2))
		return false;
}
//VecPosition Graph::find_nearest_position_outside_balk(VecPosition &destination_position_C,const Circle &balk)	//!!!!!!!! wrong if two robots are near !!!!!!!
//{
//
//	/*VecPosition destination_position;
//	VecPosition current_position;
//
//	destination_position.setX(destination_position_C.x);
//	destination_position.setY(destination_position_C.y);
//
//	current_position.setX(current_position_C.x);
//	current_position.setY(current_position_C.y);
//
//	Line temp;
//	temp = Line::makeLineFromTwoPoints(current_position, destination_position);
//
//	VecPosition pos_solution1;
//	VecPosition pos_solution2;
//	temp.getCircleIntersectionPoints(balk, &pos_solution1, &pos_solution2);
//	destination_position_C.x = pos_solution1.getX();
//	destination_position_C.y = pos_solution1.getY();
//	return destination_position_C;
//	*/
//
//	Circle second_circle;
//
//	if (balk.getRadius() - destination_position_C.getDistanceTo(balk.getCenter()) <= 0)
//		return destination_position_C;
//
//	VecPosition second_circle_center;
//
//	second_circle.setRadius(balk.getRadius() - destination_position_C.getDistanceTo(balk.getCenter()));
//	second_circle.setCenter(destination_position_C);
//
//	VecPosition pos_solution1;
//	VecPosition pos_solution2;
//	second_circle.getIntersectionPoints(balk, &pos_solution1, &pos_solution2);
//	destination_position_C = pos_solution1;
//
//	return destination_position_C;
//}
/*! CLASS GRAPH */


/*! CLASS BAR */
//double Bar::line_slope(VecPosition start, VecPosition end)
//{
//	if ((end.getX() - start.getX()) != 0)
//		return (end.getY() - start.getY()) / (end.getX() - start.getX());
//	else if(end.getY()>=start.getY())
//		return std::numeric_limits<double>::infinity();
//	else
//		return -std::numeric_limits<double>::infinity();
//}
double Bar::perp_slope(double slope)
{
	if (slope != 0)
		return -1 / slope;
	else
		return std::numeric_limits<float>::infinity();
}
//VecPosition Bar::find_VecPosition(VecPosition start, double slope, double distance)		//distance > 0 ->  x end be rast miravad	//distance < 0 ->  x end be chap miravad	//slope > 0 ->  y end be bala miravad	//slope < 0 ->  y end be paein miravad
//{
//	
//}
/*! CLASS BAR */


///this class is not needed now in the code,but maybe later
/*! CLASS CARTESIAN_COORDINATES */

//short int Cartesian_Coordinates::detect_airt(Cartesian_Coordinates crd)
//{
//	if (crd.x >= 0 && crd.y >= 0)
//		return 1;
//	else if (crd.x <= 0 && crd.y >= 0)
//		return 2;
//	else if (crd.x <= 0 && crd.y <= 0)
//		return 3;
//	else
//		return 4;
//}
//auto VecPosition::cartesian_to_polar()
//{
//	Polar_Coordinates PC;
//	short int airt = detect_airt();
//	if(airt == 1)
//		PC.angle = atan(getY() / getX());
//	else if(airt == 4)
//		PC.angle = atan(getY() / getX()) + 2*M_PI;
//	else
//		PC.angle = atan(getY() / getX()) + M_PI;
//
//	if (getX() == 0 && getY() > 0)
//		PC.angle = M_PI / 2;
//	if (getX() == 0 && getY() < 0)
//		PC.angle = 3*M_PI / 2;
//	if (getX() == 0 && getY() == 0)
//		PC.angle = 0;
//
//	PC.r = sqrt(pow(getX(), 2) + pow(getY(), 2));
//	return PC;
//}

//Polar_Coordinates VecPosition::cartesian_to_polar_from_CP(Cartesian_Coordinates c,Cartesian_Coordinates cp)
//{
//	c.x = c.x - cp.x;
//	c.y = c.y - cp.y;
//
//	Polar_Coordinates_From_CP PCFCP;
//	short int airt = Cartesian_Coordinates::detect_airt(c);
//	if (airt == 1)
//		PCFCP.angle = atan(c.y / c.x);
//	else if (airt == 4)
//		PCFCP.angle = atan(c.y / c.x) + 2 * M_PI;
//	else
//		PCFCP.angle = atan(c.y / c.x) + M_PI;
//
//	if (c.x == 0 && c.y > 0)
//		PCFCP.angle = M_PI / 2;
//	if (c.x == 0 && c.y < 0)
//		PCFCP.angle = 3 * M_PI / 2;
//	if (c.x == 0 && c.y == 0)
//		PCFCP.angle = 0;
//
//	PCFCP.r = sqrt(pow(c.x, 2) + pow(c.y, 2));
//	return PCFCP;
//}
////****

//Cartesian_Coordinates Cartesian_Coordinates::polar_to_cartesian(Polar_Coordinates p/*,Cartesian_Coordinates current_position*/)
//{
//	Cartesian_Coordinates CC;
//	CC.x = p.r*cos(p.angle) /*+ current_position.x*/;
//	CC.y = p.r*sin(p.angle) /*+ current_position.y*/;
//	return CC;
//}

/*! CLASS CARTESIAN_COORDINATES */


/*! CLASS BALK */
bool Balk::check_collision_with_robotT(VecPosition start, VecPosition end, const short int &index_of_robot)
{
	//start and end should not be in balk
	///using world.numT and world.robotT_circle_balk[i] with each other is NOT GOOD!
	/*double precision = 1000;
	start.setX(round(start.getX() * precision) / precision);
	start.setY(round(start.getY() * precision) / precision);

	end.setX(round(end.getX() * precision) / precision);
	end.setY(round(end.getY() * precision) / precision);*/

	///check collison with old algorithm : better
	VecPosition pos_solution1;
	VecPosition pos_solution2;
	Line line = Line::makeLineFromTwoPoints(start, end);
	///collision with circles
	for (int i = 0; i < world.numT; i++)
	{
		if (i == index_of_robot)
			continue;
		/*if (start.isInCircle(world.robotT_circle_balk[i]) || end.isInCircle(world.robotT_circle_balk[i]))
			continue;*/
		if (start.isInCircle(world.robotT_circle_balk[i]) || end.isInCircle(world.robotT_circle_balk[i]))
			return true;
		int num_of_intersection_points = line.getCircleIntersectionPoints(world.robotT_circle_balk[i], &pos_solution1, &pos_solution2);
		if (num_of_intersection_points != 0)
		{
			int precision = 100;
			pos_solution1.setX(round(pos_solution1.getX() * precision) / precision);
			pos_solution1.setY(round(pos_solution1.getY() * precision) / precision);
			pos_solution2.setX(round(pos_solution2.getX() * precision) / precision);
			pos_solution2.setY(round(pos_solution2.getY() * precision) / precision);

			start.setX(round(start.getX() * precision) / precision);
			start.setY(round(start.getY() * precision) / precision);
			end.setX(round(end.getX() * precision) / precision);
			end.setY(round(end.getY() * precision) / precision);

			//pos_solution1 = end;

			if (max(start.getDistanceTo(pos_solution1), end.getDistanceTo(pos_solution1)) < start.getDistanceTo(end)
				|| max(start.getDistanceTo(pos_solution2), end.getDistanceTo(pos_solution2)) < start.getDistanceTo(end))
			{
				return true;
			}

		}
	}


	//	/*if (num_of_intersection_points == 2)
	//		{
	//			if (start.getDistanceTo(min(start.getDistanceTo(pos_solution1), start.getDistanceTo(pos_solution2))) <= start.getDistanceTo(end))
	//			{
	//				return true;
	//			}
	//			cout << "";
	//		}
	//		else if (num_of_intersection_points == 1)
	//		{
	//			if (start.getDistanceTo(start.getDistanceTo(pos_solution1)) <= start.getDistanceTo(end))
	//			{
	//				start.getDistanceTo(2);
	//				return true;
	//			}
	//		}*/


	///collision with lines
	////****need changes for robotT_balk
	//for (int i = 0; i<World::num_of_active_line_balks; i++)
	//		{
	//		Line balk = Line::makeLineFromTwoPoints(World::line_balk[i].start_point, World::line_balk[i].end_point);
	//		VecPosition intersection_point = line.getIntersection(balk);
	//		if (max(intersection_point.getDistanceTo(World::line_balk[i].start_point), intersection_point.getDistanceTo(World::line_balk[i].end_point)) <= World::line_balk[i].start_point.getDistanceTo(World::line_balk[i].end_point)
	//		&& max(intersection_point.getDistanceTo(start) , intersection_point.getDistanceTo(end)) <= start.getDistanceTo(end))
	//		{
	//		return true;
	//		}
	//		}
	//}



	///check collison with paraline : worse
	/*Paraline paraline(start,end);
	for (int i = 0; i < World::num_of_active_robotT_circle_balks; i++)
	{
	if (paraline.hasIntersection(World::circle_balk[i],0))
	return true;
	}*/


	return false;
}
bool Balk::check_collision_with_robotO(VecPosition start, VecPosition end)
{
	/*double precision = 1000;
	start.setX(round(start.getX() * precision) / precision);
	start.setY(round(start.getY() * precision) / precision);

	end.setX(round(end.getX() * precision) / precision);
	end.setY(round(end.getY() * precision) / precision);*/

	///check collison with old alhorithm : better
	VecPosition pos_solution1;
	VecPosition pos_solution2;
	Line line = Line::makeLineFromTwoPoints(start, end);
	//collision with circles
	for (int i = 0; i < world.numO; i++)
	{
		/*if (start.isInCircle(world.robotT_circle_balk[i]) || end.isInCircle(world.robotT_circle_balk[i]))
		continue;*/
		if (start.isInCircle(world.robotO_circle_balk[i]) || end.isInCircle(world.robotO_circle_balk[i]))
			return true;
		int num_of_intersection_points = line.getCircleIntersectionPoints(world.robotO_circle_balk[i], &pos_solution1, &pos_solution2);
		if (num_of_intersection_points != 0)
		{
			int precision = 100;
			pos_solution1.setX(round(pos_solution1.getX() * precision) / precision);
			pos_solution1.setY(round(pos_solution1.getY() * precision) / precision);
			pos_solution2.setX(round(pos_solution2.getX() * precision) / precision);
			pos_solution2.setY(round(pos_solution2.getY() * precision) / precision);

			start.setX(round(start.getX() * precision) / precision);
			start.setY(round(start.getY() * precision) / precision);
			end.setX(round(end.getX() * precision) / precision);
			end.setY(round(end.getY() * precision) / precision);

			//pos_solution1 = end;

			if (max(start.getDistanceTo(pos_solution1), end.getDistanceTo(pos_solution1)) < start.getDistanceTo(end)
				|| max(start.getDistanceTo(pos_solution2), end.getDistanceTo(pos_solution2)) < start.getDistanceTo(end))
			{
				return true;
			}
		}
	}


	//	/*if (num_of_intersection_points == 2)
	//		{
	//			if (start.getDistanceTo(min(start.getDistanceTo(pos_solution1), start.getDistanceTo(pos_solution2))) <= start.getDistanceTo(end))
	//			{
	//				return true;
	//			}
	//			cout << "";
	//		}
	//		else if (num_of_intersection_points == 1)
	//		{
	//			if (start.getDistanceTo(start.getDistanceTo(pos_solution1)) <= start.getDistanceTo(end))
	//			{
	//				start.getDistanceTo(2);
	//				return true;
	//			}
	//		}*/


	///collision with lines
	////****need changes for robotO_balk
	//for (int i = 0; i<World::num_of_active_line_balks; i++)
	//		{
	//		Line balk = Line::makeLineFromTwoPoints(World::line_balk[i].start_point, World::line_balk[i].end_point);
	//		VecPosition intersection_point = line.getIntersection(balk);
	//		if (max(intersection_point.getDistanceTo(World::line_balk[i].start_point), intersection_point.getDistanceTo(World::line_balk[i].end_point)) <= World::line_balk[i].start_point.getDistanceTo(World::line_balk[i].end_point)
	//		&& max(intersection_point.getDistanceTo(start) , intersection_point.getDistanceTo(end)) <= start.getDistanceTo(end))
	//		{
	//		return true;
	//		}
	//		}
	//}



	//**check collison with paraline : worse
	/*Paraline paraline(start,end);
	for (int i = 0; i < World::num_of_active_robotO_circle_balks; i++)
	{
	if (paraline.hasIntersection(World::circle_balk[i],0))
	return true;
	}*/


	return false;
}
#if RECTANGULAR_PENALTY_AREA == 0
bool Balk::check_collision_with_penalty_area(VecPosition start, VecPosition end)
{
	/*double precision = 1000;
	start.setX(round(start.getX() * precision) / precision);
	start.setY(round(start.getY() * precision) / precision);

	end.setX(round(end.getX() * precision) / precision);
	end.setY(round(end.getY() * precision) / precision);*/

	///check collison with old alhorithm : better
	VecPosition pos_solution1;
	VecPosition pos_solution2;
	Line line = Line::makeLineFromTwoPoints(start, end);
	///collision with circles
	if (abs(end.getX()) < FieldLength / 2 && abs(start.getX()) < FieldLength / 2)		///for when we are allowed to move into penalty area when we want to go to back of penalty area or when we are in the back of penalty area
	{
		for (int i = 0; i < 4; i++)
		{
			/*if (start.isInCircle(world.robotT_circle_balk[i]))
				continue;*/
			if (start.isInCircle(world.penalty_area_circle_balk[i]) || end.isInCircle(world.penalty_area_circle_balk[i]))
				return true;

			//if (i == 0 && abs(e.x) > FieldLength / 2)		//for when we are allowed to move into penalty area when we want to go to back of penalty area
			//	break;
			int num_of_intersection_points = line.getCircleIntersectionPoints(World::penalty_area_circle_balk[i], &pos_solution1, &pos_solution2);
			if (num_of_intersection_points != 0)
			{
				/*if (abs(pos_solution1.getX()) > FieldLength / 2 && abs(pos_solution2.getX() > FieldLength / 2))
				return false;*/

				int precision = 100;
				pos_solution1.setX(round(pos_solution1.getX() * precision) / precision);
				pos_solution1.setY(round(pos_solution1.getY() * precision) / precision);
				pos_solution2.setX(round(pos_solution2.getX() * precision) / precision);
				pos_solution2.setY(round(pos_solution2.getY() * precision) / precision);

				start.setX(round(start.getX() * precision) / precision);
				start.setY(round(start.getY() * precision) / precision);
				end.setX(round(end.getX() * precision) / precision);
				end.setY(round(end.getY() * precision) / precision);

				if (max(start.getDistanceTo(pos_solution1), end.getDistanceTo(pos_solution1)) < start.getDistanceTo(end)
					|| max(start.getDistanceTo(pos_solution2), end.getDistanceTo(pos_solution2)) < start.getDistanceTo(end))
				{
					return true;
				}
			}
		}
	}

	//collision with lines
	//for (int i = 0; i<2; i++)
	//{
	//	/*if (i == 0 && abs(e.x) > FieldLength / 2)		//for when we are allowed to move into penalty area when we want to go to back of penalty area
	//		break;*/
	//	Line balk = Line::makeLineFromTwoPoints(World::penalty_area_line_balk[i].start_point, World::penalty_area_line_balk[i].end_point);
	//	VecPosition intersection_point = line.getIntersection(balk);
	//	if (max(intersection_point.getDistanceTo(World::penalty_area_line_balk[i].start_point), intersection_point.getDistanceTo(World::penalty_area_line_balk[i].end_point)) <= World::penalty_area_line_balk[i].start_point.getDistanceTo(World::penalty_area_line_balk[i].end_point)
	//	&& max(intersection_point.getDistanceTo(start) , intersection_point.getDistanceTo(end)) < start.getDistanceTo(end))
	//	{
	//		return true;
	//	}
	//}


	return false;
}
#else 
bool Balk::check_collision_with_penalty_area(VecPosition start, VecPosition end)
{
	if ((Field::getRightPenaltyArea().isInside(start) && Field::getRightPenaltyArea().isInside(end)) ||
		(Field::getLeftPenaltyArea().isInside(start) && Field::getLeftPenaltyArea().isInside(end)))
		return true;
	Paraline pl(start, end);
	if (pl.hasIntersection(Field::getUpParaline_LeftPenaltyArea()) || pl.hasIntersection(Field::getRightParaline_LeftPenaltyArea()) ||
		pl.hasIntersection(Field::getDownParaline_LeftPenaltyArea()) || pl.hasIntersection(Field::getLeftParaline_LeftPenaltyArea()) ||
		pl.hasIntersection(Field::getUpParaline_RightPenaltyArea()) || pl.hasIntersection(Field::getRightParaline_RightPenaltyArea()) ||
		pl.hasIntersection(Field::getDownParaline_RightPenaltyArea()) || pl.hasIntersection(Field::getLeftParaline_RightPenaltyArea()))
		return true;
	else
		return false;
}
bool Balk::check_collision_with_penalty_area(Paraline pl)
{
	if ((Field::getRightPenaltyArea().isInside(pl.getFirstPoint()) && Field::getRightPenaltyArea().isInside(pl.getSecondPoint())) ||
		(Field::getLeftPenaltyArea().isInside(pl.getFirstPoint()) && Field::getLeftPenaltyArea().isInside(pl.getSecondPoint())))
		return true;
	if (pl.hasIntersection(Field::getUpParaline_LeftPenaltyArea()) || pl.hasIntersection(Field::getRightParaline_LeftPenaltyArea()) ||
		pl.hasIntersection(Field::getDownParaline_LeftPenaltyArea()) || pl.hasIntersection(Field::getLeftParaline_LeftPenaltyArea()) ||
		pl.hasIntersection(Field::getUpParaline_RightPenaltyArea()) || pl.hasIntersection(Field::getRightParaline_RightPenaltyArea()) ||
		pl.hasIntersection(Field::getDownParaline_RightPenaltyArea()) || pl.hasIntersection(Field::getLeftParaline_RightPenaltyArea()))
		return true;
	else
		return false;
}
#endif
bool Balk::check_collision_with_ball_area(VecPosition start, VecPosition end, Ball::BalkMode ballBalkMode)
{
	//start and end should not be in balk
	///using world.numT and world.robotT_circle_balk[i] with each other is NOT GOOD!
	/*double precision = 1000;
	start.setX(round(start.getX() * precision) / precision);
	start.setY(round(start.getY() * precision) / precision);

	end.setX(round(end.getX() * precision) / precision);
	end.setY(round(end.getY() * precision) / precision);*/

	if (ballBalkMode == Ball::BalkMode::notBalk)
		return false;
	VecPosition pos_solution1;
	VecPosition pos_solution2;
	Line line = Line::makeLineFromTwoPoints(start, end);
	Circle c(world.ball.getCurrentBallPosition(), (ballBalkMode == Ball::BalkMode::balk) ? BALL_RADIUS + ROBOT_RADIUS : DISTANCE_TO_BALL_IN_STOP_MODE + ROBOT_RADIUS);
	///collision with circles
	/////////badd////////////
	/*if (start.isInCircle(c) || end.isInCircle(c))
		continue;*/
	if (start.isInCircle(c) || end.isInCircle(c))
		return true;
	int num_of_intersection_points = line.getCircleIntersectionPoints(c, &pos_solution1, &pos_solution2);
	if (num_of_intersection_points != 0)
	{
		int precision = 100;
		pos_solution1.setX(round(pos_solution1.getX() * precision) / precision);
		pos_solution1.setY(round(pos_solution1.getY() * precision) / precision);
		pos_solution2.setX(round(pos_solution2.getX() * precision) / precision);
		pos_solution2.setY(round(pos_solution2.getY() * precision) / precision);

		start.setX(round(start.getX() * precision) / precision);
		start.setY(round(start.getY() * precision) / precision);
		end.setX(round(end.getX() * precision) / precision);
		end.setY(round(end.getY() * precision) / precision);

		//pos_solution1 = end;

		if (max(start.getDistanceTo(pos_solution1), end.getDistanceTo(pos_solution1)) < start.getDistanceTo(end)
			|| max(start.getDistanceTo(pos_solution2), end.getDistanceTo(pos_solution2)) < start.getDistanceTo(end))
		{
			return true;
		}

	}

	return false;
	
}
bool Balk::check_collision_with_additional_cirlce_balks(VecPosition start, VecPosition end)
{
	VecPosition pos_solution1;
	VecPosition pos_solution2;
	Line line = Line::makeLineFromTwoPoints(start, end);
	//collision with circles
	for (int i = 0; i < world.num_of_additional_circle_balks; i++)
	{
		/*if (start.isInCircle(world.robotT_circle_balk[i]) || end.isInCircle(world.robotT_circle_balk[i]))
		continue;*/
		if (start.isInCircle(world.robotT_circle_balk[i]) || end.isInCircle(world.robotT_circle_balk[i]))
			return true;
		int num_of_intersection_points = line.getCircleIntersectionPoints(world.additional_circle_balks[i], &pos_solution1, &pos_solution2);
		if (num_of_intersection_points != 0)
		{
			int precision = 100;
			pos_solution1.setX(round(pos_solution1.getX() * precision) / precision);
			pos_solution1.setY(round(pos_solution1.getY() * precision) / precision);
			pos_solution2.setX(round(pos_solution2.getX() * precision) / precision);
			pos_solution2.setY(round(pos_solution2.getY() * precision) / precision);

			start.setX(round(start.getX() * precision) / precision);
			start.setY(round(start.getY() * precision) / precision);
			end.setX(round(end.getX() * precision) / precision);
			end.setY(round(end.getY() * precision) / precision);

			//pos_solution1 = end;

			if (max(start.getDistanceTo(pos_solution1), end.getDistanceTo(pos_solution1)) < start.getDistanceTo(end)
				|| max(start.getDistanceTo(pos_solution2), end.getDistanceTo(pos_solution2)) < start.getDistanceTo(end))
			{
				return true;
			}
		}
	}
	return false;
}
bool Balk::check_collision_with_additional_paraline_balks(VecPosition start, VecPosition end)
{
	cout << " function \"check_collision_with_additional_paraline_balks\"" << " doesnt have declaration" << endl;
	///check collison with old alhorithm : better
	//VecPosition pos_solution1;
	//VecPosition pos_solution2;
	//Line line = Line::makeLineFromTwoPoints(start, end);
	////collision with circles
	//for (int i = 0; i < world.num_of_additional_paraline_balks; i++)
	//{
	//	if (start.isInCircle(world.additional_circle_balks[i]) || end.isInCircle(world.additional_circle_balks[i]))
	//		return false;
	//	int num_of_intersection_points = line.getCircleIntersectionPoints(world.additional_circle_balks[i], &pos_solution1, &pos_solution2);
	//	if (num_of_intersection_points != 0)
	//	{
	//		int precision = 100;
	//		pos_solution1.setX(round(pos_solution1.getX() * precision) / precision);
	//		pos_solution1.setY(round(pos_solution1.getY() * precision) / precision);
	//		pos_solution2.setX(round(pos_solution2.getX() * precision) / precision);
	//		pos_solution2.setY(round(pos_solution2.getY() * precision) / precision);

	//		start.setX(round(start.getX() * precision) / precision);
	//		start.setY(round(start.getY() * precision) / precision);
	//		end.setX(round(end.getX() * precision) / precision);
	//		end.setY(round(end.getY() * precision) / precision);

	//		//pos_solution1 = end;

	//		if (max(start.getDistanceTo(pos_solution1), end.getDistanceTo(pos_solution1)) < start.getDistanceTo(end)
	//			|| max(start.getDistanceTo(pos_solution2), end.getDistanceTo(pos_solution2)) < start.getDistanceTo(end))
	//		{
	//			return true;
	//		}
	//	}
	//}
	//return false;

	return false;
}

///checks collision with all of four in the above
bool Balk::check_collision_with_(const VecPosition &start,const VecPosition &end, const short int &index_of_robot,const bool &robotT_is_balk,const bool &robotO_is_balk,const bool &penalty_area_is_balk, const Ball::BalkMode &ballBalkMode)
{
	return((robotT_is_balk && Balk::check_collision_with_robotT(start, end, index_of_robot)) ||
		   (robotO_is_balk && Balk::check_collision_with_robotO(start, end)) ||
		   (penalty_area_is_balk && Balk::check_collision_with_penalty_area(start, end)) ||
		   (ballBalkMode == Ball::BalkMode::notBalk ? false : Balk::check_collision_with_ball_area(start,end,ballBalkMode)));
}


///old ones!,maybe needed.not sure if it is working.
//bool Balk::check_collision(Cartesian_Coordinates s, Cartesian_Coordinates e, Circle circle_balk, Bar_Segment bar_balk)
//{
//
//	VecPosition start;
//	VecPosition end;
//
//	start.setX(s.x);
//	start.setY(s.y);
//
//	end.setX(e.x);
//	end.setY(e.y);
//
//	VecPosition pos_solution1;
//	VecPosition pos_solution2;
//
//	Line line = Line::makeLineFromTwoPoints(start, end);
//
//	//collision with circles
//	for (int i = 0; i<World::num_of_active_circle_balks; i++)
//	{
//		int num_of_intersection_points = line.getCircleIntersectionPoints(World::circle_balk[i], &pos_solution1, &pos_solution2);
//
//		if (num_of_intersection_points != 0)
//		{
//			if (max(start.getDistanceTo(pos_solution1), end.getDistanceTo(pos_solution1)) <= start.getDistanceTo(end))
//			{
//				circle_balk = World::circle_balk[i];
//				return true;
//			}
//		}
//
//		/*if (num_of_intersection_points == 2)
//		{
//		if (start.getDistanceTo(min(start.getDistanceTo(pos_solution1), start.getDistanceTo(pos_solution2))) <= start.getDistanceTo(end))
//		{
//		return true;
//		}
//		cout << "";
//		}
//		else if (num_of_intersection_points == 1)
//		{
//		if (start.getDistanceTo(start.getDistanceTo(pos_solution1)) <= start.getDistanceTo(end))
//		{
//		start.getDistanceTo(2);
//		return true;
//		}
//		}*/
//	}
//
//	//collision with lines
//	for (int i = 0; i<World::num_of_active_line_balks; i++)
//	{
//		Line balk = Line::makeLineFromTwoPoints(World::line_balk[i].start_point, World::line_balk[i].end_point);
//		VecPosition intersection_point = line.getIntersection(balk);
//
//		if (max(intersection_point.getDistanceTo(World::line_balk[i].start_point), intersection_point.getDistanceTo(World::line_balk[i].end_point)) <= World::line_balk[i].start_point.getDistanceTo(World::line_balk[i].end_point)
//			&& max(intersection_point.getDistanceTo(start), intersection_point.getDistanceTo(end)) <= start.getDistanceTo(end))
//		{
//			bar_balk = World::line_balk[i];
//			return true;
//		}
//
//	}
//
//	return false;
//}

///old ones!,maybe needed.not sure if it is working.
//bool Balk::check_collision(Cartesian_Coordinates s, Cartesian_Coordinates e)
//{
//	VecPosition start;
//	VecPosition end;
//
//	start.setX(s.x);
//	start.setY(s.y);
//
//	end.setX(e.x);
//	end.setY(e.y);
//
//	/*double precision = 1000;
//	start.setX(round(start.getX() * precision) / precision);
//	start.setY(round(start.getY() * precision) / precision);
//
//	end.setX(round(end.getX() * precision) / precision);
//	end.setY(round(end.getY() * precision) / precision);*/
//
//	///check collison with old alhorithm : better
//	VecPosition pos_solution1;
//	VecPosition pos_solution2;
//	Line line = Line::makeLineFromTwoPoints(start, end);
//	//collision with circles
//	for (int i = 0; i < World::num_of_active_circle_balks; i++)
//	{
//		int num_of_intersection_points = line.getCircleIntersectionPoints(World::circle_balk[i], &pos_solution1, &pos_solution2);
//		if (num_of_intersection_points != 0)
//		{
//			int precision = 100;
//			pos_solution1.setX(round(pos_solution1.getX() * precision) / precision);
//			pos_solution1.setY(round(pos_solution1.getY() * precision) / precision);
//			pos_solution2.setX(round(pos_solution2.getX() * precision) / precision);
//			pos_solution2.setY(round(pos_solution2.getY() * precision) / precision);
//			start.setX(round(start.getX() * precision) / precision);
//			start.setY(round(start.getY() * precision) / precision);
//			end.setX(round(end.getX() * precision) / precision);
//			end.setY(round(end.getY() * precision) / precision);
//			//pos_solution1 = end;
//			if (max(start.getDistanceTo(pos_solution1), end.getDistanceTo(pos_solution1)) < start.getDistanceTo(end) 
//				|| max(start.getDistanceTo(pos_solution2), end.getDistanceTo(pos_solution2)) < start.getDistanceTo(end))
//			{
//				return true;
//			}
//		}
//	}
//
//
//	//	/*if (num_of_intersection_points == 2)
//	//		{
//	//			if (start.getDistanceTo(min(start.getDistanceTo(pos_solution1), start.getDistanceTo(pos_solution2))) <= start.getDistanceTo(end))
//	//			{
//	//				return true;
//	//			}
//	//			cout << "";
//	//		}
//	//		else if (num_of_intersection_points == 1)
//	//		{
//	//			if (start.getDistanceTo(start.getDistanceTo(pos_solution1)) <= start.getDistanceTo(end))
//	//			{
//	//				start.getDistanceTo(2);
//	//				return true;
//	//			}
//	//		}*/
//
//
//	///collision with lines
//	//for (int i = 0; i<World::num_of_active_line_balks; i++)
//	//		{
//	//		Line balk = Line::makeLineFromTwoPoints(World::line_balk[i].start_point, World::line_balk[i].end_point);
//	//		VecPosition intersection_point = line.getIntersection(balk);
//	//		if (max(intersection_point.getDistanceTo(World::line_balk[i].start_point), intersection_point.getDistanceTo(World::line_balk[i].end_point)) <= World::line_balk[i].start_point.getDistanceTo(World::line_balk[i].end_point)
//	//		&& max(intersection_point.getDistanceTo(start) , intersection_point.getDistanceTo(end)) <= start.getDistanceTo(end))
//	//		{
//	//		return true;
//	//		}
//	//		}
//	//}
//
//
//
//	//**check collison with paraline : worse
//	/*Paraline paraline(start,end);
//	for (int i = 0; i < World::num_of_active_circle_balks; i++)
//	{
//		if (paraline.hasIntersection(World::circle_balk[i],0))
//			return true;
//	}*/
//
//
//	return false;
//}

///old ones!,maybe needed
//bool Balk::set_robotT_circle_balk(int num, VecPosition center, double radius, World &world)
//{
//	if (num >= 0 && num < World::num_of_allowed_robotT_circle_balks)
//	{
//		world.robotT_circle_balk[num].setCenter(center);
//		world.robotT_circle_balk[num].setRadius(radius);
//
//		/*for (int i = 0; i < World::num_of_active_circle_balks; i++)
//		{
//		if (World::index_of_active_circle_balks[i] == num)
//		return true;
//		}
//
//		World::num_of_active_circle_balks++;
//		World::index_of_active_circle_balks[World::num_of_active_circle_balks - 1] = num;*/
//
//		return true;
//	}
//
//	return false;
//}

///old ones!,maybe needed
//bool Balk::set_robotO_circle_balk(int num, VecPosition center, double radius, World &world)
//{
//	if (num >= 0 && num < World::num_of_allowed_robotT_circle_balks)
//	{
//		world.robotO_circle_balk[num].setCenter(center);
//		world.robotO_circle_balk[num].setRadius(radius);
//
//		/*for (int i = 0; i < World::num_of_active_circle_balks; i++)
//		{
//		if (World::index_of_active_circle_balks[i] == num)
//		return true;
//		}
//
//		World::num_of_active_circle_balks++;
//		World::index_of_active_circle_balks[World::num_of_active_circle_balks - 1] = num;*/
//
//		return true;
//	}
//
//	return false;
//}

///for when you want to change the defualt penalty area
//bool Balk::set_penalty_area_circle_balk(int num, double x, double y, double radius)
//{
//	if (num >= 1 && num <= 4)
//	{
//		World::penalty_area_circle_balk[num - 1].setCenter(VecPosition(x,y));
//		World::penalty_area_circle_balk[num - 1].setRadius(radius);
//
//		return true;
//	}
//
//	return false;
//}
///for when you want to change the defualt penalty area
//bool Balk::set_penalty_area_line_balk(int num, VecPosition start, VecPosition end)
//{
//	if (num >= 1 && num <= 2)
//	{
//
//		World::penalty_area_line_balk[num - 1].start_point = start;
//		World::penalty_area_line_balk[num - 1].end_point = end;
//
//		return true;
//	}
//
//	return false;
//}
///for when you want to set defualt penalty area.not needed.it has been set in world
//bool Balk::set_defualt_penalty_area()
//{
//
//	Balk::set_penalty_area_circle_balk(1, -4500, 250, 1000);		//circle penalty area left
//	Balk::set_penalty_area_circle_balk(2, -4500, -250, 1000);		//circle penalty area left
//
//	Balk::set_penalty_area_circle_balk(3, 4500, 250, 1000);			//circle penalty area right
//	Balk::set_penalty_area_circle_balk(4, 4500, -250, 1000);		//circle penalty area right
//
//	//RRT::PALU.setX(-3500);
//	//RRT::PALU.setY(250);
//	//RRT::PALD.setX(-3500);
//	//RRT::PALD.setY(-250);
//	Balk::set_penalty_area_line_balk(1, RRT::PALU, RRT::PALD);
//
//
//	//RRT::PARU.setX(3500);
//	//RRT::PARU.setY(250);
//	//RRT::PARD.setX(3500);
//	//RRT::PARD.setY(-250);
//	Balk::set_penalty_area_line_balk(2, RRT::PARU, RRT::PARD);
//
//	return true;
//}

///sets all of robots as a balk
void Balk::set_balks_in_world_object()
{
	///set temmate robot as balk
	for (int i = 0; i < world.numT; i++)
	{
		world.robotT_circle_balk[i].setCenter(world.robotT[i].position);
		world.robotT_circle_balk[i].setRadius(2 * ROBOT_RADIUS);
	}
	///set oponent robot as balk
	for (int i = 0; i <world.numO; i++)
	{
		world.robotO_circle_balk[i].setCenter(world.robotO[i].position);
		world.robotO_circle_balk[i].setRadius(2 * ROBOT_RADIUS);
	}
}







#if RECTANGULAR_PENALTY_AREA == 0
bool Balk::isInPenaltyArea(VecPosition position) 
{
	for (int i = 0; i < 4; i++)
	{
		if (i == 0 && abs(position.getX()) > FieldLength / 2)
			break;
		if (position.isInCircle(World::penalty_area_circle_balk[i]))
		{
			/*if (  abs(getX()) < abs(PARU.getX() + ROBOT_RADIUS)   &&   abs(getY()) > PARU.getY())	///can be changet to right or legt og a line
			{
			return false;
			}
			else
			return true;*/
			if (World::penalty_area_line_balk[0].isPositionInLeft(position) || World::penalty_area_line_balk[1].isPositionInRight(position))
				return true;
			else
				return false;
		}

	}
	return false;

}
#else
bool Balk::isInPenaltyArea(const VecPosition &position) 
{
	if (Field::getLeftPenaltyArea().isInside(position) || Field::getRightPenaltyArea().isInside(position))
		return true;
	else
		return false;
}
#endif
bool Balk::isInRobotT(const VecPosition &position)
{
	for (int i = 0; i < world.numT /* not good*/; i++)
	{
		if (position.isInCircle(world.robotT_circle_balk[i]))
			return true;
	}
	return false;
}
bool Balk::isInRobotTExcept(const VecPosition &position, const short int &robot_index)
{
	for (int i = 0; i < world.numT; i++)
	{
		if (i == robot_index)
			continue;
		if (position.isInCircle(world.robotT_circle_balk[i]))
			return true;
	}
	return false;
}
bool Balk::isInRobotO(const VecPosition &position)
{
	for (int i = 0; i < world.numO; i++)
	{
		if (position.isInCircle(world.robotO_circle_balk[i]))
			return true;
	}
	return false;
}
bool Balk::isInBallArea(const VecPosition &position, Ball::BalkMode bbm)
{
	Circle c (world.ball.getCurrentBallPosition(), (bbm == Ball::BalkMode::balk) ? BALL_RADIUS + ROBOT_RADIUS : (bbm == Ball::BalkMode::stopMode) ? DISTANCE_TO_BALL_IN_STOP_MODE + ROBOT_RADIUS : 0);
	return c.isInside(position);
}
bool Balk::isIn_(const VecPosition &position,const bool &robotT_is_Balk, const bool &robotO_is_Balk, const bool &penaltyArea_is_Balk, const short int &except_index)
{
	return ((Balk::isInRobotTExcept(position, except_index) && robotT_is_Balk) || (Balk::isInRobotO(position) && robotO_is_Balk) || (Balk::isInPenaltyArea(position) && penaltyArea_is_Balk));
}



///if position isnt in penalty area,it returns object itself
#if RECTANGULAR_PENALTY_AREA == 0
VecPosition Balk::getNearestPositionOutsideOfPenaltyArea(VecPosition position) 
{
	//VecPosition correctedPosition = *this;
	for (int i = 0; i < 4; i++)
	{
		if (i == 0 && abs(position.getX()) > FieldLength / 2)
			return position;
		if (position.isInCircle(World::penalty_area_circle_balk[i]))
		{
			VecPosition temp = position.getNearestPositionOutsideOfCircle(World::penalty_area_circle_balk[i]);
			position.setVecPosition(temp.getX(), temp.getY());
			///set dest to penalty area line if nearest position is on line
			if (abs(position.getX()) > abs(PARU.getX()) && (abs(position.getY()) < PARU.getY() && abs(position.getY()) > PARD.getY()))		// when destination is between circles and line of penaty area,this statement will correct destination
			{
				if (position.getX() > 0)
					position.setX(PARU.getX());
				else
					position.setX(PALU.getX());
			}
		}
	}
	return position;
}
#else
VecPosition Balk::getNearestPositionOutsideOfPenaltyArea(VecPosition position) 
{
	cout << "getNearestPositionOutsideOfPenaltyArea " << "doesnt have declaration" << endl;
	return false;
}
#endif

#if RECTANGULAR_PENALTY_AREA == 0
bool Balk::setOutsideOfPenaltyAreaIfIsInside(VecPosition position, short int robot_index)
{
	bool isInPenaltyArea = false;
	for (int i = 0; i < 4; i++)
	{
		if (i == 0 && abs(position.getX()) > FieldLength / 2)
			return false;
		if (position.isInCircle(World::penalty_area_circle_balk[i]))
		{
			isInPenaltyArea = true;
			position = position.getNearestPositionOutsideOfCircle(World::penalty_area_circle_balk[i]);

			///set dest to penalty area line if nearest position is on line
			if ((abs(position.getX()) > abs(PARU.getX()) - ROBOT_RADIUS) && (abs(position.getY()) < PARU.getY() && abs(position.getY()) > PARD.getY()))		/// when destination is between circles and line of penaty area,this statement will correct destination
			{
				if (position.getX() > 0)
					position.setX(PARU.getX() - ROBOT_RADIUS);
				else
					position.setX(PALU.getX() + ROBOT_RADIUS);
			}
		}
	}
	return isInPenaltyArea;
}
#else
bool Balk::setOutsideOfPenaltyAreaIfIsInside(VecPosition position, short int robot_index)
{
	cout << "setOutsideOfPenaltyAreaIfIsInside " << "doesnt hav declaration" << endl;
	return false;
}
#endif


//VecPosition Balk::getPositionOutsideOf_(VecPosition position, const bool &isRobotTBalk, const bool &isRobotOBalk, const bool &isPenaltyAreaBalk, bool &destination_is_in_balk, bool &destination_is_in_penalty_area, bool &destination_is_in_surround_field, const int &except_index) 
//{
//	if (!(destination_is_in_balk = (Balk::isInRobotTExcept(position,except_index) && isRobotTBalk)) & !(destination_is_in_balk = (Balk::isInRobotO(position) && isRobotOBalk)) & !(destination_is_in_penalty_area = (Balk::isInPenaltyArea(position) && isPenaltyAreaBalk)) & (destination_is_in_surround_field = Field::isInSurroundField(position)))
//		return position;
//
//	int main_circle_parts = 8;
//	double main_circle_radius = ROBOT_RADIUS / 10;
//	Circle main_circle(position, main_circle_radius);
//	double max_main_circle_radius = 3000;	//3 meter
//
//	for (; main_circle_radius < 3000; main_circle_radius += ROBOT_RADIUS)
//	{
//		//DrawShape::DrawCircle(Circle(*this, main_circle_radius));
//		for (int i = 0; i < main_circle_parts; i++)
//		{
//			VecPosition temp = position.getPositionToward((i*(2 * M_PI / main_circle_parts)), main_circle_radius);
//			//DrawShape::DrawDot(temp,20,0,0,256);
//			if (!(Balk::isInRobotTExcept(temp,except_index) && isRobotTBalk) && !(Balk::isInRobotO(temp) && isRobotOBalk) && !(Balk::isInPenaltyArea(temp) && isPenaltyAreaBalk) && Field::isInSurroundField(temp) && (destination_is_in_penalty_area ? Field::isInField(temp) : true))
//				return temp;
//		}
//	}
//	///not good!
//	return NULL;
//}

bool Balk::setPositionOutsideOf_(VecPosition &position, const bool &isRobotTBalk, const bool &isRobotOBalk, const bool &isPenaltyAreaBalk, Ball::BalkMode ballBalkMode, bool &destination_is_in_balk, bool &destination_is_in_penalty_area, bool &destination_is_in_surround_field, const int &except_index)
{
	if (!(destination_is_in_balk = (Balk::isInRobotTExcept(position,except_index) && isRobotTBalk)) &
		!(destination_is_in_balk = (Balk::isInRobotO(position) && isRobotOBalk)) &
		!(destination_is_in_penalty_area = (Balk::isInPenaltyArea(position) && isPenaltyAreaBalk)) &
		(destination_is_in_surround_field = Field::isInSurroundField(position)) &
		!((ballBalkMode == Ball::BalkMode::notBalk) ? false : Balk::isInBallArea(position, ballBalkMode)))
		return true;

	int main_circle_parts = 8;
	double main_circle_radius = ROBOT_RADIUS;
	double max_main_circle_radius = 3000;	//3 meter
	Circle main_circle(position, main_circle_radius);
	
	for (; main_circle_radius < 3000; main_circle_radius += ROBOT_RADIUS)
	{
		//DrawShape::DrawCircle(Circle(*this, main_circle_radius));
		for (int i = 0; i < main_circle_parts; i++)
		{
			VecPosition temp = position.getPositionToward((i*(2 * M_PI / main_circle_parts)), main_circle_radius);
			//DrawShape::DrawDot(temp,20,0,0,256);
			//Sleep(50);
			if (!(Balk::isInRobotTExcept(temp,except_index) && isRobotTBalk) && !(Balk::isInRobotO(temp) && isRobotOBalk) &&
				!(Balk::isInPenaltyArea(temp) && isPenaltyAreaBalk) && (Field::isInSurroundField(temp)) &&
				(destination_is_in_penalty_area ? Field::isInField(temp) : true) &&
				!((ballBalkMode == Ball::BalkMode::notBalk) ? false : Balk::isInBallArea(temp, ballBalkMode)))
			{
				position = temp;
				return true;
			}
		}
	}

	return NULL;
}

///doesnt have surround field check
//VecPosition Balk::getPositionOutsideOf_(VecPosition position, const bool &isRobotTBalk, const bool &isRobotOBalk, const bool &isPenaltyAreaBalk, const int &except_index) 
//{
//	bool isInPenaltyArea;
//	if (!((Balk::isInRobotTExcept(position, except_index) && isRobotTBalk)) & !((Balk::isInRobotO(position) && isRobotOBalk)) & !(isInPenaltyArea = (Balk::isInPenaltyArea(position) && isPenaltyAreaBalk)))
//		return position;
//
//	int main_circle_parts = 8;
//	double main_circle_radius = ROBOT_RADIUS / 10;
//	Circle main_circle(position, main_circle_radius);
//	double max_main_circle_radius = 3000;	//3 meter
//
//	for (; main_circle_radius < 3000; main_circle_radius += ROBOT_RADIUS)
//	{
//		//DrawShape::DrawCircle(Circle(*this, main_circle_radius));
//		for (int i = 0; i < main_circle_parts; i++)
//		{
//			VecPosition temp = position.getPositionToward((i*(2 * M_PI / main_circle_parts)), main_circle_radius);
//			//DrawShape::DrawDot(temp,20,0,0,256);
//			if (!(Balk::isInRobotTExcept(temp,except_index) && isRobotTBalk) && !(Balk::isInRobotO(temp) && isRobotOBalk) && !(Balk::isInPenaltyArea(temp) && isPenaltyAreaBalk) && Field::isInSurroundField(temp) && (isInPenaltyArea ? Field::isInField(temp) : true))
//				return temp;
//		}
//	}
//
//	return NULL;
//}

///doesnt have surround field check
//bool Balk::setPositionOutsideOf_(VecPosition &position, const bool &isRobotTBalk, const bool &isRobotOBalk, const bool &isPenaltyAreaBalk, const int &except_index)
//{
//	bool isInPenaltyArea;
//	if (!((Balk::isInRobotTExcept(position, except_index) && isRobotTBalk)) & !((Balk::isInRobotO(position) && isRobotOBalk)) & !(isInPenaltyArea = (Balk::isInPenaltyArea(position) && isPenaltyAreaBalk)))
//		return true;
//
//	int main_circle_parts = 8;
//	double main_circle_radius = ROBOT_RADIUS / 10;
//	Circle main_circle(position, main_circle_radius);
//	double max_main_circle_radius = 3000;	//3 meter
//
//	for (; main_circle_radius < 3000; main_circle_radius += ROBOT_RADIUS)
//	{
//		//DrawShape::DrawCircle(Circle(*this, main_circle_radius));
//		for (int i = 0; i < main_circle_parts; i++)
//		{
//			VecPosition temp = position.getPositionToward((i*(2 * M_PI / main_circle_parts)), main_circle_radius);
//			//DrawShape::DrawDot(temp,20,0,0,256);
//			if (!(Balk::isInRobotTExcept(temp, except_index) && isRobotTBalk) && !(Balk::isInRobotO(temp) && isRobotOBalk) && !(Balk::isInPenaltyArea(temp) && isPenaltyAreaBalk) && Field::isInSurroundField(temp) && (isInPenaltyArea ? Field::isInField(temp) : true))
//			{
//				position = temp;
//				return true;
//			}
//		}
//	}
//
//	return NULL;
//}

/*! CLASS BALK */


/*! CLASS RRT */

///if orgin is in a balk,it sticks
RRT::RRT_result RRT::MakeRRT(const int &robot_number, VecPosition destination_position_C, VecPosition* const optimized_pass, int &size_of_OP, const bool &robotT_is_balk, const bool &robotO_is_balk, const bool &penalty_area_is_balk, const Ball::BalkMode &ballBalkMode /*, const bool &robot_cant_go_outside_of_field*/)
{
	//add first check of collision
	///assumptions
	//we assume that we are allowed to move into penalty area when we want to go to back of penalty area or when we want to go to the field from back of penalty area
	//we assume that we cant go outside of field when we cant go inside penalty area
	//we check if destination is in the penalty area
	//we check if destination is in the balk
	//we dont check if destination is in the field
	//we asume that if panalty area is balk for a robot and that robot is in penalty area,the next postion that robot will go is nearest position outside the penalty area
	//REMOVED : robot_can_go_outside_of_field is showing that if route can pass outside of field or not , and it doesnt show that the destination can be outside of field or not
	//robot will go outside of field if destination is in outside of field
	//we assume that robot can go outside of field but in surround field

	//dakhel zamin = 6000*9000
	//kole zamin = 7400 * 10400
	//taghiir auto num_of_vertices
	//stop karadn auto jostejoo dar soorat peyda nakardan masir

	//current_position_C = World::get_robotT_location(robot_number, world);


	/*current_position_C.x = 0;
	current_position_C.y = 0;*/

	/*if (current_position_C.getX() == destination_position_C.getX && current_position_C.getY() == destination_position_C.getY())
	return NULL;*/

	//start_time = std::chrono::high_resolution_clock::now();

	is_check_r_max = false;
	is_check_angle_max = false;
	destination_estimation_circle_increase_angle = 0;
	destination_estimation_circle_is_found = false;
	graph = Graph();

	robot_index = world.getIndexForRobotTNumber(robot_number);
	if (j < 1)		//1 == numOfGraphs
	{
		current_position_C = world.get_robotT_position(robot_number);

		/*if (!Balk::check_collision_with_(current_position_C, destination_position_C, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, world))
		return ;*/
		/*if (current_position_C == false)
			return false;*/
		/*///check if DESTINATION is OUTSIDE OF SURROUND FIELD
		
		if (!Field::isInSurroundField(destination_position_C))
		{
			destination_is_outside_of_surround_field = true;
			destination_position_C.setPositionOutsideOf_(robotT_is_balk, robotO_is_balk, penalty_area_is_balk, robot_index);
		}*/

		///check if UNAUTHORIZED ROBOT is in PENALTY AREA
		unauthorized_robot_is_in_penalty_area = false;
		if (penalty_area_is_balk && Balk::isInPenaltyArea(world.robotT[robot_index].position))
			unauthorized_robot_is_in_penalty_area = true;

		//for (int i = 0; i < 4; i++)
		//{
		//	if (i == 0 && abs(CWorld.robotT[robot_index].position.getX()) > FieldLength / 2)
		//		break;
		//	if (CWorld.robotT[robot_index].position.isInCircle(World::penalty_area_circle_balk[i]))
		//	{
		//		if (!unauthorized_robot_is_in_penalty_area)
		//		{
		//			unauthorized_robot_is_in_penalty_area = true;
		//			destination_position_C = CWorld.robotT[robot_index].position;
		//		}
		//		destination_position_C = destination_position_C.getNearestPositionOutsideCircle(World::penalty_area_circle_balk[i]);
		//		if (abs(destination_position_C.getX()) > abs(PARU.getX()) && (  abs(destination_position_C.getY()) < PARU.getY()   &&   abs(destination_position_C.getY()) > PARD.getY()  ))		// when destination is between circles and line of penaty area,this statement will correct destination
		//		{
		//			if (destination_position_C.getX() > 0)
		//				destination_position_C.setX(PARU.getX());
		//			else
		//				destination_position_C.setX(PALU.getX());
		//		}
		//	}
		//}

		/////check if DESTINATION is in the PENALTY AREA for unauthorized robot,if yes,correct the destination
		//destination_is_in_penalty_area = false;
		//if (/*!destination_is_in_balk &&*/ penalty_area_is_balk && !unauthorized_robot_is_in_penalty_area)
		//{
		//	/*if (destination_position_C.setOutsideOfPenaltyAreaIfIsInside(robot_index))
		//		destination_is_in_penalty_area = true;*/
		//	if (destination_position_C.getPositionOutsideOf_(false, false, penalty_area_is_balk, robot_cant_go_outside_of_field, robot_index);
		//		destination_is_in_penalty_area = true;
		//	///not good way: need to double check the collision so destination_is_in_penalty_area can be valued correct
		//	/*destination_position_C = destination_position_C.getNearestPositionOutsideOfCurvedPenaltyArea();
		//	destination_is_in_penalty_area = true;*/
		//}

		/////check if DESTINATION is in the ROBOT BALK
		//destination_is_in_balk = false;
		//if (robotT_is_balk)
		//{
		//	/*for (int i = 0; i < world.numT; i++)
		//	{
		//	if (pow(destination_position_C.getX() - world.circle_balk[i].getCenter().getX(), 2) + pow(destination_position_C.getY() - World::circle_balk[i].getCenter().getY(), 2)
		//	< pow(World::circle_balk[i].getRadius(), 2))
		//	{
		//	destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::circle_balk[i]);
		//	destination_is_in_balk = true;
		//	}
		//	}*/
		//	for (int i = 0; i < CWorld.numT; i++)
		//	{
		//		if (i == robot_index)
		//			continue;
		//		if (destination_position_C.isInCircle(CWorld.robotT_circle_balk[i]))
		//		{
		//			destination_position_C = destination_position_C.getNearestPositionOutsideCircle(CWorld.robotT_circle_balk[i]);
		//			destination_is_in_balk = true;
		//			break;
		//		}
		//	}
		//}
		//if (!destination_is_in_balk && robotO_is_balk)
		//{
		//	for (int i = 0; i < CWorld.numO; i++)
		//	{
		//		if (destination_position_C.isInCircle(CWorld.robotO_circle_balk[i]))
		//		{
		//			destination_position_C = destination_position_C.getNearestPositionOutsideCircle(CWorld.robotO_circle_balk[i]);
		//			destination_is_in_balk = true;
		//			break;
		//		}
		//	}
		//}

		///check if DESTINATION is in the PENALTY AREA (for unauthorized robot) or ROBOT BALK or OUTSIDE OF FIELD or BALL BALK, if yes, correct the destination
		destination_is_in_penalty_area = false;
		destination_is_in_balk = false;
		destination_is_in_surround_field = false;
		/*if ((penalty_area_is_balk && !unauthorized_robot_is_in_penalty_area) || (robotT_is_balk) || (robotO_is_balk))*/
		Balk::setPositionOutsideOf_(destination_position_C, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, ballBalkMode, destination_is_in_balk, destination_is_in_penalty_area, destination_is_in_surround_field, robot_index);
		//DrawShape::DrawDot(destination_position_C, 50, 0, 0, 200);


		///check if any balk is between current_position_c and destination_position_c
		if (!Balk::check_collision_with_(current_position_C, destination_position_C, robot_index, robotT_is_balk, robotO_is_balk, !unauthorized_robot_is_in_penalty_area && penalty_area_is_balk, ballBalkMode))
		{
			size_of_OP = 1;
			optimized_pass[0] = destination_position_C;
			optimized_pass[1] = current_position_C;
			//end_time = std::chrono::high_resolution_clock::now();
			//auto time = end_time - start_time;
			//cout << time.count() / 1000000.0 << endl;
			return path_found;
		}

		///add current and destination to graph
		graph.add_vertex(current_position_C, -1);
		graph.add_vertex(destination_position_C, -2);
		current_position_C = graph.vertex[0];
		destination_position_C = graph.vertex[1];
		
		///set destination_estimation circle
		destination_estimation_circle.setCenter(destination_position_C);
		destination_estimation_circle.setRadius(destination_estimation_circle_radius_increase_rate);

		///convert cartesian to polar
		destination_position_P = destination_position_C.cartesian_to_polar();
		destination_position_P_from_CP = destination_position_C.cartesian_to_polar_from_CP(current_position_C);

		///check_r
		check_r = current_position_C.getDistanceTo(destination_position_C);
		if (destination_is_in_balk)
			check_r += robot_radius;		//robot radius
		distance_from_current_to_destination = check_r;

		///check_angle and distance_of_vertices
		check_angle = M_PI* pow(M_E, (-distance_from_current_to_destination / 5000));
		distance_of_vertices = 1200 + (-pow((distance_from_current_to_destination - 11000), 2)) / 121000;
	
		///main loop
		for (j = 0; j < 1; j++)		//1 == numOfGraphs
		{
			///determine max of check_r by airt of current position
			max_check_r = RRT::calculateMaxOfCheck_r(current_position_C);
		
			for (num_of_countinus_fails_to_find_random_crd = 0;graph.p < World::num_of_vertices - 1 && graph.q < World::num_of_vertices - 2 && !reach_destination && num_of_countinus_fails_to_find_random_crd<max_num_of_countinus_fails;)
			{
				
				///create random polar coordinates
				std::uniform_real_distribution<double> unif1(0, check_r);
				random_crd_P.setX(unif1(gen));
				std::uniform_real_distribution<double> unif2(destination_position_P_from_CP.getY() - (check_angle / 2), check_angle + destination_position_P_from_CP.getY() - (check_angle / 2));
				random_crd_P.setY(unif2(gen));

				///convert random polar coordinates to cartesian coordinates
				random_crd_C = random_crd_P.polar_to_cartesian();
				random_crd_C.setX(random_crd_C.getX() + current_position_C.getX());
				random_crd_C.setY(random_crd_C.getY() + current_position_C.getY());

				///check outside
				//if (!Field::isInField(random_crd_C))	/////////////////////////!!!!!!!!!!!!!
				//	continue;

				///check outside and behind goal of surround field for random_crd_C
				if (!Field::isInSurroundField_notInBehindGoal(random_crd_C))
				{
					num_of_countinus_fails_to_find_random_crd++;
					continue;
				}

				///check if random_crd_C is in penalty area (for unauthorized robot)
				/*if (!unauthorized_robot_is_in_penalty_area && penalty_area_is_balk)
				{
					if (Balk::isInPenaltyArea(random_crd_C))
					{
						num_of_countinus_fails_to_find_random_crd++;
						continue;
					}
				}*/

				///check if random_crd_C is in the balk
				//if (robotT_is_balk)
				//{
				//	/*for (int i = 0; i < CWorld.numT; i++)
				//	{
				//		if (i == robot_index)
				//			continue;
				//		if (random_crd_C.isInCircle(CWorld.robotT_circle_balk[i]))
				//		{
				//			flag = true;
				//			break;
				//		}
				//	}*/
				//	if (Balk::isInRobotTExcept(random_crd_C, robot_index))
				//		flag = true;
				//}
				//if (!flag && robotO_is_balk)
				//{
				//	/*for (int i = 0; i < CWorld.numO; i++)
				//	{
				//		if (random_crd_C.isInCircle(CWorld.robotO_circle_balk[i]))
				//		{
				//			flag = true;
				//			break;
				//		}
				//	}*/
				//	if (Balk::isInRobotO(random_crd_C))
				//		flag = true;
				//}
				//if (flag)
				//{
				//	flag = false;
				//	num_of_countinus_fails_to_find_random_crd++;
				//	continue;
				//}

				///reset num_of_countinues_fails,because appropriate random_crd found.
				//num_of_countinus_fails_to_find_random_crd = 0;


				///detect nearest vertex
				nearest_vertex = graph.nearest_vertex(random_crd_C);

				///not needed!!///////////////////////////////////////////////////////////////////////

				///check if nearest_vertex is in the balk
				/*if (robotT_is_balk)
				{
					if (Balk::isInRobotTExcept(nearest_vertex,robot_index))
						flag = true;
				}
				if (!flag && robotO_is_balk)
				{
					if (Balk::isInRobotO(nearest_vertex))
						flag = true;
				}
				if (flag)
				{
					flag = false;
					num_of_countinus_fails_to_find_random_crd++;
					continue;
				}*/
				///not needed!!///////////////////////////////////////////////////////////////////////


				///generate new vertex
				new_vertex = nearest_vertex.getPositionToward(random_crd_C, distance_of_vertices);

				///check outside and behind goal of surround field for new_vertex
				if (!Field::isInSurroundField_notInBehindGoal(new_vertex))
					continue;

				///check if new_vertex is in penalty area (for unauthorized robot)
				/*if (!unauthorized_robot_is_in_penalty_area && penalty_area_is_balk)
				{
					if (Balk::isInPenaltyArea(new_vertex))
						continue;
				}*/

				///check if new_vertex is in the balk
				/*if (robotT_is_balk)
				{
					if (Balk::isInRobotTExcept(new_vertex, robot_index))
						flag = true;
				}
				if (!flag && robotO_is_balk)
				{
					if (Balk::isInRobotO(new_vertex))
						flag = true;
				}
				if (flag)
				{
					flag = false;
					continue;
				}*/

				/*for (int i = 0; i < World::num_of_active_circle_balks; i++)
				{
				if (pow(new_vertex.getX() - World::circle_balk[i].getCenter().getX(), 2) + pow(new_vertex.getY() - World::circle_balk[i].getCenter().getY(), 2)
				<= pow(World::circle_balk[i].getRadius(), 2))
				{
				flag = false;
				break;
				}
				}
				if (!flag)
				{
				flag = true;
				continue;
				}*/

				//DrawShape::DrawDot(new_vertex,10);
				//cout << graph.p << endl;
				//Sleep(50);

				//DrawShape::ClearCircles();
				//DrawShape::DrawDot(nearest_vertex, 50, 0, 0, 255);
				//DrawShape::DrawDot(new_vertex, 50, 0, 0, 0);
				///check collision of path between nearest_verterx and new_vertex with everything
				if (Balk::check_collision_with_(nearest_vertex, new_vertex, robot_index, robotT_is_balk, robotO_is_balk, !unauthorized_robot_is_in_penalty_area && penalty_area_is_balk, ballBalkMode))
				{
					num_of_countinus_fails_to_find_random_crd++;
					continue;
				}
				
				///reset num_of_countinues_fails,because appropriate random_crd found.
				num_of_countinus_fails_to_find_random_crd = 0;

				///add new vertex
				graph.add_edge(nearest_vertex, new_vertex);
				graph.add_vertex(new_vertex, nearest_vertex.number);

				//DrawShape::DrawDot(new_vertex);
				//cout << graph.p << endl;
				//Sleep(50);

				///check if there is any balk between latest added vertex and destination
				VecPosition latest_vertex = graph.vertex[graph.p - 1];
				//!Balk::check_collision(latest_vertex, destination_position_C) && !(penalty_area_is_balk && Balk::check_collision_with_penalty_area(latest_vertex, destination_position_C))


				///check collision with destination
				if (graph.p < World::num_of_vertices - 1 && graph.q < World::num_of_vertices - 1
					&& !Balk::check_collision_with_(latest_vertex, destination_position_C, robot_index, robotT_is_balk, robotO_is_balk, !unauthorized_robot_is_in_penalty_area && penalty_area_is_balk, ballBalkMode))
				{
					reach_destination = true;
					graph.add_edge(latest_vertex, destination_position_C);
					graph.vertex[1].parent = latest_vertex.number;
					destination_position_C.parent = latest_vertex.number;
				}

			}

			///tasks to do when path not found
			if (!reach_destination)
			{
				try_time++;
				//DrawShape::ClearCircles();
				graph = Graph();
				graph.add_vertex(current_position_C, -1);
				graph.add_vertex(destination_position_C, -2);
				current_position_C = graph.vertex[0];
				destination_position_C = graph.vertex[1];
				j--;


				
				///for producing nearest destination out of ring of balks, if try_time is greater than max_try_time(cant find path)
				if (try_time > max_try_time)
				{

					if (destination_estimation_circle.getRadius() > max_of_destination_estimation_circle_radius)
						return path_not_found;
					if (destination_estimation_circle_increase_angle >= 2 * M_PI)
					{
						destination_estimation_circle.setRadius(destination_estimation_circle.getRadius() + destination_estimation_circle_radius_increase_rate);
						destination_estimation_circle_increase_angle = 0;
					}
					//DrawShape::DrawCircle(destination_estimation_circle);
					try_time = 1;
					
					///convert destination to nearest position on destination_estimation_circle to current_position_C
					
				
					VecPosition origin_pos = destination_estimation_circle.getNearestPointOnCircleTo(current_position_C);
					//DrawShape::DrawDot(origin_pos,40);
					
					double a = (destination_estimation_circle.getCenter()).getAngleToward(origin_pos);
					VecPosition obtained_pos = destination_estimation_circle.getPositionOnCircleWithAngle((destination_estimation_circle.getCenter()).getAngleToward(origin_pos) + destination_estimation_circle_increase_angle);
					//DrawShape::DrawDot(obtained_pos,40);
					
					if (destination_estimation_circle_increase_angle < 2 * M_PI)
						destination_estimation_circle_increase_angle += M_PI /*1*/;
					while (Balk::isIn_(obtained_pos, robotT_is_balk, robotO_is_balk, penalty_area_is_balk))
					{
						do
						{
							obtained_pos = destination_estimation_circle.getPositionOnCircleWithAngle((destination_estimation_circle.getCenter()).getAngleToward(origin_pos) + destination_estimation_circle_increase_angle);
							//DrawShape::DrawDot(obtained_pos,40);
							destination_estimation_circle_increase_angle += M_PI /*1*/;
						}while ((destination_estimation_circle_is_found = Balk::isIn_(obtained_pos, robotT_is_balk, robotO_is_balk, penalty_area_is_balk)) && destination_estimation_circle_increase_angle < 2 * M_PI);
						if (!destination_estimation_circle_is_found)
							break;
						destination_estimation_circle_increase_angle = 0;
						destination_estimation_circle.setRadius(destination_estimation_circle.getRadius() + destination_estimation_circle_radius_increase_rate);
						//DrawShape::DrawCircle(destination_estimation_circle);
						
						if (destination_estimation_circle.getRadius() > max_of_destination_estimation_circle_radius)
							return path_not_found;
						origin_pos = destination_estimation_circle.getNearestPointOnCircleTo(current_position_C);
						obtained_pos = origin_pos;
						//DrawShape::DrawDot(origin_pos,40);
						
					}

					destination_position_C = obtained_pos;

					//DrawShape::DrawDot(destination_position_C,40/*, 255,255,255*/);
					//DrawShape::DrawCircle(destination_estimation_circle);
					//Sleep(2000);
					///reset check_r and check_angle
					check_r = current_position_C.getDistanceTo(destination_position_C);
					if (destination_is_in_balk)
						check_r += robot_radius;		//robot radius
					distance_from_current_to_destination = check_r;

					check_angle = M_PI* pow(M_E, (-distance_from_current_to_destination / 5000));
					distance_of_vertices = 1200 + (-pow((distance_from_current_to_destination - 11000), 2)) / 121000;

					continue;
				}
				

				///for producing destination out of ring of balks, if try_time is greater than max_try_time(cant find path)
				//if (try_time > max_try_time)
				//{
				//	if (destination_estimation_circle.getRadius() > max_of_destination_estimation_circle_radius)
				//		return path_not_found;			
				//	do
				//	{
				//		destination_estimation_circle.setRadius(destination_estimation_circle.getRadius() + destination_estimation_circle_radius_increase_rate);
				//		destination_position_C = destination_estimation_circle.getNearestPointOnCircleTo(current_position_C);
				//		DrawShape::DrawCircle(destination_estimation_circle);
				//		getch();
				//		DrawShape::DrawDot(destination_position_C);
				//		getch();
				//	} while (destination_position_C.isIn_(robotT_is_balk, robotO_is_balk, penalty_area_is_balk, robot_index));
				//	try_time = 1;
				//	DrawShape::DrawCircle(destination_estimation_circle);
				//	getch();

				//	DrawShape::DrawDot(destination_position_C);
				//	getch();
				//	///reset check_r and check_angle
				//	check_r = current_position_C.getDistanceTo(destination_position_C);
				//	if (destination_is_in_balk)
				//		check_r += robot_radius;		//robot radius
				//	distance_from_current_to_destination = check_r;

				//	check_angle = M_PI* pow(M_E, (-distance_from_current_to_destination / 5000));
				//	distance_of_vertices = 1200 + (-pow((distance_from_current_to_destination - 11000), 2)) / 121000;

				//	continue;
				//}


				if (check_r < max_check_r - (check_r*check_r_increase) && try_time >= 0)
				{
					////////baraye afzayesh sorat dar entekhab maghsad dar nazdiki posht manee,check_r ra bishtar ziad mikonim.
					if (!destination_is_in_balk)
					{
						//check_r += 500;
						check_r = check_r + check_r*check_r_increase;
					}
					else
					{
						//check_r += 700;
						check_r = check_r + check_r*check_r_increase;
					}
				}
				else if (try_time >= 0)
				{
					is_check_r_max = true;
					check_r = max_check_r;
					
				}
				
				///old check_angle_increase
				if (check_angle <= 2 * M_PI - (check_angle * check_angle_increase) && try_time >= 0)
				{
					////////baraye afzayesh sorat vaghti mabda dar nazdiki posht manee bashad,check_angle ra bishtar ziad mikonim. .
					//check_angle += M_PI / 4;
					check_angle = check_angle + (check_angle*check_angle_increase);
				}
				else if (try_time >= 0)
				{
					
					is_check_angle_max = true;
					check_angle = 2 * M_PI;
				}
				
				///new check_angle_increase
				//check_angle_increase = ((check_r / max_check_r) * (max_check_angle_increase - min_check_angle_increase)) + min_check_angle_increase;
				//if (!is_check_angle_max && check_angle + check_angle_increase < 2*M_PI /*&& try_time >= 0*/)
				//	check_angle += check_angle_increase;
				//else /*if (try_time >= 0)*/
				//{
				//	is_check_angle_max = true;
				//	check_angle = 2 * M_PI;
				//}

				//DrawShape::DrawCircle(Circle(current_position_C, check_r));
				continue;
			}
			///tasks to do when path found
			reach_destination = false;
			//DrawShape::ClearCircles();///////////////////////////////////
			try_time = 1;
			graph.find_pass(current_position_C, destination_position_C, pass, size_of_pass);
			graph.optimize_pass(pass, optimized_pass, size_of_pass, size_of_optimized_pass, robot_index, robotT_is_balk, robotO_is_balk, !unauthorized_robot_is_in_penalty_area && penalty_area_is_balk, ballBalkMode);

			//end_time = std::chrono::high_resolution_clock::now();
			//auto time = end_time - start_time;
			//cout << time.count() / 1000000.0 << endl;
			/*if (time.count() / 1000000.0 > 10)
				cout << "bad rrt" << endl;*/
			size_of_OP = size_of_optimized_pass;
			//DrawShape::ClearCircles();
			return path_found;
		}
	}


}

//VecPosition* RRT::MakeRRT_withDrawOfPath(int robot_number, VecPosition destination_position_C, int &size_of_OP, World &world, bool robotT_is_balk, bool robotO_is_balk, bool penalty_area_is_balk, bool robot_can_go_outside_of_field)
//{
//	//add first check of collision
//
//	///assumptions
//	//we assume that we are allowed to move into penalty area when we want to go to back of penalty area or when we want to go to the field from back of penalty area
//	//we assume that we cant go outside of field when we cant go inside penalty area
//	//we check if destination is in the penalty area
//	//we check if destination is in the balk
//	//we dont check if destination is in the field
//	//robot_can_go_outside_of_field is showing that if route can pass outside of field or not , and it doesnt show that the destination can be outside of field or not
//
//	//dakhel zamin = 6000*9000
//	//kole zamin = 7400 * 10400
//	//taghiir auto num_of_vertices
//	//stop karadn auto jostejoo dar soorat peyda nakardan masir
//
//	const int num_of_graphs = 1;		//number of graphs to paint
//	int j = 0;		//number of painted graphs
//	double check_angle;
//	double check_r;
//	bool reach_destination = false;
//	double distance_of_vertices/* = 1000*/;	//700
//	bool flag = false;
//	bool is_check_r_max = false;
//	bool is_check_angle_max = false;
//	double distance_from_current_to_destination;
//	int try_time = 0;
//	bool destination_is_in_balk = false;
//	bool destination_is_in_penalty_area = false;
//	double robot_radius = 150;
//	bool destination_is_close_to_current = false;
//	VecPosition current_position_C;	//in Cartesian
//	VecPosition current_position_P(POLAR);		//in Polar
//	VecPosition destination_position_P(POLAR);	//in Polar
//	VecPosition destination_position_P_from_CP(POLAR);	//destination position in polar system that its center is curent position
//	VecPosition new_vertex;
//	////****hatman be dakhel for dar keshidan hame graph ha borde shavad.
//	VecPosition pass[World::num_of_vertices - 1];
//	int size_of_pass;
//	VecPosition optimized_pass[World::num_of_vertices - 1];
//	int size_of_optimized_pass;
//	////****
//
//	std::random_device rd;
//	std::mt19937 gen(rd());
//
//	std::chrono::steady_clock::time_point start_time;
//	std::chrono::steady_clock::time_point end_time;
//
//	double check_r_increase = 1.0 / 4.0;
//	double check_angle_increase = 2.0 / 3.0;
//
//	//current_position_C = World::get_robotT_location(robot_number, world);
//
//
//	/*current_position_C.x = 0;
//	current_position_C.y = 0;*/
//
//	/*if (current_position_C.getX() == destination_position_C.getX && current_position_C.getY() == destination_position_C.getY())
//	return NULL;*/
//
//
//	if (j < 1)		//1 == numOfGraphs
//	{
//
//		start_time = std::chrono::high_resolution_clock::now();
//
//		Balk::set_balks(world, robot_number);
//		current_position_C = World::get_robotT_position(robot_number, world);
//		Graph graph[1];
//
//		if (current_position_C == false)
//			return false;
//
//		///check if destination is in the robot balk
//		destination_is_in_balk = false;
//		if (robotT_is_balk)
//		{
//			/*for (int i = 0; i < world.numT; i++)
//			{
//			if (pow(destination_position_C.getX() - world.circle_balk[i].getCenter().getX(), 2) + pow(destination_position_C.getY() - World::circle_balk[i].getCenter().getY(), 2)
//			< pow(World::circle_balk[i].getRadius(), 2))
//			{
//			destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::circle_balk[i]);
//			destination_is_in_balk = true;
//			}
//			}*/
//
//			for (int i = 0; i < world.numT; i++)
//			{
//				if (destination_position_C.isInCircle(world.robotT_circle_balk[i]))
//				{
//					if (i == robot_number)
//					{
//						destination_is_close_to_current = true;
//						break;
//					}
//					destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, world.robotT_circle_balk[i]);
//					destination_is_in_balk = true;
//					break;
//				}
//			}
//		}
//		if (!destination_is_close_to_current && !destination_is_in_balk && robotO_is_balk)
//		{
//			for (int i = 0; i < world.numO; i++)
//			{
//				if (destination_position_C.isInCircle(world.robotO_circle_balk[i]))
//				{
//					destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, world.robotO_circle_balk[i]);
//					destination_is_in_balk = true;
//					break;
//				}
//			}
//		}
//
//
//		///check if destination is in the penalty area
//		if (/*!destination_is_in_balk &&*/ penalty_area_is_balk)
//		{
//			for (int i = 0; i < 4; i++)
//			{
//				if (i == 0 && abs(destination_position_C.getX()) > FieldLength / 2)
//					break;
//				if (destination_position_C.isInCircle(world.penalty_area_circle_balk[i]))
//				{
//					destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::penalty_area_circle_balk[i]);
//					if (abs(destination_position_C.getX()) > abs(PARU.getX()) && (abs(destination_position_C.getY()) < PARU.getY() && abs(destination_position_C.getY()) > PARD.getY()))		// when destination is between circles and line of penaty area,this statement will correct destination
//					{
//						if (destination_position_C.getX() > 0)
//							destination_position_C.setX(PARU.getX());
//						else
//							destination_position_C.setX(PALU.getX());
//					}
//					destination_is_in_penalty_area = true;
//				}
//			}
//		}
//
//		///add current position and destination position to graph
//		graph.add_vertex(current_position_C, -1);
//		graph.add_vertex(destination_position_C, -2);
//		current_position_C = graph.vertex[0];
//		destination_position_C = graph.vertex[1];
//
//		///convert cartesian to polar
//		destination_position_P = destination_position_C.cartesian_to_polar();
//		destination_position_P_from_CP = destination_position_C.cartesian_to_polar_from_CP(current_position_C);
//
//		///check_r
//		check_r = current_position_C.getDistanceTo(destination_position_C);
//		if (destination_is_in_balk)
//			check_r += robot_radius;		//robot radius
//		distance_from_current_to_destination = check_r;
//
//		///check_angle and distance_of_vertices
//		check_angle = M_PI* pow(M_E, (-distance_from_current_to_destination / 5000));
//		distance_of_vertices = 1200 + (-pow((distance_from_current_to_destination - 11000), 2)) / 121000;
//
//		///main loop
//		for (j = 0; j < 1; j++)		//1 == numOfGraphs
//		{
//			double max_check_r;
//
//
//			///check current position is in which airt
//			int current_airt = current_position_C.detect_airt();
//
//			///determine max of check_r by airt of current position
//			VecPosition t;
//			if (current_airt == 1)
//			{
//				t.setX(-FieldLength / 2);
//				t.setY(-FieldWidth / 2);
//				max_check_r = current_position_C.getDistanceTo(t);
//
//			}
//			else if (current_airt == 2)
//			{	
//				t.setX(FieldLength / 2);
//				t.setY(-FieldWidth / 2);
//				max_check_r = current_position_C.getDistanceTo(t);
//			}
//			else if (current_airt == 3)
//			{	
//				t.setX(FieldLength / 2);
//				t.setY(FieldWidth / 2);
//				max_check_r = current_position_C.getDistanceTo(t);
//			}
//			else
//			{
//				t.setX(-FieldLength / 2);
//				t.setY(FieldWidth / 2);
//				max_check_r = current_position_C.getDistanceTo(t);
//			}
//
//			for (int i = 0, e = 0;  i < 2 * world.num_of_vertices  &&  graph.p < world.num_of_vertices - 1
//				&& graph.q<world.num_of_vertices - 2 && !reach_destination;   i += 2, e++)
//			{
//
//
//
//
//				bool q = Balk::check_collision_with_(current_position_C, destination_position_C, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, world);
//
//
//
//
//
//				VecPosition random_crd_C;		//nearest coordinates
//				VecPosition random_crd_P(POLAR);			//random coordinates
//				VecPosition nearest_vertex;	//nearest coordinates
//
//											///create random polar coordinates
//				std::uniform_real_distribution<double> unif1(0, check_r);
//				random_crd_P.setX(unif1(gen));
//				std::uniform_real_distribution<double> unif2(destination_position_P_from_CP.getY() - (check_angle / 2), check_angle + destination_position_P_from_CP.getY() - (check_angle / 2));
//				random_crd_P.setY(unif2(gen));
//
//				///convert random polar coordinates to cartesian coordinates
//				random_crd_C = random_crd_P.polar_to_cartesian();
//				random_crd_C.setX(random_crd_C.getX() + current_position_C.getX());
//				random_crd_C.setY(random_crd_C.getY() + current_position_C.getY());
//
//				///check outside
//				if (!robot_can_go_outside_of_field && !Graph::is_in_field(random_crd_C))
//				{
//					i -= 2;
//					e--;
//					continue;
//				}
//
//				///detect nearest vertex
//				nearest_vertex = graph.nearest_vertex(random_crd_C);
//
//				///generate new vertex
//				new_vertex = Graph::calculate_vertex(nearest_vertex, random_crd_C, distance_of_vertices);
//
//				///check if new_vertex is in the balk
//				if (robotT_is_balk)
//				{
//					for (int i = 0; i < world.numT; i++)
//					{
//						if (new_vertex.isInCircle(world.robotT_circle_balk[i]))
//						{
//							flag = true;
//							break;
//						}
//					}
//				}
//				if (!flag && robotO_is_balk)
//				{
//					for (int i = 0; i < world.numO; i++)
//					{
//						if (new_vertex.isInCircle(world.robotO_circle_balk[i]))
//						{
//							flag = true;
//							break;
//						}
//					}
//				}
//				if (flag)
//				{
//					flag = false;
//					i -= 2;
//					e--;
//					continue;
//				}
//
//				/*for (int i = 0; i < World::num_of_active_circle_balks; i++)
//				{
//				if (pow(new_vertex.getX() - World::circle_balk[i].getCenter().getX(), 2) + pow(new_vertex.getY() - World::circle_balk[i].getCenter().getY(), 2)
//				<= pow(World::circle_balk[i].getRadius(), 2))
//				{
//				flag = false;
//				break;
//				}
//				}
//				if (!flag)
//				{
//				flag = true;
//				continue;
//				}*/
//
//				///check collision of path between nearest_verterx and new_vertex with everything
//				if (Balk::check_collision_with_(nearest_vertex, new_vertex, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, world))
//				{
//					i -= 2;
//					e--;
//					continue;
//				}
//
//				///add new vertex
//				graph.add_edge(nearest_vertex, new_vertex);
//				graph.add_vertex(new_vertex, nearest_vertex.number);
//
//				///check if there is any balk between latest added vertex and destination
//				VecPosition latest_vertex = graph.vertex[graph.p - 1];
//				//!Balk::check_collision(latest_vertex, destination_position_C) && !(penalty_area_is_balk && Balk::check_collision_with_penalty_area(latest_vertex, destination_position_C))
//
//				/*world.r[i] = graph.edge[e].start_vertex;
//				world.r[i + 1] = graph.edge[e].end_vertex;
//				Sleep(1000);*/
//
//				///check collision with destination
//				if (graph.p < world.num_of_vertices - 1 && graph.q < world.num_of_vertices - 1
//					&& !Balk::check_collision_with_(latest_vertex, destination_position_C, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, world))
//				{
//					reach_destination = true;
//					graph.add_edge(latest_vertex, destination_position_C);
//					graph.vertex[1].parent = latest_vertex.number;
//					i += 2;
//					e++;
//					destination_position_C.parent = latest_vertex.number;
//				}
//
//			}
//
//			///tasks to do when path not found
//			if (!reach_destination)
//			{
//				graph = Graph();
//				graph.add_vertex(current_position_C, -1);
//				graph.add_vertex(destination_position_C, -2);
//				current_position_C = graph.vertex[0];
//				destination_position_C = graph.vertex[1];
//				j--;
//				//memset(world.r, NULL, sizeof(world.r));
//				try_time++;
//
//				if (check_r < max_check_r - (check_r*check_r_increase) && try_time >= 0)
//				{
//					////////baraye afzayesh sorat dar entekhab maghsad dar nazdiki posht manee,check_r ra bishtar ziad mikonim.
//					if (!destination_is_in_balk)
//					{
//						//check_r += 500;
//						check_r = check_r + check_r*check_r_increase;
//						//cout << "b" << endl;
//					}
//					else
//					{
//						//check_r += 700;
//						check_r = check_r + check_r*check_r_increase;
//						//cout << "b" << endl;
//					}
//				}
//				else if (try_time >= 0)
//				{
//					is_check_r_max = true;
//					check_r = max_check_r;
//				}
//
//
//
//				if (check_angle <= 2 * M_PI - (check_angle * check_angle_increase) && try_time >= 0)
//				{
//					////////baraye afzayesh sorat vaghti mabda dar nazdiki posht manee bashad,check_angle ra bishtar ziad mikonim. .
//					//check_angle += M_PI / 4;
//					check_angle = check_angle + (check_angle*check_angle_increase);
//					//cout << "a" << endl;
//				}
//				else if (try_time >= 0)
//				{
//					is_check_angle_max = true;
//					check_angle = 2 * M_PI;
//				}
//
//
//				//cout << "t" << endl;
//				continue;
//
//			}
//
//			///tasks to do when path found
//			reach_destination = false;
//			try_time = 0;
//			graph.find_pass(current_position_C, destination_position_C, pass, size_of_pass);
//			graph.optimum_pass(pass, optimized_pass, size_of_pass, size_of_optimized_pass, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, world);
//
//			//memset(world.r, NULL, sizeof(world.r));
//
//			/*for (int i = 0, j = 0; i + 1 < size_of_optimized_pass; i++, j += 2)
//			{
//				world.r[j] = optimized_pass[i];
//				world.r[j + 1] = optimized_pass[i + 1];
//				
//			}*/
//
//			end_time = std::chrono::high_resolution_clock::now();
//			auto time = end_time - start_time;
//			cout << time.count() / 1000000.0 << endl;
//
//			size_of_OP = size_of_optimized_pass;
//			return optimized_pass;
//		}
//	}
//
//
//}

double RRT::calculateMaxOfCheck_r(const VecPosition &current_position)	///determine max of check_r by airt of current position
{
	///check current position is in which airt
	int current_airt = Field::detect_airt(current_position);

	///determine max of check_r by airt of current position
	if (current_airt == 1)
	{
		VecPosition t;
		t.setX(-SurroundFieldLength / 2);
		t.setY(-SurroundFieldWidth / 2);
		return current_position.getDistanceTo(t);

	}
	else if (current_airt == 2)
	{
		VecPosition t;
		t.setX(SurroundFieldLength / 2);
		t.setY(-SurroundFieldWidth / 2);
		return current_position.getDistanceTo(t);
	}
	else if (current_airt == 3)
	{
		VecPosition t;
		t.setX(SurroundFieldLength / 2);
		t.setY(SurroundFieldWidth / 2);
		return current_position.getDistanceTo(t);
	}
	else
	{
		VecPosition t;
		t.setX(-SurroundFieldLength / 2);
		t.setY(SurroundFieldWidth / 2);
		return current_position.getDistanceTo(t);
	}
}

///previous version of MakeRRT
//VecPosition* RRT::MakeRRT(int robot_number, VecPosition destination_position_C, int &size_of_OP, World &world, bool robotT_is_balk, bool robotO_is_balk, bool penalty_area_is_balk, bool robot_can_go_outside_of_field)
//{
//	//add first check of collision
//
//	///assumptions
//	//we assume that we are allowed to move into penalty area when we want to go to back of penalty area or when we want to go to the field from back of penalty area
//	//we assume that we cant go outside of field when we cant go inside penalty area
//	//we check if destination is in the penalty area
//	//we check if destination is in the balk
//	//we dont check if destination is in the field
//	//robot_can_go_outside_of_field is showing that if route can pass outside of field or not , and it doesnt show that the destination can be outside of field or not
//
//	//dakhel zamin = 6000*9000
//	//kole zamin = 7400 * 10400
//	//taghiir auto num_of_vertices
//	//stop karadn auto jostejoo dar soorat peyda nakardan masir
//
//	const int num_of_graphs = 1;		//number of graphs to paint
//	int j = 0;		//number of painted graphs
//	double check_angle;
//	double check_r;
//	bool reach_destination = false;
//	double distance_of_vertices/* = 1000*/;	//700
//	bool flag = true;
//	bool is_check_r_max = false;
//	bool is_check_angle_max = false;
//	double distance_from_current_to_destination;
//	int try_time = 0;
//	bool destination_is_in_balk = false;
//	bool destination_is_in_penalty_area = false;
//	double robot_radius = 150;
//	VecPosition current_position_C;	//in Cartesian
//	VecPosition current_position_P(POLAR);		//in Polar
//	VecPosition destination_position_P(POLAR);	//in Polar
//	VecPosition destination_position_P_from_CP(POLAR);	//destination position in polar system that its center is curent position
//	VecPosition new_vertex;
//	////****hatman be dakhel for dar keshidan hame graph ha borde shavad.
//	VecPosition pass[World::num_of_vertices - 1];
//	int size_of_pass;
//	VecPosition optimized_pass[World::num_of_vertices - 1];
//	int size_of_optimized_pass;
//	////****
//
//	std::random_device rd;
//	std::mt19937 gen(rd());
//
//	std::chrono::steady_clock::time_point start_time;
//	std::chrono::steady_clock::time_point end_time;
//
//	double check_r_increase = 1.0 / 4.0;
//	double check_angle_increase = 2.0 / 3.0;
//
//	//current_position_C = World::get_robotT_location(robot_number, world);
//
//
//	/*current_position_C.x = 0;
//	current_position_C.y = 0;*/
//
//	/*if (current_position_C.getX() == destination_position_C.getX && current_position_C.getY() == destination_position_C.getY())
//	return NULL;*/
//
//
//	if (j < 1)		//1 == numOfGraphs
//	{
//		start_time = std::chrono::high_resolution_clock::now();
//
//		Balk::set_balks(world, robot_number);
//		Graph graph[1];
//
//		current_position_C = World::get_robotT_position(robot_number, world);
//
//
//		///check if destination is in the robot balk
//		destination_is_in_balk = false;
//		if (robotT_is_balk)
//		{
//			/*for (int i = 0; i < world.numT; i++)
//			{
//			if (pow(destination_position_C.getX() - world.circle_balk[i].getCenter().getX(), 2) + pow(destination_position_C.getY() - World::circle_balk[i].getCenter().getY(), 2)
//			< pow(World::circle_balk[i].getRadius(), 2))
//			{
//			destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::circle_balk[i]);
//			destination_is_in_balk = true;
//			}
//			}*/
//
//			for (int i = 0; i < world.numT; i++)
//			{
//				if (destination_position_C.isInCircle(world.robotT_circle_balk[i]))
//				{
//					destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, world.robotT_circle_balk[i]);
//					destination_is_in_balk = true;
//					break;
//				}
//			}
//		}
//		if (robotO_is_balk)
//		{
//			for (int i = 0; i < world.numO; i++)
//			{
//				if (destination_position_C.isInCircle(world.robotO_circle_balk[i]))
//				{
//					destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, world.robotO_circle_balk[i]);
//					destination_is_in_balk = true;
//					break;
//				}
//			}
//		}
//
//
//		///check if destination is in the penalty area
//		if (/*!destination_is_in_balk &&*/ penalty_area_is_balk)
//		{
//			for (int i = 0; i < 4; i++)
//			{
//				if (i == 0 && abs(destination_position_C.getX()) > FieldLength / 2)
//					break;
//				if (destination_position_C.isInCircle(world.penalty_area_circle_balk[i]))
//				{
//					destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::penalty_area_circle_balk[i]);
//					if (abs(destination_position_C.getX()) > abs(RRT::PARU.getX()) && (abs(destination_position_C.getY()) < RRT::PARU.getY() && abs(destination_position_C.getY()) > RRT::PARD.getY()))		// when destination is between circles and line of penaty area,this statement will correct destination
//					{
//						if (destination_position_C.getX() > 0)
//							destination_position_C.setX(RRT::PARU.getX());
//						else
//							destination_position_C.setX(RRT::PALU.getX());
//					}
//					destination_is_in_penalty_area = true;
//				}
//			}
//		}
//
//		///add currnt and destination to graph
//		graph.add_vertex(current_position_C, -1);
//		graph.add_vertex(destination_position_C, -2);
//		current_position_C = graph.vertex[0];
//		destination_position_C = graph.vertex[1];
//
//		///convert cartesian to polar
//		destination_position_P = destination_position_C.cartesian_to_polar();
//		destination_position_P_from_CP = destination_position_C.cartesian_to_polar_from_CP(current_position_C);
//
//		///check_r
//		check_r = current_position_C.getDistanceTo(destination_position_C);
//		if (destination_is_in_balk)
//			check_r += robot_radius;		//robot radius
//		distance_from_current_to_destination = check_r;
//
//		///check_angle and distance_of_vertices
//		check_angle = M_PI* pow(M_E, (-distance_from_current_to_destination / 5000));
//		distance_of_vertices = 1200 + (-pow((distance_from_current_to_destination - 11000), 2)) / 121000;
//
//		///main loop
//		for (j = 0; j < 1; j++)		//1 == numOfGraphs
//		{
//			double max_check_r;
//
//			///check current position is in which airt
//			int current_airt = current_position_C.detect_airt();
//
//			///determine max of check_r by airt of current position
//			if (current_airt == 1)
//			{
//				VecPosition t;
//				t.setX(-FieldLength / 2);
//				t.setY(-FieldWidth / 2);
//				max_check_r = current_position_C.getDistanceTo(t);
//
//			}
//			else if (current_airt == 2)
//			{
//				VecPosition t;
//				t.setX(FieldLength / 2);
//				t.setY(-FieldWidth / 2);
//				max_check_r = current_position_C.getDistanceTo(t);
//			}
//			else if (current_airt == 3)
//			{
//				VecPosition t;
//				t.setX(FieldLength / 2);
//				t.setY(FieldWidth / 2);
//				max_check_r = current_position_C.getDistanceTo(t);
//			}
//			else
//			{
//				VecPosition t;
//				t.setX(-FieldLength / 2);
//				t.setY(FieldWidth / 2);
//				max_check_r = current_position_C.getDistanceTo(t);
//			}
//
//			while (graph.p < world.num_of_vertices - 1 && graph.q < world.num_of_vertices - 2 && !reach_destination)
//			{
//				VecPosition random_crd_C;		//nearest coordinates
//				VecPosition random_crd_P(POLAR);			//random coordinates
//				VecPosition nearest_vertex;	//nearest coordinates
//
//											///create random polar coordinates
//				std::uniform_real_distribution<double> unif1(0, check_r);
//				random_crd_P.setX(unif1(gen));
//				std::uniform_real_distribution<double> unif2(destination_position_P_from_CP.getY() - (check_angle / 2), check_angle + destination_position_P_from_CP.getY() - (check_angle / 2));
//				random_crd_P.setY(unif2(gen));
//
//				///convert random polar coordinates to cartesian coordinates
//				random_crd_C = random_crd_P.polar_to_cartesian();
//				random_crd_C.setX(random_crd_C.getX() + current_position_C.getX());
//				random_crd_C.setY(random_crd_C.getY() + current_position_C.getY());
//
//				///check outside
//				if (!robot_can_go_outside_of_field && !Graph::is_in_field(random_crd_C))
//					continue;
//
//				///detect nearest vertex
//				nearest_vertex = graph.nearest_vertex(random_crd_C);
//
//				///generate new vertex
//				new_vertex = Graph::calculate_vertex(nearest_vertex, random_crd_C, distance_of_vertices);
//
//				///check if new_vertex is in the balk
//				if (robotT_is_balk)
//				{
//					for (int i = 0; i < world.numT; i++)
//					{
//						if (destination_position_C.isInCircle(world.robotT_circle_balk[i]))
//						{
//							flag = false;
//							break;
//						}
//					}
//				}
//				if (robotO_is_balk)
//				{
//					for (int i = 0; i < world.numO; i++)
//					{
//						if (destination_position_C.isInCircle(world.robotO_circle_balk[i]))
//						{
//							flag = false;
//							break;
//						}
//					}
//				}
//				if (!flag)
//				{
//					flag = true;
//					continue;
//				}
//
//				/*for (int i = 0; i < World::num_of_active_circle_balks; i++)
//				{
//				if (pow(new_vertex.getX() - World::circle_balk[i].getCenter().getX(), 2) + pow(new_vertex.getY() - World::circle_balk[i].getCenter().getY(), 2)
//				<= pow(World::circle_balk[i].getRadius(), 2))
//				{
//				flag = false;
//				break;
//				}
//				}
//				if (!flag)
//				{
//				flag = true;
//				continue;
//				}*/
//
//				///check collision of path between nearest_verterx and new_vertex with everything
//				if (Balk::check_collision_with_(nearest_vertex, new_vertex, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, world))
//				{
//					continue;
//				}
//
//				///add new vertex
//				graph.add_edge(nearest_vertex, new_vertex);
//				graph.add_vertex(new_vertex, nearest_vertex.number);
//
//				///check if there is any balk between latest added vertex and destination
//				VecPosition latest_vertex = graph.vertex[graph.p - 1];
//				//!Balk::check_collision(latest_vertex, destination_position_C) && !(penalty_area_is_balk && Balk::check_collision_with_penalty_area(latest_vertex, destination_position_C))
//
//				///check collision with destination
//				if (graph.p < world.num_of_vertices - 1 && graph.q < world.num_of_vertices - 1
//					&& !Balk::check_collision_with_(latest_vertex, destination_position_C, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, world))
//				{
//					reach_destination = true;
//					graph.add_edge(latest_vertex, destination_position_C);
//					graph.vertex[1].parent = latest_vertex.number;
//					destination_position_C.parent = latest_vertex.number;
//				}
//			}
//
//			///tasks to do when path not found
//			if (!reach_destination)
//			{
//				graph = Graph();
//				graph.add_vertex(current_position_C, -1);
//				graph.add_vertex(destination_position_C, -2);
//				current_position_C = graph.vertex[0];
//				destination_position_C = graph.vertex[1];
//				j--;
//				//memset(world.r, NULL, sizeof(world.r));
//				try_time++;
//
//				if (check_r < max_check_r - (check_r*check_r_increase) && try_time >= 0)
//				{
//					////////baraye afzayesh sorat dar entekhab maghsad dar nazdiki posht manee,check_r ra bishtar ziad mikonim.
//					if (!destination_is_in_balk)
//					{
//						//check_r += 500;
//						check_r = check_r + check_r*check_r_increase;
//						//cout << "b" << endl;
//					}
//					else
//					{
//						//check_r += 700;
//						check_r = check_r + check_r*check_r_increase;
//						//cout << "b" << endl;
//					}
//				}
//				else if (try_time >= 0)
//				{
//					is_check_r_max = true;
//					check_r = max_check_r;
//				}
//
//
//
//				if (check_angle <= 2 * M_PI - (check_angle * check_angle_increase) && try_time >= 0)
//				{
//					////////baraye afzayesh sorat vaghti mabda dar nazdiki posht manee bashad,check_angle ra bishtar ziad mikonim. .
//					//check_angle += M_PI / 4;
//					check_angle = check_angle + (check_angle*check_angle_increase);
//					//cout << "a" << endl;
//				}
//				else if (try_time >= 0)
//				{
//					is_check_angle_max = true;
//					check_angle = 2 * M_PI;
//				}
//
//
//				//cout << "t" << endl;
//				continue;
//
//			}
//
//			///tasks to do when path found
//			reach_destination = false;
//			try_time = 0;
//			graph.find_pass(current_position_C, destination_position_C, pass, size_of_pass);
//			graph.optimum_pass(pass, optimized_pass, size_of_pass, size_of_optimized_pass, robotT_is_balk, robotO_is_balk, penalty_area_is_balk, world);
//
//			/*memset(world.r, NULL, sizeof(world.r));
//
//			VecPosition tempvec;
//			for (int i = 0, j = 0; i + 1 < size_of_optimized_pass; i++, j += 2)
//			{
//			tempvec.setX(optimized_pass[i].x);
//			tempvec.setY(optimized_pass[i].y);
//			world.r[j] = tempvec;
//
//			tempvec.setX(optimized_pass[i + 1].x);
//			tempvec.setY(optimized_pass[i + 1].y);
//			world.r[j + 1] = tempvec;
//			}*/
//
//			end_time = std::chrono::high_resolution_clock::now();
//			auto time = end_time - start_time;
//			cout << time.count() / 1000000.0 << endl;
//
//			size_of_OP = size_of_optimized_pass;
//			return optimized_pass;
//		}
//	}
//
//
//}
///previous version of MakeRRT_with_draw_of_path
//VecPosition* RRT::MakeRRT_with_draw_of_path(int robot_number, World &world, VecPosition destination_position_C, int &size_of_OP, bool robotT_is_balk, bool robotO_is_balk, bool penalty_area_is_balk, bool robot_can_go_outside_of_field)
//{
//
//	
//	//add first check of collision
//
//	///assumptions
//	//we assume that we are allowed to move into penalty area when we want to go to back of penalty area or when we want to go to the field from back of penalty area
//	//we assume that we cant go outside of field when we cant go inside penalty area
//	//we check if destination is in the penalty area
//	//we check if destination is in the balk
//	//we dont check if destination is in the field
//	//robot_can_go_outside_of_field is showing that if route can pass outside of field or not , and it doesnt show that the destination can be outside of field or not
//
//	//dakhel zamin = 6000*9000
//	//kole zamin = 7400 * 10400
//	//taghiir auto num_of_vertices
//	//stop karadn auto jostejoo dar soorat peyda nakardan masir
//
//	const int num_of_graphs = 1;		//number of graphs to paint
//	int j = 0;		//number of painted graphs
//	double check_angle;
//	double check_r;
//	bool reach_destination = false;
//	double distance_of_vertices/* = 1000*/;	//700
//	bool flag = true;
//	bool is_check_r_max = false;
//	bool is_check_angle_max = false;
//	double distance_from_current_to_destination;
//	int try_time = 0;
//	bool destination_is_in_balk = false;
//	bool destination_is_in_penalty_area = false;
//	double robot_radius = 150;
//	VecPosition current_position_C;	//in Cartesian
//	VecPosition current_position_P(POLAR);		//in Polar
//	VecPosition destination_position_P(POLAR);	//in Polar
//	VecPosition destination_position_P_from_CP(POLAR);	//destination position in polar system that its center is curent position
//	VecPosition new_vertex;
//	////****hatman be dakhel for dar keshidan hame graph ha borde shavad.
//	VecPosition pass[World::num_of_vertices - 1];
//	int size_of_pass;
//	VecPosition optimized_pass[World::num_of_vertices - 1];
//	int size_of_optimized_pass;
//	////****
//
//	std::random_device rd;
//	std::mt19937 gen(rd());
//
//	std::chrono::steady_clock::time_point start_time;
//	std::chrono::steady_clock::time_point end_time;
//
//	double check_r_increase = 1.0 / 4.0;
//	double check_angle_increase = 2.0 / 3.0;
//
//	//current_position_C = World::get_robotT_location(robot_number, world);
//
//
//	/*current_position_C.x = 0;
//	current_position_C.y = 0;*/
//
//	/*if (current_position_C.getX() == destination_position_C.getX && current_position_C.getY() == destination_position_C.getY())
//	return NULL;*/
//
//
//	if (j < 1)		//1 == numOfGraphs
//	{
//	start_time = std::chrono::high_resolution_clock::now();
//
//	Graph graph[1];
//
//	current_position_C = World::get_robotT_position(robot_number, world);
//
//
//	//check if destination is in the balk
//	destination_is_in_balk = false;
//	for (int i = 0; i < World::num_of_active_circle_balks; i++)
//	{
//		if (pow(destination_position_C.getX() - World::circle_balk[i].getCenter().getX(), 2) + pow(destination_position_C.getY() - World::circle_balk[i].getCenter().getY(), 2)
//			< pow(World::circle_balk[i].getRadius(), 2))
//		{
//			destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::circle_balk[i]);
//			destination_is_in_balk = true;
//		}
//	}
//
//	//check if destination is in the penalty area
//	if (/*!destination_is_in_balk &&*/ penalty_area_is_balk)
//	{
//		for (int i = 0; i < 4; i++)
//		{
//			if (i == 0 && abs(destination_position_C.getX()) > FieldLength / 2)
//				break;
//			if (pow(destination_position_C.getX() - World::penalty_area_circle_balk[i].getCenter().getX(), 2) + pow(destination_position_C.getY() - World::penalty_area_circle_balk[i].getCenter().getY(), 2)
//				< pow(World::penalty_area_circle_balk[i].getRadius(), 2))
//			{
//				destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::penalty_area_circle_balk[i]);
//				if (abs(destination_position_C.getX()) > abs(RRT::PARU.getX()) && (abs(destination_position_C.getY()) < RRT::PARU.getY() && abs(destination_position_C.getY()) > RRT::PARD.getY()))		// when destination is between circles and line of penaty area,this statement will correct destination
//				{
//					if (destination_position_C.getX() > 0)
//						destination_position_C.setX(RRT::PARU.getX());
//					else
//						destination_position_C.setX(RRT::PALU.getX());
//				}
//				destination_is_in_penalty_area = true;
//			}
//		}
//	}
//
//	graph.add_vertex(current_position_C, -1);
//	graph.add_vertex(destination_position_C, -2);
//
//	current_position_C = graph.vertex[0];
//	destination_position_C = graph.vertex[1];
//
//	destination_position_P = destination_position_C.cartesian_to_polar();
//	destination_position_P_from_CP = destination_position_C.cartesian_to_polar_from_CP(current_position_C);
//
//
//	check_r = current_position_C.getDistanceTo(destination_position_C);
//	if (destination_is_in_balk)
//		check_r += robot_radius;		//robot radius
//	distance_from_current_to_destination = check_r;
//
//	check_angle = M_PI* pow(M_E, (-distance_from_current_to_destination / 5000));
//	distance_of_vertices = 1200 + (-pow((distance_from_current_to_destination - 11000), 2)) / 121000;
//
//	for (j = 0; j < 1; j++)		//1 == numOfGraphs
//	{
//		double max_check_r;
//
//		//check current position is in which airt
//		int current_airt = current_position_C.detect_airt();
//
//		//determine max of check_r by airt of current position
//		if (current_airt == 1)
//		{
//			VecPosition t;
//			t.setX(-FieldLength / 2);
//			t.setY(-FieldWidth / 2);
//			max_check_r = current_position_C.getDistanceTo(t);
//
//		}
//		else if (current_airt == 2)
//		{
//			VecPosition t;
//			t.setX(FieldLength / 2);
//			t.setY(-FieldWidth / 2);
//			max_check_r = current_position_C.getDistanceTo(t);
//		}
//		else if (current_airt == 3)
//		{
//			VecPosition t;
//			t.setX(FieldLength / 2);
//			t.setY(FieldWidth / 2);
//			max_check_r = current_position_C.getDistanceTo(t);
//		}
//		else
//		{
//			VecPosition t;
//			t.setX(-FieldLength / 2);
//			t.setY(FieldWidth / 2);
//			max_check_r = current_position_C.getDistanceTo(t);
//		}
//
//		for (int i = 0, e = 0;  i < 2 * world.num_of_vertices  &&  graph.p < world.num_of_vertices - 1
//			&& graph.q<world.num_of_vertices - 2 && !reach_destination;   i += 2, e++)
//		{
//			VecPosition random_crd_C;		//nearest coordinates
//			VecPosition random_crd_P(POLAR);			//random coordinates
//			VecPosition nearest_vertex;	//nearest coordinates
//
//			std::uniform_real_distribution<double> unif1(0, check_r);
//			random_crd_P.setX(unif1(gen));
//
//			std::uniform_real_distribution<double> unif2(destination_position_P_from_CP.getY() - (check_angle / 2), check_angle + destination_position_P_from_CP.getY() - (check_angle / 2));
//			random_crd_P.setY(unif2(gen));
//
//			random_crd_C = random_crd_P.polar_to_cartesian();
//			random_crd_C.setX(random_crd_C.getX() + current_position_C.getX());
//			random_crd_C.setY(random_crd_C.getY() + current_position_C.getY());
//
//			if (!robot_can_go_outside_of_field && !Graph::is_in_field(random_crd_C))
//			{
//				i -= 2;
//				e--;
//				continue;
//			}
//
//			nearest_vertex = graph.nearest_vertex(random_crd_C);
//
//			new_vertex = Graph::calculate_vertex(nearest_vertex, random_crd_C, distance_of_vertices);
//
//			//check if new_vertex is in the balk
//			for (int i = 0; i < World::num_of_active_circle_balks; i++)
//			{
//				if (pow(new_vertex.getX() - World::circle_balk[i].getCenter().getX(), 2) + pow(new_vertex.getY() - World::circle_balk[i].getCenter().getY(), 2)
//					<= pow(World::circle_balk[i].getRadius(), 2))
//				{
//					flag = true;
//					break;
//				}
//			}
//			if (flag)
//			{
//				flag = false;
//				i -= 2;
//				e--;
//				continue;
//			}
//
//			if (Balk::check_collision_with_(nearest_vertex, new_vertex, robotT_is_balk, robotO_is_balk, penalty_area_is_balk))
//			{
//				i -= 2;
//				e--;
//				continue;
//			}
//
//			graph.add_edge(nearest_vertex, new_vertex);
//			graph.add_vertex(new_vertex, nearest_vertex.number);
//
//			//check if there is any balk between latest added vertex and destination
//			VecPosition latest_vertex = graph.vertex[graph.p - 1];
//			//!Balk::check_collision(latest_vertex, destination_position_C) && !(penalty_area_is_balk && Balk::check_collision_with_penalty_area(latest_vertex, destination_position_C))
//			if (graph.p < world.num_of_vertices - 1 && graph.q < world.num_of_vertices - 1
//				&& !Balk::check_collision_with_(latest_vertex, destination_position_C, robotT_is_balk, robotO_is_balk, penalty_area_is_balk))
//			{
//				reach_destination = true;
//				graph.add_edge(latest_vertex, destination_position_C);
//				graph.vertex[1].parent = latest_vertex.number;
//				i += 2;
//				e++;
//				destination_position_C.parent = latest_vertex.number;
//			}
//		}
//
//		if (!reach_destination)
//		{
//			graph = Graph();
//			graph.add_vertex(current_position_C, -1);
//			graph.add_vertex(destination_position_C, -2);
//			current_position_C = graph.vertex[0];
//			destination_position_C = graph.vertex[1];
//			j--;
//			memset(world.r, NULL, sizeof(world.r));
//			try_time++;
//
//			if (check_r < max_check_r - (check_r*check_r_increase) && try_time >= 0)
//			{
//				////////baraye afzayesh sorat dar entekhab maghsad dar nazdiki posht manee,check_r ra bishtar ziad mikonim.
//				if (!destination_is_in_balk)
//				{
//					//check_r += 500;
//					check_r = check_r + check_r*check_r_increase;
//					//cout << "b" << endl;
//				}
//				else
//				{
//					//check_r += 700;
//					check_r = check_r + check_r*check_r_increase;
//					//cout << "b" << endl;
//				}
//			}
//			else if (try_time >= 0)
//			{
//				is_check_r_max = true;
//				check_r = max_check_r;
//			}
//
//
//
//			if (check_angle <= 2 * M_PI - (check_angle * check_angle_increase) && try_time >= 0)
//			{
//				////////baraye afzayesh sorat vaghti mabda dar nazdiki posht manee bashad,check_angle ra bishtar ziad mikonim. .
//				//check_angle += M_PI / 4;
//				check_angle = check_angle + (check_angle*check_angle_increase);
//				//cout << "a" << endl;
//			}
//			else if (try_time >= 0)
//			{
//				is_check_angle_max = true;
//				check_angle = 2 * M_PI;
//			}
//
//
//			//cout << "t" << endl;
//			continue;
//
//		}
//
//		reach_destination = false;
//		try_time = 0;
//
//		graph.find_pass(current_position_C, destination_position_C, pass, size_of_pass);
//		graph.optimum_pass(pass, optimized_pass, size_of_pass, size_of_optimized_pass, robotT_is_balk, robotO_is_balk, penalty_area_is_balk);
//
//		memset(world.r, NULL, sizeof(world.r));
//
//		for (int i = 0, j = 0; i + 1 < size_of_optimized_pass; i++, j += 2)
//		{
//			world.r[j] = optimized_pass[i];
//			world.r[j + 1] = optimized_pass[i + 1];
//		}
//
//		end_time = std::chrono::high_resolution_clock::now();
//		auto time = end_time - start_time;
//		cout << time.count() / 1000000.0 << endl;
//
//		size_of_OP = size_of_optimized_pass;
//		return optimized_pass;
//	}
//}
//
//
//	
//}
///previous version of MakeRRT_with_draw_of_path
//Cartesian_Coordinates* RRT::MakeRRT_with_draw_of_path(int robot_number, World &world, Cartesian_Coordinates destination_position_C, int &size_of_OP, bool robotT_is_balk, bool robotO_is_balk, bool penalty_area_is_balk, bool robot_can_go_outside_of_field)
//{
//	//add first check of collision
//	//clear i and e in rrt loop
//	//add return to every for that determine position of robot
//
//	///assumptions
//	//we assume that we are allowed to move into penalty area when we want to go to back of penalty area or when we want to go to the field from back of penalty area
//	//we assume that we cant go outside of field when we cant go inside penalty area
//	//we check if destination is in the penalty area
//	//we check if destination is in the balk
//	//we dont check if destination is in the field
//
//	//dakhel zamin = 6000*9000
//	//kole zamin = 7400 * 10400
//	//taghiir auto num_of_vertices
//	//stop karadn auto jostejoo dar soorat peyda nakardan masir
//
//	const int num_of_graphs = 1;		//number of graphs to paint
//	int j = 0;		//number of painted graphs
//	double check_angle;
//	double check_r;
//	bool reach_destination = false;
//	double distance_of_vertices/* = 1000*/;	//700
//	bool flag = true;
//	bool is_check_r_max = false;
//	bool is_check_angle_max = false;
//	double distance_from_current_to_destination;
//	int try_time = 0;
//	bool destination_is_in_balk = false;
//	bool destination_is_in_penalty_area = false;
//	double robot_radius = 150;
//	Cartesian_Coordinates current_position_C;	//in Cartesian
//	Polar_Coordinates current_position_P;		//in Polar
//	Polar_Coordinates destination_position_P;	//in Polar
//	Polar_Coordinates_From_CP destination_position_from_CP_P;	//destination position in polar system that its center is curent position
//	Cartesian_Coordinates new_vertex;
//
//	////****hatman be dakhel for dar keshidan hame graph ha borde shavad.
//	Cartesian_Coordinates pass[World::num_of_vertices - 1];
//	int size_of_pass;
//	Cartesian_Coordinates optimized_pass[World::num_of_vertices - 1];
//	int size_of_optimized_pass;
//	////****
//
//	std::random_device rd;
//	std::mt19937 gen(rd());
//
//	std::chrono::steady_clock::time_point start_time;
//	std::chrono::steady_clock::time_point end_time;
//
//	double check_r_increase = 1.0 / 4.0;
//	double check_angle_increase = 2.0 / 3.0;
//
//
//
//
//
//
//
//	
//	for (int i = 0; i < world.numT; i++)
//	{
//		if (world.robotT[i].id == robot_number)
//		{
//			current_position_C.x = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
//			current_position_C.y = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
//		}
//	}
//
//	if (current_position_C.x == destination_position_C.x && current_position_C.y == destination_position_C.y)
//		return NULL;
//
//
//	if (j < 1)		//1 == numOfGraphs
//	{
//		start_time = std::chrono::high_resolution_clock::now();
//
//		Graph graph[1];
//
//		for (int i = 0; i < world.numT; i++)
//		{
//			if (world.robotT[i].id == robot_number)
//			{
//				current_position_C.x = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
//				current_position_C.y = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
//			}
//		}
//
//		//check if destination is in the balk
//		destination_is_in_balk = false;
//		for (int i = 0; i < World::num_of_active_circle_balks; i++)
//		{
//			if (pow(destination_position_C.x - World::circle_balk[i].getCenter().getX(), 2) + pow(destination_position_C.y - World::circle_balk[i].getCenter().getY(), 2)
//				< pow(World::circle_balk[i].getRadius(), 2))
//			{
//				destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::circle_balk[i]);
//				destination_is_in_balk = true;
//			}
//		}
//
//		//check if destination is in the penalty area
//		if (/*!destination_is_in_balk &&*/ penalty_area_is_balk)
//		{
//			for (int i = 0; i < 4; i++)
//			{
//				if (i == 0 && abs(destination_position_C.x) > FieldLength / 2)
//					break;
//				if (pow(destination_position_C.x - World::penalty_area_circle_balk[i].getCenter().getX(), 2) + pow(destination_position_C.y - World::penalty_area_circle_balk[i].getCenter().getY(), 2)
//					< pow(World::penalty_area_circle_balk[i].getRadius(), 2))
//				{
//					destination_position_C = Graph::find_nearest_position_outside_balk(destination_position_C, World::penalty_area_circle_balk[i]);
//					if (abs(destination_position_C.x) > abs(RRT::PARU.getX()) && (abs(destination_position_C.y) < RRT::PARU.getY() && abs(destination_position_C.y) > RRT::PARD.getY()))		// when destination is between circles and line of penaty area,this statement will correct destination
//					{
//						if (destination_position_C.x > 0)
//							destination_position_C.x = RRT::PARU.getX();
//						else
//							destination_position_C.x = RRT::PALU.getX();
//					}
//					destination_is_in_penalty_area = true;
//				}
//			}
//		}
//
//		graph.add_vertex(current_position_C, -1);
//		graph.add_vertex(destination_position_C, -2);
//
//		current_position_C = graph.vertex[0];
//		destination_position_C = graph.vertex[1];
//
//		destination_position_P = Cartesian_Coordinates::cartesian_to_polar(destination_position_C);
//		destination_position_from_CP_P = Cartesian_Coordinates::cartesian_to_polar_from_CP(destination_position_C, current_position_C);
//
//
//		check_r = Cartesian_Coordinates::distance(current_position_C, destination_position_C);
//		if (destination_is_in_balk)
//			check_r += robot_radius;		//robot radius
//		distance_from_current_to_destination = check_r;
//
//		check_angle = M_PI* pow(M_E, (-distance_from_current_to_destination / 5000));
//		distance_of_vertices = 1200 + (-pow((distance_from_current_to_destination - 11000), 2)) / 121000;
//
//		for (j = 0; j < 1; j++)		//1 == numOfGraphs
//		{
//
//			double max_check_r;
//
//			//check current position is in wich airt
//			int current_airt = Cartesian_Coordinates::detect_airt(current_position_C);
//
//			//determine max of check_r by airt of current position
//			if (current_airt == 1)
//			{
//				Cartesian_Coordinates t;
//				t.x = -FieldLength / 2;
//				t.y = -FieldWidth / 2;
//				max_check_r = Cartesian_Coordinates::distance(current_position_C, t);
//
//			}
//			else if (current_airt == 2)
//			{
//				Cartesian_Coordinates t;
//				t.x = FieldLength / 2;
//				t.y = -FieldWidth / 2;
//				max_check_r = Cartesian_Coordinates::distance(current_position_C, t);
//			}
//			else if (current_airt == 3)
//			{
//				Cartesian_Coordinates t;
//				t.x = FieldLength / 2;
//				t.y = FieldWidth / 2;
//				max_check_r = Cartesian_Coordinates::distance(current_position_C, t);
//			}
//			else
//			{
//				Cartesian_Coordinates t;
//				t.x = -FieldLength / 2;
//				t.y = FieldWidth / 2;
//				max_check_r = Cartesian_Coordinates::distance(current_position_C, t);
//			}
//
//			for (int i = 0, e = 0;  i < 2 * world.num_of_vertices  &&  graph.p < world.num_of_vertices - 1
//				&& graph.q<world.num_of_vertices - 2 && !reach_destination;   i += 2, e++)
//			{
//				Cartesian_Coordinates random_crd_C;		//nearest coordinates
//				Polar_Coordinates random_crd_P;			//random coordinates
//				Cartesian_Coordinates nearest_vertex;	//nearest coordinates
//
//				std::uniform_real_distribution<double> unif1(0, check_r);
//				random_crd_P.r = unif1(gen);
//
//				std::uniform_real_distribution<double> unif2(destination_position_from_CP_P.angle - (check_angle / 2), check_angle + destination_position_from_CP_P.angle - (check_angle / 2));
//				random_crd_P.angle = unif2(gen);
//
//				random_crd_C = Cartesian_Coordinates::polar_to_cartesian(random_crd_P/*, current_position_C*/);
//				random_crd_C.x += current_position_C.x;
//				random_crd_C.y += current_position_C.y;
//
//				if (!robot_can_go_outside_of_field && !Graph::is_in_field(random_crd_C))
//				{
//					i -= 2;
//					e--;
//					continue;
//				}
//
//
//				nearest_vertex = graph.nearest_vertex(random_crd_C);
//
//				new_vertex = Graph::calculate_vertex(nearest_vertex, random_crd_C, distance_of_vertices);
//
//				//check if new_vertex is in the balk
//				for (int i = 0; i < World::num_of_active_circle_balks; i++)
//				{
//					if (pow(new_vertex.x - World::circle_balk[i].getCenter().getX(), 2) + pow(new_vertex.y - World::circle_balk[i].getCenter().getY(), 2)
//						<= pow(World::circle_balk[i].getRadius(), 2))
//					{
//						flag = false;
//						break;
//					}
//				}
//				if (!flag)
//				{
//					flag = true;
//					i -= 2;
//					e--;
//					continue;
//				}
//
//				if (Balk::check_collision_with_(nearest_vertex, new_vertex, robotT_is_balk, robotO_is_balk, penalty_area_is_balk))
//				{
//					i -= 2;
//					e--;
//					continue;
//				}
//
//				graph.add_edge(nearest_vertex, new_vertex);
//				graph.add_vertex(new_vertex, nearest_vertex.number);
//
//				//check if there is any balk between latest added vertex and destination
//				Cartesian_Coordinates latest_vertex = graph.vertex[graph.p - 1];
//				//!Balk::check_collision(latest_vertex, destination_position_C) && !(penalty_area_is_balk && Balk::check_collision_with_penalty_area(latest_vertex, destination_position_C))
//
//
//				if (graph.p < world.num_of_vertices - 1 && graph.q < world.num_of_vertices - 1
//					&& !Balk::check_collision_with_(latest_vertex, destination_position_C, robotT_is_balk, robotO_is_balk, penalty_area_is_balk))
//				{
//						reach_destination = true;
//						graph.add_edge(latest_vertex, destination_position_C);
//						graph.vertex[1].parent = latest_vertex.number;
//						i += 2;
//						e++;
//						destination_position_C.parent = latest_vertex.number;
//				}
//
//				//if (i < (2 * world.num_of_vertices) - 2 && graph.p < world.num_of_vertices - 1 && graph.q < world.num_of_vertices - 1
//				//	&& !Balk::check_collision_with_(latest_vertex, destination_position_C, robotT_is_balk, robotO_is_balk, penalty_area_is_balk))
//				//{
//				//	reach_destination = true;
//				//	graph.add_edge(latest_vertex, destination_position_C);
//				//	i += 2;
//				//	e++;
//				//	//
//				//	world.r[i].setVecPosition(graph.edge[e].start_vertex.x, graph.edge[e].start_vertex.y);
//				//	world.r[i + 1].setVecPosition(graph.edge[e].end_vertex.x, graph.edge[e].end_vertex.y);
//				//	//
//				//	graph.vertex[1].parent = latest_vertex.number;
//				//	destination_position_C.parent = latest_vertex.number;
//				//}
//			}
//
//			if (!reach_destination)
//			{
//				graph = Graph();
//				graph.add_vertex(current_position_C, -1);
//				graph.add_vertex(destination_position_C, -2);
//				current_position_C = graph.vertex[0];
//				destination_position_C = graph.vertex[1];
//				j--;
//				memset(world.r, NULL, sizeof(world.r));
//				try_time++;
//
//				if (check_r < max_check_r - (check_r*check_r_increase) && try_time >= 0)
//				{
//					////////baraye afzayesh sorat dar entekhab maghsad dar nazdiki posht manee,check_r ra bishtar ziad mikonim.
//					if (!destination_is_in_balk)
//					{
//						//check_r += 500;
//						check_r = check_r + check_r*check_r_increase;
//						//cout << "b" << endl;
//					}
//					else
//					{
//						//check_r += 700;
//						check_r = check_r + check_r*check_r_increase;
//						//cout << "b" << endl;
//					}
//				}
//				else if (try_time >= 0)
//				{
//					is_check_r_max = true;
//					check_r = max_check_r;
//				}
//
//
//
//				if (check_angle <= 2 * M_PI - (check_angle * check_angle_increase) && try_time >= 0)
//				{
//					////////baraye afzayesh sorat vaghti mabda dar nazdiki posht manee bashad,check_angle ra bishtar ziad mikonim. .
//					//check_angle += M_PI / 4;
//					check_angle = check_angle + (check_angle*check_angle_increase);
//					//cout << "a" << endl;
//				}
//				else if (try_time >= 0)
//				{
//					is_check_angle_max = true;
//					check_angle = 2 * M_PI;
//				}
//
//
//				//cout << "t" << endl;
//				continue;
//
//			}
//
//			reach_destination = false;
//			try_time = 0;
//
//			graph.find_pass(current_position_C, destination_position_C, pass, size_of_pass);
//			graph.optimum_pass(pass, optimized_pass, size_of_pass, size_of_optimized_pass, robotT_is_balk, robotO_is_balk, penalty_area_is_balk);
//
//			memset(world.r, NULL, sizeof(world.r));
//
//			VecPosition tempvec;
//			for (int i = 0, j = 0; i + 1 < size_of_optimized_pass; i++, j += 2)
//			{
//				tempvec.setX(optimized_pass[i].x);
//				tempvec.setY(optimized_pass[i].y);
//				world.r[j] = tempvec;
//
//				tempvec.setX(optimized_pass[i + 1].x);
//				tempvec.setY(optimized_pass[i + 1].y);
//				world.r[j + 1] = tempvec;
//			}
//
//			end_time = std::chrono::high_resolution_clock::now();
//			auto time = end_time - start_time;
//			cout << time.count() / 1000000.0 << endl;
//			size_of_OP = size_of_optimized_pass;
//			return optimized_pass;
//		}
//	}
//
//
//}
/*! CLASS RRT */