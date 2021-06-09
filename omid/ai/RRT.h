#ifndef RRT_H
#define RRT_H

#include"world.h"
#include <random>
#ifdef _WIN32
   #include <io.h>
   #include <Windows.h>
#elif __linux__
   #include <inttypes.h>
   #include <unistd.h>
   #define __int64 int64_t
   #define _close close
   #define _read read
   #define _lseek64 lseek64
   #define _O_RDONLY O_RDONLY
   #define _open open
   #define _lseeki64 lseek64
   #define _lseek lseek
   #define stricmp strcasecmp
   #include <stdio.h>
   #include <string.h>
   #include <stdlib.h>
   #include <math.h>
   #include <GL/gl.h>
   #include <GL/glu.h>
   //#include <GL/glx.h>
   //#include <X11/Xlib.h>
  // #include "myheader.h"    // can no longer use windows.h or conio.h
  // #include "myheader2.h"
#endif#include <fcntl.h>#include "geometry.h"
#include "Switches.h"


//class Polar_Coordinates
//{
//public:
//	double angle;
//	double r;
//};

//class Cartesian_Coordinates
//{
//public:
//	double x;
//	double y;
//	int parent;
//	int number;
//
//	static double distance(Cartesian_Coordinates crd1, Cartesian_Coordinates crd2);
//	static short int detect_airt(Cartesian_Coordinates crd);
//	/*static Polar_Coordinates cartesian_to_polar(Cartesian_Coordinates c);*/
//	static Cartesian_Coordinates polar_to_cartesian(Polar_Coordinates p/*,Cartesian_Coordinates current_position*/);
//	static Polar_Coordinates_From_CP cartesian_to_polar_from_CP(Cartesian_Coordinates c,Cartesian_Coordinates cp);
//	static double deg_to_rad(double deg);
//	static double rad_to_deg(double rad);
//};

class Edge
{
public:
	VecPosition start_vertex;
	VecPosition end_vertex;
};

class Graph
{
public:
	Graph();
	int p;				//number of live vertices
	int q;				//number of live edges
	VecPosition vertex[World::num_of_vertices];
	Edge edge[World::num_of_vertices - 1];

	VecPosition nearest_vertex(const VecPosition &crd);
	void add_vertex(const VecPosition &crd,const int &parrent);
	void add_edge(const VecPosition &start_crd,const VecPosition &end_crd);
	void remove_vertex(int number);
	//VecPosition calculate_vertex(const VecPosition &start,const VecPosition &destination, const double &distance);
	void find_pass(const VecPosition &start, const VecPosition &end, VecPosition* pass , int &size_of_pass);
	void optimize_pass(const VecPosition* const pass, VecPosition* const optimized_pass, const int &size_of_pass, int &size_of_optimized_pass, const short int &robot_index, const bool &robotT_is_balk, const bool &robotO_is_balk, const bool &penalty_area_is_balk,const Ball::BalkMode &ballBalkMode);
	//Cartesian_Coordinates nearest_circle_balk(Cartesian_Coordinates crd);
	static bool is_in_field(const VecPosition &point);
	//static VecPosition find_nearest_position_outside_balk(VecPosition &destination_position, const Circle &balk);
	//static bool is_in_field_with_outside(VecPosition point);
};

class Bar
{
public:
	VecPosition start_point;
	VecPosition end_point;
	/*static double line_slope(VecPosition start, VecPosition end);*/
	static double perp_slope(double slope);
	
};

class Balk
{
public:
	static void set_balks_in_world_object();


	static bool check_collision_with_robotT(VecPosition start, VecPosition end, const short int &robot_number);
	static bool check_collision_with_robotO(VecPosition start, VecPosition end);
	static bool check_collision_with_penalty_area(VecPosition start, VecPosition end);
	static bool check_collision_with_penalty_area(Paraline pl);
	static bool check_collision_with_ball_area(VecPosition position, VecPosition end, Ball::BalkMode ballBalkMode);
	static bool check_collision_with_additional_cirlce_balks(VecPosition start, VecPosition end);
	static bool check_collision_with_additional_paraline_balks(VecPosition start, VecPosition end);	///doesnt have declaration
	static bool check_collision_with_(const VecPosition &start, const VecPosition &end, const short int &index_of_robot, const bool &robotT_is_balk, const bool &robotO_is_balk, const bool &penalty_area_is_balk, const Ball::BalkMode &ballBalkMode);


	static VecPosition getNearestPositionOutsideOfPenaltyArea(VecPosition position); ///doesnt have declaration
	static bool		   setOutsideOfPenaltyAreaIfIsInside(VecPosition position, short int robot_index); ///doesnt have declaration

	//static VecPosition getPositionOutsideOf_(VecPosition position, const bool &isRobotTBalk, const bool &isRobotOBalk, const bool &isPenaltyAreaBalk, /*const bool &cantGoOutside,*/ bool &destination_is_in_balk, bool &destination_is_in_penalty_area, bool &destination_is_outside_of_surround_field, const int &except_index = -1) ;
	static bool		   setPositionOutsideOf_(VecPosition &position, const bool &isRobotTBalk, const bool &isRobotOBalk, const bool &isPenaltyAreaBalk, Ball::BalkMode ballBalkMode, /*const bool &cantGoOutside,*/ bool &destination_is_in_balk, bool &destination_is_in_penalty_area, bool &destination_is_outside_of_surround_field, const int &except_index = -1);
	//static VecPosition getPositionOutsideOf_(VecPosition position, const bool &isRobotTBalk, const bool &isRobotOBalk, const bool &isPenaltyAreaBalk, /*const bool &cantGoOutside,*/ const int &except_index = -1) ;
	//static bool		 setPositionOutsideOf_(VecPosition &position, const bool &isRobotTBalk, const bool &isRobotOBalk, const bool &isPenaltyAreaBalk, /*const bool &cantGoOutside,*/ const int &except_index = -1);

	static bool		   isInPenaltyArea(const VecPosition &position) ;
	static bool		   isInRobotT(const VecPosition &position) ;
	static bool	       isInRobotTExcept(const VecPosition &position, const short int &robot_index) ;
	static bool	       isInRobotO(const VecPosition &position) ;
	static bool		   isInBallArea(const VecPosition &position, Ball::BalkMode bbm);
	static bool	       isIn_(const VecPosition &position, const bool &robotT_is_Balk, const bool &robotO_is_Balk, const bool &penaltyArea_is_Balk, const short int &except_index = -1) ;
	
	//short int detect_airt() const;
	//static bool is_in_circle(Balk balk[World::num_of_allowed_circle_balks],double m,double x0,double y0);
	//static bool check_collision(Cartesian_Coordinates start, Cartesian_Coordinates end, Circle circle_balk,Bar_Segment line_balk);
	//static bool check_collision(Cartesian_Coordinates start, Cartesian_Coordinates end);
	//static bool set_robotT_circle_balk(int num, VecPosition center, double radius, World &world);
	//static bool set_robotO_circle_balk(int num, VecPosition center, double radius, World &world);
	//static bool set_penalty_area_circle_balk(int num, double x, double y, double radius);
	//static bool set_penalty_area_line_balk(int num, VecPosition start, VecPosition end);
	//static bool set_defualt_penalty_area();
	//static void set_balk(World world);
	//static bool set_circle_balk(int num,double x,double y, double radius);
	//static bool set_line_balk(int num, VecPosition start, VecPosition end);

};

class RRT
{
public:

	bool flag = false;
	bool is_check_r_max = false;
	bool is_check_angle_max = false;
	bool reach_destination = false;
	bool destination_is_in_balk = false;
	bool destination_is_in_penalty_area = false;
	bool destination_is_in_surround_field = false;
	bool unauthorized_robot_is_in_penalty_area = false;

	const int num_of_graphs = 1;		//number of graphs to paint
	
	int try_time = 1;
	int j = 0;		//number of painted graphs
	int size_of_optimized_pass;
	int size_of_pass;
	int robot_index;
	int max_try_time = 7;	//7 is good
	int max_num_of_countinus_fails = 300;	//50
	int num_of_countinus_fails_to_find_random_crd = 0;

	double check_angle;
	double check_r;
	double distance_of_vertices/* = 1000*/;	//700
	double distance_from_current_to_destination;
	double robot_radius = 150;
	double check_r_increase = 1.0 / 4.0;	// 1.0/4.0
	double max_check_r;
	double check_angle_increase = 2.0/3.0;	// 2.0/3.0
	double max_check_angle_increase = M_PI/3;	// 2.0/3.0
	double min_check_angle_increase = M_PI/6;
	double destination_estimation_circle_increase_angle = 0;
	bool destination_estimation_circle_is_found;
	VecPosition current_position_C;	//in Cartesian
	VecPosition current_position_P{ POLAR };		//in Polar
	VecPosition destination_position_P{ POLAR };	//in Polar
	VecPosition destination_position_P_from_CP{ POLAR };	//destination position in polar system that its center is curent position
	VecPosition new_vertex;
	VecPosition pass[World::num_of_vertices - 1];
	
	VecPosition random_crd_C;		//nearest coordinates
	VecPosition random_crd_P{ POLAR };			//random coordinates
	VecPosition nearest_vertex;	//nearest coordinates

	Circle destination_estimation_circle;
	double destination_estimation_circle_radius_increase_rate = ROBOT_RADIUS; // 2 * ROBOT_RADIUS;
	double max_of_destination_estimation_circle_radius = 6 * ROBOT_RADIUS;
	Graph graph;

	enum RRT_result { path_found, path_not_found };

	std::random_device rd;
	std::mt19937 gen{ rd() };

	std::chrono::steady_clock::time_point start_time;
	std::chrono::steady_clock::time_point end_time;

	RRT_result MakeRRT(const int &robot_number, VecPosition destination_position_C, VecPosition* const optimized_pass, int &size_of_OP, const bool &robotT_is_balk = true, const bool &robotO_is_balk = true, const bool &penalty_area_is_balk = true, const Ball::BalkMode &ballBalkMode = Ball::BalkMode::notBalk /*, const bool &robot_cant_go_outside_of_field = true*/);
	//VecPosition* MakeRRT_withDrawOfPath(int robot_number, VecPosition destination, int &size_of_OP, World &world, bool robotT_is_balk = true, bool robotO_is_balk = true, bool penalty_area_is_balk = true, bool robot_can_go_outside_of_field = true);

private:
	double calculateMaxOfCheck_r(const VecPosition &current_position);	///determine max of check_r by airt of current position
	
};

extern RRT rrt;

#endif // !RRT_H

 