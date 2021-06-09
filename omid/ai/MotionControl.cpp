#include "MotionControl.h"
#include "geometry.h"
#include "world.h"
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
  //include <GL/glx.h>
   //#include <X11/Xlib.h>
   #include <unistd.h>
  // #include "myheader.h"    // can no longer use windows.h or conio.h
  // #include "myheader2.h"
#endif
#include "nrf.h"
#include "Switches.h"
/*! PD METHODS*/

///old GoPd
//void PD::GoPd(int robot_number, char robot_id, World &world, Cartesian_Coordinates destination)
//{
//	VecPosition v;
//	double u_x;				//baraye voroodi sorat_x robat
//	double u_y;				//baraye voroodi sorat_y robat
//	double kp_x;			//zaribe  p baraye x robat
//	double kp_y;			//zaribe  p baraye y robat
//	double derivative_x;	//moshtagh error baraye x
//	double derivative_y;	//moshtagh error baraye y
//	double kd_x;			//zaribe  d baraye x robat
//	double kd_y;			//zaribe  d baraye y robat
//	double p_error_x;		//error ghabli baraye x robat
//	double p_error_y;		//error ghabli baraye y robat
//	double error_x;			//error baraye x robat
//	double error_y;			//error baraye x robat
//	double dt;				//baze zamani baraye barresi robat
//	double xc;				//x feeli robat
//	double yc;				//y feeli robat
//	double xd;				//x maghsad robat
//	double yd;				//y maghsad robat
//	//double working_position_x;	//taein mokhtasat delkhah rooye zamin
//	//double working_position_y;	//taein mokhtasat delkhah rooye zamin
//	double vsize;
//	double count = 0;
//	bool close_x = false;
//	bool close_y = false;
//	bool close_r = false;
//
//	double rotation;		//meghdare mored niaz charkhesh
//	double kp_r;
//	double kd_r;
//	double error_r;
//	double derivative_r;
//	double p_error_r;
//	double u_r;
//
//	kp_x = 0.6;
//	kd_x = 0.05;
//
//	kp_y = 0.6;
//	kd_y = 0.05;
//
//	dt = 0.016;
//
//	kp_r = 0.015;
//	kd_r = 0.00100;
//
//	rotation = 0;
//	//rotation = M_PI / 10000000.000000000;
//	//rotation =2* (M_PI/3 );
//	//rotation = (M_PI);
//
//	//while(true)
//	//{
//	//sleep(10000);
//
//	for (int i = 0; i < world.numT; i++)
//	{
//		if (world.robotT[i].id == robot_number)
//		{
//			xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
//			yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
//		}
//	}
//
//	xd = destination.x;		//mokhtasati ke ba mouse click kardim.
//	yd = destination.y;		//mokhtasati ke ba mouse click kardim.
//
//	p_error_x = xd - xc;
//	p_error_y = yd - yc;
//	p_error_r = rotation - world.robotT[0].angle;
//
//	if (p_error_x < 800 && p_error_x > -800)
//		close_x = true;
//	else
//		close_x = false;
//
//	if (p_error_y < 800 && p_error_y > -800)
//		close_y = true;
//	else
//		close_y = false;
//
//	if (p_error_r < 0.5 && p_error_r > -0.5)
//		close_r = true;
//	else
//		close_r = false;
//
//
//	//while (abs(world.robotT[0].angle - rotation) > 0.08)
//	//{
//	//	auto start_time_r = std::chrono::high_resolution_clock::now();
//	//	//v.setX(0);		//taein x bordar sorat
//	//	//v.setY(0);		//taein y bordar sorat
//	//	error_r = rotation - world.robotT[0].angle;
//	//	if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) && close_r)
//	//	{
//	//		kp_r = 0.05;
//	//	}
//	//	else if (error_r < 0.3 && error_r > -0.3 && close_r)
//	//	{
//	//		kp_r = 0.03;
//	//	}
//	//	else
//	//	{
//	//		kp_r = 0.015;
//	//	}
//	//	if (abs(rotation - (world.robotT[0].angle)) > M_PI)
//	//	{
//	//		if (rotation  < 0)
//	//		{
//	//			error_r = 2 * M_PI - abs(error_r);
//	//		}
//	//		else
//	//		{
//	//			error_r = error_r - 2 * M_PI;
//	//		}
//	//	}
//	//	derivative_r = (error_r - p_error_r) / dt;
//	//	u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//	//	p_error_r = error_r;
//	//	/*if (u_r < 0.00555555 && u_r > -0.00555555)
//	//	{
//	//	if (u_r > 0)
//	//	{
//	//	u_r = 0.006;
//	//	}
//	//	if (u_r < 0)
//	//	{
//	//	u_r = -0.006;
//	//	}
//	//	}*/
//	//	/*
//	//	if (u_r > 0.02)
//	//	{
//	//	u_r = 0.02;
//	//	}*/
//	//	nrf::go(v, id, world, u_r);//dastoor harekat robat
//	//	auto end_time_r = std::chrono::high_resolution_clock::now();
//	//	auto time_r = end_time_r - start_time_r;
//	//	if (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() < 16000)
//	//	{
//	//		sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() / 1000));
//	//	}
//	//	if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//	//		break;
//	//}
//
//	while (abs(xc - xd) > 60 || abs(yc - yd) > 60)
//	{
//
//		auto start_time = std::chrono::high_resolution_clock::now();
//
//		for (int i = 0; i < world.numT; i++)
//		{
//			if (world.robotT[i].id == robot_number)
//			{
//				xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
//				yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
//			}
//		}
//
//		error_x = xd - xc;
//
//		if (((error_x < 800 && error_x > 500) || (error_x > -800 && error_x < -500)) && close_x)
//		{
//			kp_x = 0.7;
//			kp_y = 0.7;
//		}
//		else if (((error_x < 500 && error_x > 100) || (error_x > -500 && error_x < -100)) && close_x)
//		{
//			kp_x = 1.3;
//			kp_y = 1.3;
//		}
//		else if (error_x < 100 && error_x > -100 && close_x)
//		{
//			kp_x = 3;
//			kp_y = 3;
//		}
//		else
//		{
//			kp_x = 0.6;
//			kp_y = 0.6;
//		}
//
//		derivative_x = (error_x - p_error_x) / dt;
//		u_x = error_x * kp_x + kd_x * derivative_x;		//*mohasebe voroodi badi*
//		p_error_x = error_x;
//
//
//
//		error_y = yd - yc;
//
//		if (((error_y < 800 && error_y > 500) || (error_y > -800 && error_y < -500)) && close_y)
//		{
//			kp_y = 0.7;
//			kp_x = 0.7;
//		}
//		else if (((error_y < 500 && error_y > 100) || (error_y > -500 && error_y < -100)) && close_y)
//		{
//			kp_y = 1.3;
//			kp_x = 1.3;
//		}
//		else if (error_y < 100 && error_y > -100 && close_y)
//		{
//			kp_y = 3;
//			kp_x = 3;
//		}
//		else
//		{
//			kp_y = 0.6;
//			kp_x = 0.6;
//		}
//
//		derivative_y = (error_y - p_error_y) / dt;
//		u_y = error_y * kp_y + kd_y * derivative_y;		//*mohasebe voroodi badi*
//		p_error_y = error_y;
//
//		/*vsize = sqrt(pow(u_x, 2) + pow(u_y, 2));
//		u_x = u_x / vsize;
//		u_y = u_y / vsize;*/
//
//		v.setX(u_x);		//taein x bordar sorat
//		v.setY(u_y);		//taein y bordar sorat
//
//
//		//PD baraye rotation robat dar hein harekat va dar nazdiki maghsad
//
//		//if ((abs(xc - xd) < 300 && abs(yc - yd) < 300) && abs(world.robotT[0].angle - rotation) > 0.08)
//		//{
//		//	error_r = rotation - world.robotT[0].angle;
//		//	if (abs(rotation - (world.robotT[0].angle)) > M_PI)
//		//	{
//		//		if (rotation < 0)
//		//		{
//		//			error_r = 2 * M_PI - abs(error_r);
//		//		}
//		//		else
//		//		{
//		//			error_r = error_r - 2 * M_PI;
//		//		}
//		//	}
//		//	derivative_r = (error_r - p_error_r) / dt;
//		//	u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//		//	p_error_r = error_r;
//		//	/////*if (u_r < 0.00555555 && u_r > -0.00555555)
//		//	////{
//		//	////if (u_r > 0)
//		//	////{
//		//	////u_r = 0.006;
//		//	////}
//		//	////if (u_r < 0)
//		//	////{
//		//	////u_r = -0.006;
//		//	////}
//		//	////}
//		//	////if (u_r > 0.02)
//		//	////{
//		//	////u_r = 0.02;
//		//	////}*/
//		//	nrf::go(v, id, world, u_r);//dastoor harekat robat
//		//}
//		//else
//
//		nrf::go(v,robot_number, robot_id, world, 0);//dastoor harekat robat
//
//		auto end_time = std::chrono::high_resolution_clock::now();
//		auto time = end_time - start_time;
//
//		if (std::chrono::duration_cast<std::chrono::microseconds>(time).count() < 16000)
//		{
//			sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000));
//		}
//
//
//		//agar hadaf ghabl az residan robat be hadaf taghir kard,halghe ro dobare az avval shoroo kon.
//		///if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//		///	break;
//
//		//PD baraye rotation robat dar maghsad
//
//		//while (abs(xc - xd) < 50 && abs(yc - yd) < 50 && abs(world.robotT[0].angle - rotation) > 0.1)
//		//{
//		//	auto start_time_r = std::chrono::high_resolution_clock::now();
//		//	v.setX(0);		//taein x bordar sorat
//		//	v.setY(0);		//taein y bordar sorat
//		//	error_r = rotation - world.robotT[0].angle;
//		//	if (((error_r < 800 && error_r > 500) || (error_r > -800 && error_r < -500)) && close_r)
//		//	{
//		//		kp_r = 2.5;
//		//	}
//		//	else if (((error_r < 500 && error_r > 100) || (error_r > -500 && error_r < -100)) && close_r)
//		//	{
//		//		kp_r = 3;
//		//	}
//		//	else if (error_r < 100 && error_r > -100 && close_r)
//		//	{
//		//		kp_r = 3.5;
//		//	}
//		//	else
//		//	{
//		//		kp_r = 0.6;
//		//	}
//		//	if (abs(rotation - (world.robotT[0].angle)) > M_PI)
//		//	{
//		//		if (rotation  < 0)
//		//		{
//		//			error_r = 2 * M_PI - abs(error_r);
//		//		}
//		//		else
//		//		{
//		//			error_r = error_r - 2 * M_PI;
//		//		}
//		//	}
//		//	derivative_r = (error_r - p_error_r) / dt;
//		//	u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//		//	p_error_r = error_r;
//		//	/////*if (u_r < 0.00555555 && u_r > -0.00555555)
//		//	////{
//		//	////if (u_r > 0)
//		//	////{
//		//	////u_r = 0.006;
//		//	////}
//		//	////if (u_r < 0)
//		//	////{
//		//	////u_r = -0.006;
//		//	////}
//		//	////}
//		//	////if (u_r > 0.02)
//		//	////{
//		//	////u_r = 0.02;
//		//	////}*/
//		//	nrf::go(v, id, world, u_r);//dastoor harekat robat
//		//	auto end_time_r = std::chrono::high_resolution_clock::now();
//		//	auto time_r = end_time_r - start_time_r;
//		//	if (std::chrono::duration_cast<std::chrono::microseconds>(time).count() < 16000)
//		//	{
//		//		sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000));
//		//	}
//		///	///if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//		///	///	break;
//		//}
//
//
//		//while (abs(xc - xd) < 50 && abs(yc - yd) < 50 && abs(world.robotT[0].angle - rotation) > 0.08)
//		//{
//		//	auto start_time_r = std::chrono::high_resolution_clock::now();
//		//	v.setX(0);		//taein x bordar sorat
//		//	v.setY(0);		//taein y bordar sorat
//		//	error_r = rotation - world.robotT[0].angle;
//		//	if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) && close_r)
//		//	{
//		//		kp_r = 0.05;
//		//	}
//		//	else if (error_r < 0.3 && error_r > -0.3 && close_r)
//		//	{
//		//		kp_r = 0.03;
//		//	}
//		//	else
//		//	{
//		//		kp_r = 0.015;
//		//	}
//		//	if (abs(rotation - (world.robotT[0].angle)) > M_PI)
//		//	{
//		//		if (rotation  < 0)
//		//		{
//		//			error_r = 2 * M_PI - abs(error_r);
//		//		}
//		//		else
//		//		{
//		//			error_r = error_r - 2 * M_PI;
//		//		}
//		//	}
//		//	derivative_r = (error_r - p_error_r) / dt;
//		//	u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//		//	p_error_r = error_r;
//		//	/*if (u_r < 0.00555555 && u_r > -0.00555555)
//		//	{
//		//	if (u_r > 0)
//		//	{
//		//	u_r = 0.006;
//		//	}
//		//	if (u_r < 0)
//		//	{
//		//	u_r = -0.006;
//		//	}
//		//	}*/
//		//	/*
//		//	if (u_r > 0.02)
//		//	{
//		//	u_r = 0.02;
//		//	}*/
//		//	nrf::go(v, id, world, u_r);//dastoor harekat robat
//		//	auto end_time_r = std::chrono::high_resolution_clock::now();
//		//	auto time_r = end_time_r - start_time_r;
//		//	if (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() < 16000)
//		//	{
//		//		sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() / 1000));
//		//	}
//		///	if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//		///		break;
//		//}
//	}
//
//	v.setX(0);		//taein x bordar sorat
//	v.setY(0);		//taein y bordar sorat
//
//	nrf::go(v, robot_number, robot_id, world, 0);
//
//
//
//}
///drawing chart need to be corrected
void PD::GoPD(int robot_number, char robot_id, VecPosition destination, bool rotate, double rotation, World &world)
{
	///k_p changed
	///min_u_x va min_u_y avaz shode!!!!!!!!!!!!!!!!!!!!!!!!!!!

	//auto s_time = std::chrono::high_resolution_clock::now();
	//sleep(2000);



	VecPosition v;	        //baraye taein bordar sorat

	double u_x;				//baraye voroodi sorat_x robat
	double u_y;				//baraye voroodi sorat_y robat
	double kp_x;			//zaribe  p baraye x robat
	double kp_y;			//zaribe  p baraye y robat
	double derivative_x;	//moshtagh error baraye x
	double derivative_y;	//moshtagh error baraye y
	double kd_x;			//zaribe  d baraye x robat
	double kd_y;			//zaribe  d baraye y robat
	double p_error_x;		//error ghabli baraye x robat
	double p_error_y;		//error ghabli baraye y robat
	double error_x;			//error baraye x robat
	double error_y;			//error baraye x robat
	double dt;				//baze zamani baraye barresi robat
							//double xc;				//x feeli robat
							//double yc;				//y feeli robat
	VecPosition current_position;
	double xd;				//x maghsad robat
	double yd;				//y maghsad robat
	double xc;
	double yc;
	//double working_position_x;	//taein mokhtasat delkhah rooye zamin
	//double working_position_y;	//taein mokhtasat delkhah rooye zamin
	bool close_x = false;
	bool close_y = false;
	bool close_r = false;
	const double min_u_x = 100; //37.3
	const double min_u_y = 100; //37.3

	double rd;
	double kp_r;
	double kd_r;
	double error_r;
	double derivative_r;
	double p_error_r;
	double u_r;
	double rc;
	const double min_u_r = 0.006;
	const double max_u_r = 0.02;
	double vision_time = 0;
	bool time = true;

	kp_x = 0.6;		//0.6
	kd_x = 0.05;

	kp_y = 0.6;		//0.6
	kd_y = 0.05;

	dt = 0.016;

	kp_r = 0.015;
	kd_r = 0.00100;

	rd = rotation;


	for (int i = 0; i < world.numT; i++)
	{
		if (world.robotT[i].id == robot_number)
		{
			xc = world.robotT[i].position.getX();
			yc = world.robotT[i].position.getY();
			rc = world.robotT[i].angle;
		}
	}

	xd = destination.getX();
	yd = destination.getY();


	p_error_x = xd - xc;
	p_error_y = yd - yc;
	error_r = PD::cal_error_r(rc, rd);
	p_error_r = error_r;

	if (p_error_x < 800 && p_error_x > -800)
		close_x = true;
	else
		close_x = false;

	if (p_error_y < 800 && p_error_y > -800)
		close_y = true;
	else
		close_y = false;


	///rotate && ....  isnt tested!!
#if DRAW_CHART == 1
	int chart_loop = 0;
#endif

	while (abs(xc - xd) > 60 || abs(yc - yd) > 60 || (rotate && (abs(rd - rc) > 0.1)))
	{

		auto start_time = std::chrono::high_resolution_clock::now();

#if DRAW_CHART == 1
		chart_loop++;
		PD::shift_AI_speeds();
		//PD::shift_ROBOT_speeds();
#endif

		for (int i = 0; i < world.numT; i++)
		{
			if (world.robotT[i].id == robot_number)
			{
				xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
				yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
			}
		}
		for (int i = 0; i < world.numT; i++)
		{
			if (world.robotT[i].id == robot_number)
			{
				rc = world.robotT[i].angle;
			}
		}
		error_x = xd - xc;

		if (((error_x < 800 && error_x > 500) || (error_x > -800 && error_x < -500)) && close_x)
		{
			kp_x = 0.7;
			kp_y = 0.7;
		}
		else if (((error_x < 500 && error_x > 100) || (error_x > -500 && error_x < -100)) && close_x)
		{
			kp_x = 1.3;
			kp_y = 1.3;
		}
		else if (error_x < 100 && error_x > -100 && close_x)
		{
			kp_x = 3;
			kp_y = 3;
		}
		else
		{
			kp_x = 0.8;
			kp_y = 0.8;
		}

		derivative_x = (error_x - p_error_x) / dt;
		u_x = error_x * kp_x + kd_x * derivative_x;		//*mohasebe voroodi badi*
		p_error_x = error_x;



		error_y = yd - yc;

		if (((error_y < 800 && error_y > 500) || (error_y > -800 && error_y < -500)) && close_y)
		{
			kp_y = 0.7;
			kp_x = 0.7;
		}
		else if (((error_y < 500 && error_y > 100) || (error_y > -500 && error_y < -100)) && close_y)
		{
			kp_y = 1.3;
			kp_x = 1.3;
		}
		else if (error_y < 100 && error_y > -100 && close_y)
		{
			kp_y = 3;
			kp_x = 3;
		}
		else
		{
			kp_y = 0.8;
			kp_x = 0.8;
		}

		derivative_y = (error_y - p_error_y) / dt;
		u_y = error_y * kp_y + kd_y * derivative_y;		//*mohasebe voroodi badi*

		p_error_y = error_y;

#if DRAW_CHART == 1
		if (chart_loop == 1)
		{
			chart_loop = 0;
			World::AI_speeds[0][0] = 3 * (u_x / 2000.000);
			World::AI_speeds[0][1] = 3 * (u_y / 2000.000);

			/*if (World::ROBOT_speeds[World::number_of_speeds - 1][0] != 0 && World::ROBOT_speeds[World::number_of_speeds - 1][1] != 0)
			{
			auto e_time = std::chrono::high_resolution_clock::now();
			e_time = s_time - e_time;
			}*/
			/*World::ROBOT_speeds[0][0] = (3 * (World::robot_speed.getX())) / 700.000;
			World::ROBOT_speeds[0][1] = (3 * (World::robot_speed.getY())) / 700.000;*/
		}
		//World::ROBOT_speeds[0][0] = 3 * ((150*((((int)world.robotT[0].velocity.getX())/100) / 150))) / 7.000;
		//World::ROBOT_speeds[0][1] = 3 * ((150*((((int)world.robotT[0].velocity.getY())/100) / 150))) / 7.000;

#endif

		//set min u_x
		if (u_x < min_u_x && u_x > -min_u_x)
		{
			if (u_x > 0)
			{
				u_x = min_u_x;
			}

			if (u_x < 0)
			{
				u_x = -min_u_x;
			}
		}

		//set min u_x
		if (u_y < min_u_y && u_y > -min_u_y)
		{
			if (u_y > 0)
			{
				u_y = min_u_y;
			}

			if (u_y < 0)
			{
				u_y = -min_u_y;
			}
		}



		v.setX(u_x);		//taein x bordar sorat
		v.setY(u_y);		//taein y bordar sorat

							///PD baraye rotation robat dar hein harekat va dar nazdiki maghsad
		if (rotate)
		{
			for (int i = 0; i < world.numT; i++)
			{
				if (world.robotT[i].id == robot_number)
				{
					rc = world.robotT[i].angle;
				}
			}
			if ((abs(xc - xd) < 300 && abs(yc - yd) < 300) && (rd - rc) > 0.08)
			{
				error_r = PD::cal_error_r(rc, rd);
				if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) /*&& close_r*/)
				{
					kp_r = 0.03;		//0.06
				}
				else if (error_r < 0.3 && error_r > -0.3 /*&& close_r*/)
				{
					kp_r = 0.02;
				}
				else
				{
					kp_r = 0.015;
				}

				derivative_r = (error_r - p_error_r) / dt;
				u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
				p_error_r = error_r;

				//set min_r
				if (u_r < min_u_r && u_r > -min_u_r)
				{
					if (u_r > 0)
					{
						u_r = min_u_r;
					}

					if (u_r < 0)
					{
						u_r = -min_u_r;
					}
				}

				//set max u_r
				if (u_r > max_u_r)
				{
					u_r = max_u_r;
				}

				nrf::go(v, robot_number, robot_id, world, 0);
			}
			else
				nrf::go(v, robot_number, robot_id, world, 0);
		}
		else
			nrf::go(v, robot_number, robot_id, world, 0);





		//agar hadaf ghabl az residan robat be hadaf taghir kard,halghe ro dobare az avval shoroo kon.
		/*if (working_position_x != world.mouseX || working_position_x != world.mouseX)
		break;*/

		///PD baraye rotation robat dar maghsad
		if (rotate)
		{
			if (abs(xc - xd) <= 60 && abs(yc - yd) <= 60)
			{
				v.setX(0);
				v.setY(0);

				for (int i = 0; i < world.numT; i++)
				{
					if (world.robotT[i].id == robot_number)
					{
						rc = world.robotT[i].angle;
					}
				}

				error_r = PD::cal_error_r(rc, rd);

				while (abs(error_r) > 0.1)
				{
					auto start_time_r = std::chrono::high_resolution_clock::now();

					for (int i = 0; i < world.numT; i++)
					{
						if (world.robotT[i].id == robot_number)
						{
							rc = world.robotT[i].angle;
						}
					}

					error_r = PD::cal_error_r(rc, rd);
					p_error_r = error_r;

					if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) /*&& close_r*/)
					{
						kp_r = 0.03;		//0.06
					}
					else if (error_r < 0.3 && error_r > -0.3 /*&& close_r*/)
					{
						kp_r = 0.02;
					}
					else
					{
						kp_r = 0.015;
					}

					derivative_r = (error_r - p_error_r) / dt;
					u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
					p_error_r = error_r;

					//set min u_r
					if (u_r < min_u_r && u_r > -min_u_r)
					{
						if (u_r > 0)
						{
							u_r = min_u_r;
						}

						if (u_r < 0)
						{
							u_r = -min_u_r;
						}
					}

					//set max u_r
					if (u_r > max_u_r)
					{
						u_r = max_u_r;
					}

					nrf::go(v, robot_number, robot_id, world, u_r);

					auto end_time_r = std::chrono::high_resolution_clock::now();
					auto time_r = end_time_r - start_time_r;
					if (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() < 16000)
					{
						sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() / 1000));
					}
					/*if (working_position_x != world.mouseX || working_position_x != world.mouseX)
					break;*/

					for (int i = 0; i < world.numT; i++)
					{
						if (world.robotT[i].id == robot_number)
						{
							rc = world.robotT[i].angle;
						}
					}

					error_r = PD::cal_error_r(rc, rd);
				}

			}
		}

		auto end_time = std::chrono::high_resolution_clock::now();
		auto time = end_time - start_time;

		if (std::chrono::duration_cast<std::chrono::microseconds>(time).count() < 16000)
		{
			sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000));
		}
	}

	v.setX(0);		//taein x bordar sorat
	v.setY(0);		//taein y bordar sorat

	nrf::go(v, robot_number, robot_id, world, 0);
	//std::cout << "reached!" << endl;
}
///////needs correction:
//previous errors
//returns
//rotations
//...
VecPosition PD::GenVecVelPD(int robot_number, VecPosition destination, bool rotate, double rotation, World &world)
{

	//auto s_time = std::chrono::high_resolution_clock::now();
	//min_u_x va min_u_y avaz shode!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//sleep(2000);
	//k_p changed
	//correct thrown of exception when vision in off

	VecPosition v;	        //baraye taein bordar sorat

	double u_x;				//baraye voroodi sorat_x robat
	double u_y;				//baraye voroodi sorat_y robat
	double u_r;
	double kp_x;			//zaribe  p baraye x robat
	double kp_y;			//zaribe  p baraye y robat
	double kp_r;
	double kd_x;			//zaribe  d baraye x robat
	double kd_y;			//zaribe  d baraye y robat
	double kd_r;
	double ki_x;
	double ki_y;
	double derivative_x;	//moshtagh error baraye x
	double derivative_y;	//moshtagh error baraye y
	static double integral_x = 0;
	static double integral_y = 0;
	double error_x;			//error baraye x robat
	double error_y;			//error baraye x robat
	static double p_error_x = 0;		//error ghabli baraye x robat
	static double p_error_y = 0;		//error ghabli baraye y robat
	double dt;				//baze zamani baraye barresi robat

	double error_r;
	double derivative_r;
	static double p_error_r = 0;
	double size_of_error;

	VecPosition current_position;
	static VecPosition working_destination;

	double xd;				//x maghsad robat
	double yd;				//y maghsad robat
	double rd;		//meghdare mored niaz charkhesh
	double xc;
	double yc;
	double rc;

	bool close_x = false;
	bool close_y = false;
	bool close_r = false;

	const double min_u_x = 37.3; //37.3
	const double min_u_y = 37.3; //37.3
	const double min_u_r = 0.006;
	const double max_u_r = 0.02;
	//bool time = true;

	//kp_x = 0.6;		//0.6
	//kp_y = 0.6;		//0.6
	//kp_r = 0.015;

	kd_x = 0.01;  //0.1
	kd_y = 0.01;  //0.1
	kd_r = 0.00100;

	ki_x = 0.1;
	ki_y = 0.1;


	dt = 0.016;

	/*for (int i = 0; i < world.numT; i++)
	{
		if (world.robotT[i].id == robot_number)
		{
			xc = world.robotT[i].position.getX();
			yc = world.robotT[i].position.getY();
			rc = world.robotT[i].angle;
			break;
		}
	}*/

	xd = destination.getX();
	yd = destination.getY();
	rd = rotation;

	//p_error_x = xd - xc;
	//p_error_y = yd - yc;

	///////////////////////////////////////????????????????
	//error_r = PD::cal_error_r(rc, rd);
	//p_error_r = error_r;
	///////////////////////////////////////????????????????

	/*if (p_error_x < 800 && p_error_x > -800)
		close_x = true;
	else
		close_x = false;

	if (p_error_y < 800 && p_error_y > -800)
		close_y = true;
	else
		close_y = false;*/

	for (int i = 0; i < world.numT; i++)
	{
		if (world.robotT[i].id == robot_number)
		{
			xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
			yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
			rc = world.robotT[i].angle;
			break;
		}
	}

	if (abs(xd - xc) < 30 && abs(yd - yc) < 30)
		return VecPosition(0,0);

	//rotate && ....  isnt tested!!
	//int chart_loop = 0;


	//abs(xc - xd) > 60 || abs(yc - yd) > 60 || (rotate && (abs(rd - rc) > 0.1))
	if (true)
	{

		auto start_time = std::chrono::high_resolution_clock::now();

		//chart_loop++;
		//PD::shift_AI_speeds();
		//PD::shift_ROBOT_speeds();

		for (int i = 0; i < world.numT; i++)
		{
			if (world.robotT[i].id == robot_number)
			{
				xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
				yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
				rc = world.robotT[i].angle;
				break;
			}
		}


		//if (((error_x < 800 && error_x > 500) || (error_x > -800 && error_x < -500)) && close_x)
		//{
		//	kp_x = 0.05; //0.7
		//	kp_y = 0.05;
		//}
		//else if (((error_x < 500 && error_x > 100) || (error_x > -500 && error_x < -100)) && close_x)
		//{
		//	kp_x = 0.1; // 1.3
		//	kp_y = 0.1;
		//}
		//else if (error_x < 100 && error_x > -100 && close_x)
		//{
		//	kp_x = 0.2; // 3
		//	kp_y = 0.2;
		//}
		//else
		//{
		//	kp_x = 0.05; //0.8
		//	kp_y = 0.05;
		//}
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////change!!

		error_x = xd - xc;
		error_y = yd - yc;

		size_of_error = sqrt(pow(error_x, 2) + pow(error_y, 2));
		kp_x = pow(M_E, (-(size_of_error/ 30) - 0.25)) + 0.05;
		kp_y = kp_x;
		system("cls");
		cout << kp_x << endl;
		cout << size_of_error << endl;


		if (working_destination != destination)
		{
			working_destination = destination;

			//p_error_r = error_r;
			p_error_x = error_x;
			p_error_y = error_y;

			integral_x = 0;
			derivative_x = (error_x - p_error_x) / dt;
			u_x = error_x * kp_x + kd_x * derivative_x;

			integral_y = 0;
			derivative_y = (error_y - p_error_y) / dt;
			u_y = error_y * kp_y + kd_y * derivative_y;
		}

		else
		{
			integral_x = integral_x + error_x*dt;
			derivative_x = (error_x - p_error_x) / dt;
			u_x = error_x * kp_x + kd_x * derivative_x /*+ ki_x * integral_x*/;
			p_error_x = error_x;

			integral_y = integral_y + error_y*dt;
			derivative_y = (error_y - p_error_y) / dt;
			u_y = error_y * kp_y + kd_y * derivative_y /*+ ki_y * integral_y*/;
			p_error_y = error_y;
		}

		//if (((error_y < 800 && error_y > 500) || (error_y > -800 && error_y < -500)) && close_y)
		//{
		//	kp_y = 0.05; //0.7
		//	kp_x = 0.05;
		//}
		//else if (((error_y < 500 && error_y > 100) || (error_y > -500 && error_y < -100)) && close_y)
		//{
		//	kp_y = 0.1; //1.3
		//	kp_x = 0.1;
		//}
		//else if (error_y < 100 && error_y > -100 && close_y)
		//{
		//	kp_y = 0.2; //3
		//	kp_x = 0.2;
		//}
		//else
		//{
		//	kp_y = 0.05; //0.8
		//	kp_x = 0.05;
		//}

		//if (chart_loop == 1)
		//{
		//	chart_loop = 0;
		//	World::AI_speeds[0][0] = 3 * (u_x / 2000.000);
		//	World::AI_speeds[0][1] = 3 * (u_y / 2000.000);
		//	/*if (World::ROBOT_speeds[World::number_of_speeds - 1][0] != 0 && World::ROBOT_speeds[World::number_of_speeds - 1][1] != 0)
		//	{
		//	auto e_time = std::chrono::high_resolution_clock::now();
		//	e_time = s_time - e_time;
		//	}*/
		//	/*World::ROBOT_speeds[0][0] = (3 * (World::robot_speed.getX())) / 700.000;
		//	World::ROBOT_speeds[0][1] = (3 * (World::robot_speed.getY())) / 700.000;*/
		//}
		//World::ROBOT_speeds[0][0] = 3 * ((150*((((int)world.robotT[0].velocity.getX())/100) / 150))) / 7.000;
		//World::ROBOT_speeds[0][1] = 3 * ((150*((((int)world.robotT[0].velocity.getY())/100) / 150))) / 7.000;

		//set min u_x

		if (u_x < min_u_x && u_x > -min_u_x)
		{
			if (u_x > 0)
			{
				u_x = min_u_x;
			}

			if (u_x < 0)
			{
				u_x = -min_u_x;
			}
		}

		//set min u_y

		if (u_y < min_u_y && u_y > -min_u_y)
		{
			if (u_y > 0)
			{
				u_y = min_u_y;
			}

			if (u_y < 0)
			{
				u_y = -min_u_y;
			}
		}

		v.setX(u_x);		//taein x bordar sorat
		v.setY(u_y);		//taein y bordar sorat


#pragma region Rotation

///////////////////////////// rotate??? bad!
		///PD baraye rotation robat dar hein harekat va dar nazdiki maghsad
		if (rotate)
		{
			for (int i = 0; i < world.numT; i++)
			{
				if (world.robotT[i].id == robot_number)
				{
					rc = world.robotT[i].angle;
				}
			}
			if ((abs(xc - xd) < 300 && abs(yc - yd) < 300) && (rd - rc) > 0.08)
			{
				error_r = PD::cal_error_r(rc, rd);
				if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) /*&& close_r*/)
				{
					kp_r = 0.03;		//0.06
				}
				else if (error_r < 0.3 && error_r > -0.3 /*&& close_r*/)
				{
					kp_r = 0.02;
				}
				else
				{
					kp_r = 0.015;
				}

				derivative_r = (error_r - p_error_r) / dt;
				u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
				p_error_r = error_r;

				//set min_r
				if (u_r < min_u_r && u_r > -min_u_r)
				{
					if (u_r > 0)
					{
						u_r = min_u_r;
					}

					if (u_r < 0)
					{
						u_r = -min_u_r;
					}
				}

				//set max u_r
				if (u_r > max_u_r)
				{
					u_r = max_u_r;
				}

				//nrf::go(v, robot_number, robot_id, world, 0);
				return v;
			}
			else
			{
				//nrf::go(v, robot_number, robot_id, world, 0);
				return v;
			}
		}
		else
		{
			//nrf::go(v, robot_number, robot_id, world, 0);
			return v;
		}




		//agar hadaf ghabl az residan robat be hadaf taghir kard,halghe ro dobare az avval shoroo kon.
		/*if (working_position_x != world.mouseX || working_position_x != world.mouseX)
		break;*/

		///PD baraye rotation robat dar maghsad
		if (rotate)
		{
			if (abs(xc - xd) <= 60 && abs(yc - yd) <= 60)
			{
				v.setX(0);
				v.setY(0);

				for (int i = 0; i < world.numT; i++)
				{
					if (world.robotT[i].id == robot_number)
					{
						rc = world.robotT[i].angle;
					}
				}

				error_r = PD::cal_error_r(rc, rd);

				if (abs(error_r) > 0.1)
				{
					auto start_time_r = std::chrono::high_resolution_clock::now();

					for (int i = 0; i < world.numT; i++)
					{
						if (world.robotT[i].id == robot_number)
						{
							rc = world.robotT[i].angle;
						}
					}

					error_r = PD::cal_error_r(rc, rd);
					p_error_r = error_r;

					if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) /*&& close_r*/)
					{
						kp_r = 0.03;		//0.06
					}
					else if (error_r < 0.3 && error_r > -0.3 /*&& close_r*/)
					{
						kp_r = 0.02;
					}
					else
					{
						kp_r = 0.015;
					}

					derivative_r = (error_r - p_error_r) / dt;
					u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
					p_error_r = error_r;

					//set min u_r
					if (u_r < min_u_r && u_r > -min_u_r)
					{
						if (u_r > 0)
						{
							u_r = min_u_r;
						}

						if (u_r < 0)
						{
							u_r = -min_u_r;
						}
					}

					//set max u_r
					if (u_r > max_u_r)
					{
						u_r = max_u_r;
					}

					//nrf::go(v, robot_number, robot_id, world, u_r);
					return v;

					auto end_time_r = std::chrono::high_resolution_clock::now();
					auto time_r = end_time_r - start_time_r;
					/*if (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() < 16000)
					{
						sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() / 1000));
					}*/


					/*if (working_position_x != world.mouseX || working_position_x != world.mouseX)
					break;*/

					for (int i = 0; i < world.numT; i++)
					{
						if (world.robotT[i].id == robot_number)
						{
							rc = world.robotT[i].angle;
						}
					}

					error_r = PD::cal_error_r(rc, rd);
				}

			}
		}

		/*auto end_time = std::chrono::high_resolution_clock::now();
		auto time = end_time - start_time;

		if (std::chrono::duration_cast<std::chrono::microseconds>(time).count() < 16000)
		{
			sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000));
		}*/
#pragma endregion

	}

	v.setX(0);		//taein x bordar sorat
	v.setY(0);		//taein y bordar sorat

	//nrf::go(v, robot_number, robot_id, world, 0);
	return v;
	//std::cout << "reached!" << endl;
}
double PD::cal_error_r(double rc, double rd)
{
	double error_r = abs(rd - rc);
	double front_side_rd;

	if (error_r > M_PI)
		error_r = 2 * M_PI - error_r;

	if (rd < 0)
		front_side_rd = rd + M_PI;
	else
		front_side_rd = rd - M_PI;

	if (rc < front_side_rd && rc<rd || rc > front_side_rd && rc>rd)
		return -error_r;
	else
		return error_r;

}

#pragma region "For Drawing Chart"
#if DRAW_CHART == 1
void PD::shift_AI_speeds()
{
	for (int i = World::number_of_speeds - 1; i > 0; i--)
	{
		World::AI_speeds[i][0] = World::AI_speeds[i - 1][0];
		World::AI_speeds[i][1] = World::AI_speeds[i - 1][1];
	}
}

void PD::shift_ROBOT_speeds()
{
	for (int i = World::number_of_speeds - 1; i > 0; i--)
	{
		World::ROBOT_speeds[i][0] = World::ROBOT_speeds[i - 1][0];
		World::ROBOT_speeds[i][1] = World::ROBOT_speeds[i - 1][1];
	}
}
#endif
#pragma endregion

///old GoPd
//void PD::GoPd(int robot_num, int robot_id, World &world, Cartesian_Coordinates destination)
//{
//	//dar comment ha id badane robot dorost nashode....
//	sleep(5000);
//	VecPosition v;	        //baraye taein bordar sorat
//
//	double u_x;				//baraye voroodi sorat_x robat
//	double u_y;				//baraye voroodi sorat_y robat
//	double kp_x;			//zaribe  p baraye x robat
//	double kp_y;			//zaribe  p baraye y robat
//	double derivative_x;	//moshtagh error baraye x
//	double derivative_y;	//moshtagh error baraye y
//	double kd_x;			//zaribe  d baraye x robat
//	double kd_y;			//zaribe  d baraye y robat
//	double p_error_x;		//error ghabli baraye x robat
//	double p_error_y;		//error ghabli baraye y robat
//	double error_x;			//error baraye x robat
//	double error_y;			//error baraye x robat
//	double dt;				//baze zamani baraye barresi robat
//	double xc;				//x feeli robat
//	double yc;				//y feeli robat
//	double xd;				//x maghsad robat
//	double yd;				//y maghsad robat
//	double working_position_x;	//taein mokhtasat delkhah rooye zamin
//	double working_position_y;	//taein mokhtasat delkhah rooye zamin
//	double vsize;
//	double count = 0;
//	bool close_x = false;
//	bool close_y = false;
//	bool close_r = false;
//
//	double rotation;		//meghdare mored niaz charkhesh
//	double kp_r;
//	double kd_r;
//	double error_r;
//	double derivative_r;
//	double p_error_r;
//	double u_r;
//
//	int id = 1;
//	int robot_number = 2;
//	kp_x = 0.6;
//	kd_x = 0.05;
//
//	kp_y = 0.6;
//	kd_y = 0.05;
//
//	dt = 0.016;
//
//	kp_r = 0.015;
//	kd_r = 0.00100;
//
//	rotation = 0;
//	//rotation = M_PI / 10000000.000000000;
//	//rotation =2* (M_PI/3 );
//	//rotation = (M_PI);
//
//	while (true)
//	{
//		//sleep(2000);
//		for (int i = 0; i < world.numT; i++)
//		{
//			if (world.robotT[i].id == robot_number)
//			{
//				xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
//				yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
//			}
//		}
//
//		xd = world.mouseX;		//mokhtasati ke ba mouse click kardim.
//		yd = world.mouseY;		//mokhtasati ke ba mouse click kardim.
//
//		working_position_x = world.mouseX;
//		working_position_y = world.mouseY;
//
//		p_error_x = xd - xc;
//		p_error_y = yd - yc;
//		for (int i = 0; i < world.numT; i++)
//		{
//			if (world.robotT[i].id == robot_number)
//			{
//				p_error_r = rotation - world.robotT[i].angle;
//			}
//		}
//
//
//		if (p_error_x < 800 && p_error_x > -800)
//			close_x = true;
//		else
//			close_x = false;
//
//		if (p_error_y < 800 && p_error_y > -800)
//			close_y = true;
//		else
//			close_y = false;
//
//		if (p_error_r < 0.5 && p_error_r > -0.5)
//			close_r = true;
//		else
//			close_r = false;
//
//
//		//while (abs(world.robotT[robot_number].angle - rotation) > 0.08)
//		//{
//		//	auto start_time_r = std::chrono::high_resolution_clock::now();
//		//	//v.setX(0);		//taein x bordar sorat
//		//	//v.setY(0);		//taein y bordar sorat
//		//	error_r = rotation - world.robotT[robot_number].angle;
//		//	if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) && close_r)
//		//	{
//		//		kp_r = 0.05;
//		//	}
//		//	else if (error_r < 0.3 && error_r > -0.3 && close_r)
//		//	{
//		//		kp_r = 0.03;
//		//	}
//		//	else
//		//	{
//		//		kp_r = 0.015;
//		//	}
//		//	if (abs(rotation - (world.robotT[robot_number].angle)) > M_PI)
//		//	{
//		//		if (rotation  < 0)
//		//		{
//		//			error_r = 2 * M_PI - abs(error_r);
//		//		}
//		//		else
//		//		{
//		//			error_r = error_r - 2 * M_PI;
//		//		}
//		//	}
//		//	derivative_r = (error_r - p_error_r) / dt;
//		//	u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//		//	p_error_r = error_r;
//		//	/*if (u_r < 0.00555555 && u_r > -0.00555555)
//		//	{
//		//	if (u_r > 0)
//		//	{
//		//	u_r = 0.006;
//		//	}
//		//	if (u_r < 0)
//		//	{
//		//	u_r = -0.006;
//		//	}
//		//	}*/
//		//	/*
//		//	if (u_r > 0.02)
//		//	{
//		//	u_r = 0.02;
//		//	}*/
//		//	nrf::go(v, id, world, u_r);//dastoor harekat robat
//		//	auto end_time_r = std::chrono::high_resolution_clock::now();
//		//	auto time_r = end_time_r - start_time_r;
//		//	if (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() < 16000)
//		//	{
//		//		sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() / 1000));
//		//	}
//		//	if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//		//		break;
//		//}
//
//		while (abs(xc - xd) > 60 || abs(yc - yd) > 60)
//		{
//
//			auto start_time = std::chrono::high_resolution_clock::now();
//
//			for (int i = 0; i < world.numT; i++)
//			{
//				if (world.robotT[i].id == robot_number)
//				{
//					xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
//					yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
//				}
//			}
//
//			error_x = xd - xc;
//
//			if (((error_x < 800 && error_x > 500) || (error_x > -800 && error_x < -500)) && close_x)
//			{
//				kp_x = 0.7;
//				kp_y = 0.7;
//			}
//			else if (((error_x < 500 && error_x > 100) || (error_x > -500 && error_x < -100)) && close_x)
//			{
//				kp_x = 1.3;
//				kp_y = 1.3;
//			}
//			else if (error_x < 100 && error_x > -100 && close_x)
//			{
//				kp_x = 3;
//				kp_y = 3;
//			}
//			else
//			{
//				kp_x = 0.6;
//				kp_y = 0.6;
//			}
//
//			derivative_x = (error_x - p_error_x) / dt;
//			u_x = error_x * kp_x + kd_x * derivative_x;		//*mohasebe voroodi badi*
//			p_error_x = error_x;
//
//
//
//			error_y = yd - yc;
//
//			if (((error_y < 800 && error_y > 500) || (error_y > -800 && error_y < -500)) && close_y)
//			{
//				kp_y = 0.7;
//				kp_x = 0.7;
//			}
//			else if (((error_y < 500 && error_y > 100) || (error_y > -500 && error_y < -100)) && close_y)
//			{
//				kp_y = 1.3;
//				kp_x = 1.3;
//			}
//			else if (error_y < 100 && error_y > -100 && close_y)
//			{
//				kp_y = 3;
//				kp_x = 3;
//			}
//			else
//			{
//				kp_y = 0.6;
//				kp_x = 0.6;
//			}
//
//			derivative_y = (error_y - p_error_y) / dt;
//			u_y = error_y * kp_y + kd_y * derivative_y;		//*mohasebe voroodi badi*
//			p_error_y = error_y;
//
//			/*vsize = sqrt(pow(u_x, 2) + pow(u_y, 2));
//			u_x = u_x / vsize;
//			u_y = u_y / vsize;*/
//
//			v.setX(u_x);		//taein x bordar sorat
//			v.setY(u_y);		//taein y bordar sorat
//
//
//								//PD baraye rotation robat dar hein harekat va dar nazdiki maghsad
//
//								//if ((abs(xc - xd) < 300 && abs(yc - yd) < 300) && abs(world.robotT[robot_number].angle - rotation) > 0.08)
//								//{
//								//	error_r = rotation - world.robotT[robot_number].angle;
//								//	if (abs(rotation - (world.robotT[robot_number].angle)) > M_PI)
//								//	{
//								//		if (rotation < 0)
//								//		{
//								//			error_r = 2 * M_PI - abs(error_r);
//								//		}
//								//		else
//								//		{
//								//			error_r = error_r - 2 * M_PI;
//								//		}
//								//	}
//								//	derivative_r = (error_r - p_error_r) / dt;
//								//	u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//								//	p_error_r = error_r;
//								//	/*if (u_r < 0.00555555 && u_r > -0.00555555)
//								//	{
//								//	if (u_r > 0)
//								//	{
//								//	u_r = 0.006;
//								//	}
//								//	if (u_r < 0)
//								//	{
//								//	u_r = -0.006;
//								//	}
//								//	}
//								//	if (u_r > 0.02)
//								//	{
//								//	u_r = 0.02;
//								//	}*/
//								//	nrf::go(v, id, world, u_r);//dastoor harekat robat
//								//}
//								//else
//			nrf::go(v, id, world, 0);//dastoor harekat robat
//
//			auto end_time = std::chrono::high_resolution_clock::now();
//			auto time = end_time - start_time;
//
//			if (std::chrono::duration_cast<std::chrono::microseconds>(time).count() < 16000)
//			{
//				sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000));
//			}
//
//
//			//agar hadaf ghabl az residan robat be hadaf taghir kard,halghe ro dobare az avval shoroo kon.
//			if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//				break;
//
//			///PD baraye rotation robat dar maghsad
//
//			//while (abs(xc - xd) < 50 && abs(yc - yd) < 50 && abs(world.robotT[robot_number].angle - rotation) > 0.1)
//			//{
//			//	auto start_time_r = std::chrono::high_resolution_clock::now();
//			//	v.setX(0);		//taein x bordar sorat
//			//	v.setY(0);		//taein y bordar sorat
//			//	error_r = rotation - world.robotT[robot_number].angle;
//			//	if (((error_r < 800 && error_r > 500) || (error_r > -800 && error_r < -500)) && close_r)
//			//	{
//			//		kp_r = 2.5;
//			//	}
//			//	else if (((error_r < 500 && error_r > 100) || (error_r > -500 && error_r < -100)) && close_r)
//			//	{
//			//		kp_r = 3;
//			//	}
//			//	else if (error_r < 100 && error_r > -100 && close_r)
//			//	{
//			//		kp_r = 3.5;
//			//	}
//			//	else
//			//	{
//			//		kp_r = 0.6;
//			//	}
//			//	if (abs(rotation - (world.robotT[robot_number].angle)) > M_PI)
//			//	{
//			//		if (rotation  < 0)
//			//		{
//			//			error_r = 2 * M_PI - abs(error_r);
//			//		}
//			//		else
//			//		{
//			//			error_r = error_r - 2 * M_PI;
//			//		}
//			//	}
//			//	derivative_r = (error_r - p_error_r) / dt;
//			//	u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//			//	p_error_r = error_r;
//			//	/////*if (u_r < 0.00555555 && u_r > -0.00555555)
//			//	////{
//			//	////if (u_r > 0)
//			//	////{
//			//	////u_r = 0.006;
//			//	////}
//			//	////if (u_r < 0)
//			//	////{
//			//	////u_r = -0.006;
//			//	////}
//			//	////}
//			//	////if (u_r > 0.02)
//			//	////{
//			//	////u_r = 0.02;
//			//	////}*/
//			//	nrf::go(v, id, world, u_r);//dastoor harekat robat
//			//	auto end_time_r = std::chrono::high_resolution_clock::now();
//			//	auto time_r = end_time_r - start_time_r;
//			//	if (std::chrono::duration_cast<std::chrono::microseconds>(time).count() < 16000)
//			//	{
//			//		sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000));
//			//	}
//			//	if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//			//		break;
//			//}
//
//
//			//while (abs(xc - xd) < 50 && abs(yc - yd) < 50 && abs(world.robotT[robot_number].angle - rotation) > 0.08)
//			//{
//			//	auto start_time_r = std::chrono::high_resolution_clock::now();
//			//	v.setX(0);		//taein x bordar sorat
//			//	v.setY(0);		//taein y bordar sorat
//			//	error_r = rotation - world.robotT[robot_number].angle;
//			//	if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) && close_r)
//			//	{
//			//		kp_r = 0.05;
//			//	}
//			//	else if (error_r < 0.3 && error_r > -0.3 && close_r)
//			//	{
//			//		kp_r = 0.03;
//			//	}
//			//	else
//			//	{
//			//		kp_r = 0.015;
//			//	}
//			//	if (abs(rotation - (world.robotT[robot_number].angle)) > M_PI)
//			//	{
//			//		if (rotation  < 0)
//			//		{
//			//			error_r = 2 * M_PI - abs(error_r);
//			//		}
//			//		else
//			//		{
//			//			error_r = error_r - 2 * M_PI;
//			//		}
//			//	}
//			//	derivative_r = (error_r - p_error_r) / dt;
//			//	u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//			//	p_error_r = error_r;
//			//	/*if (u_r < 0.00555555 && u_r > -0.00555555)
//			//	{
//			//	if (u_r > 0)
//			//	{
//			//	u_r = 0.006;
//			//	}
//			//	if (u_r < 0)
//			//	{
//			//	u_r = -0.006;
//			//	}
//			//	}*/
//			//	/*
//			//	if (u_r > 0.02)
//			//	{
//			//	u_r = 0.02;
//			//	}*/
//			//	nrf::go(v, id, world, u_r);//dastoor harekat robat
//			//	auto end_time_r = std::chrono::high_resolution_clock::now();
//			//	auto time_r = end_time_r - start_time_r;
//			//	if (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() < 16000)
//			//	{
//			//		sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() / 1000));
//			//	}
//			//	if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//			//		break;
//			//}
//		}
//
//		v.setX(0);		//taein x bordar sorat
//		v.setY(0);		//taein y bordar sorat
//
//		nrf::go(v, id, world, 0);
//	}
//}

///backup of gopd new
//void PD::GoPd(int robot_number, char robot_id, World &world, VecPosition destination, bool rotate, double rotation)
//{
//
//	//auto s_time = std::chrono::high_resolution_clock::now();
//
//	//min_u_x va min_u_y avaz shode!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
//	//sleep(2000);
//
//	//k_p changed
//
//	VecPosition v;	        //baraye taein bordar sorat
//
//	double u_x;				//baraye voroodi sorat_x robat
//	double u_y;				//baraye voroodi sorat_y robat
//	double kp_x;			//zaribe  p baraye x robat
//	double kp_y;			//zaribe  p baraye y robat
//	double derivative_x;	//moshtagh error baraye x
//	double derivative_y;	//moshtagh error baraye y
//	double kd_x;			//zaribe  d baraye x robat
//	double kd_y;			//zaribe  d baraye y robat
//	double p_error_x;		//error ghabli baraye x robat
//	double p_error_y;		//error ghabli baraye y robat
//	double error_x;			//error baraye x robat
//	double error_y;			//error baraye x robat
//	double dt;				//baze zamani baraye barresi robat
//							//double xc;				//x feeli robat
//							//double yc;				//y feeli robat
//	VecPosition current_position;
//	//double xd;				//x maghsad robat
//	//double yd;				//y maghsad robat
//	//double working_position_x;	//taein mokhtasat delkhah rooye zamin
//	//double working_position_y;	//taein mokhtasat delkhah rooye zamin
//	bool close_x = false;
//	bool close_y = false;
//	bool close_r = false;
//	const double min_u_x = 100; //37.3
//	const double min_u_y = 100; //37.3
//
//	double rd;		//meghdare mored niaz charkhesh
//	double kp_r;
//	double kd_r;
//	double error_r;
//	double derivative_r;
//	double p_error_r;
//	double u_r;
//	double rc;
//	const double min_u_r = 0.006;
//	const double max_u_r = 0.02;
//	double vision_time = 0;
//	bool time = true;
//
//	kp_x = 0.6;		//0.6
//	kd_x = 0.05;
//
//	kp_y = 0.6;		//0.6
//	kd_y = 0.05;
//
//	dt = 0.016;
//
//	kp_r = 0.015;
//	kd_r = 0.00100;
//
//	rd = rotation;
//
//	for (int i = 0; i < world.numT; i++)
//	{
//		if (world.robotT[i].id == robot_number)
//		{
//			xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
//			yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
//		}
//	}
//
//	for (int i = 0; i < world.numT; i++)
//	{
//		if (world.robotT[i].id == robot_number)
//		{
//			rc = world.robotT[i].angle;
//		}
//	}
//
//
//	xd = destination.x;		//mokhtasati ke ba mouse click kardim.
//	yd = destination.y;		//mokhtasati ke ba mouse click kardim.
//
//							/*working_position_x = world.mouseX;
//							working_position_y = world.mouseY;*/
//
//	p_error_x = xd - xc;
//	p_error_y = yd - yc;
//	error_r = PD::cal_error_r(rc, rd);
//	p_error_r = error_r;
//
//	if (p_error_x < 800 && p_error_x > -800)
//		close_x = true;
//	else
//		close_x = false;
//
//	if (p_error_y < 800 && p_error_y > -800)
//		close_y = true;
//	else
//		close_y = false;
//
//
//	//rotate && ....  isnt tested!!
//	int chart_loop = 0;
//	while (abs(xc - xd) > 60 || abs(yc - yd) > 60 || (rotate && (abs(rd - rc) > 0.1)))
//	{
//
//		auto start_time = std::chrono::high_resolution_clock::now();
//
//		chart_loop++;
//		PD::shift_AI_speeds();
//		//PD::shift_ROBOT_speeds();
//
//
//		for (int i = 0; i < world.numT; i++)
//		{
//			if (world.robotT[i].id == robot_number)
//			{
//				xc = world.robotT[i].position.getX();		//gereftan mokhtasat jadid
//				yc = world.robotT[i].position.getY();		//gereftan mokhtasat jadid
//			}
//		}
//		for (int i = 0; i < world.numT; i++)
//		{
//			if (world.robotT[i].id == robot_number)
//			{
//				rc = world.robotT[i].angle;
//			}
//		}
//		error_x = xd - xc;
//
//		if (((error_x < 800 && error_x > 500) || (error_x > -800 && error_x < -500)) && close_x)
//		{
//			kp_x = 0.7;
//			kp_y = 0.7;
//		}
//		else if (((error_x < 500 && error_x > 100) || (error_x > -500 && error_x < -100)) && close_x)
//		{
//			kp_x = 1.3;
//			kp_y = 1.3;
//		}
//		else if (error_x < 100 && error_x > -100 && close_x)
//		{
//			kp_x = 3;
//			kp_y = 3;
//		}
//		else
//		{
//			kp_x = 0.8;
//			kp_y = 0.8;
//		}
//
//		derivative_x = (error_x - p_error_x) / dt;
//		u_x = error_x * kp_x + kd_x * derivative_x;		//*mohasebe voroodi badi*
//		p_error_x = error_x;
//
//
//
//		error_y = yd - yc;
//
//		if (((error_y < 800 && error_y > 500) || (error_y > -800 && error_y < -500)) && close_y)
//		{
//			kp_y = 0.7;
//			kp_x = 0.7;
//		}
//		else if (((error_y < 500 && error_y > 100) || (error_y > -500 && error_y < -100)) && close_y)
//		{
//			kp_y = 1.3;
//			kp_x = 1.3;
//		}
//		else if (error_y < 100 && error_y > -100 && close_y)
//		{
//			kp_y = 3;
//			kp_x = 3;
//		}
//		else
//		{
//			kp_y = 0.8;
//			kp_x = 0.8;
//		}
//
//		derivative_y = (error_y - p_error_y) / dt;
//		u_y = error_y * kp_y + kd_y * derivative_y;		//*mohasebe voroodi badi*
//
//		p_error_y = error_y;
//
//		if (chart_loop == 1)
//		{
//			chart_loop = 0;
//			World::AI_speeds[0][0] = 3 * (u_x / 2000.000);
//			World::AI_speeds[0][1] = 3 * (u_y / 2000.000);
//
//			/*if (World::ROBOT_speeds[World::number_of_speeds - 1][0] != 0 && World::ROBOT_speeds[World::number_of_speeds - 1][1] != 0)
//			{
//			auto e_time = std::chrono::high_resolution_clock::now();
//			e_time = s_time - e_time;
//			}*/
//			/*World::ROBOT_speeds[0][0] = (3 * (World::robot_speed.getX())) / 700.000;
//			World::ROBOT_speeds[0][1] = (3 * (World::robot_speed.getY())) / 700.000;*/
//		}
//		//World::ROBOT_speeds[0][0] = 3 * ((150*((((int)world.robotT[0].velocity.getX())/100) / 150))) / 7.000;
//		//World::ROBOT_speeds[0][1] = 3 * ((150*((((int)world.robotT[0].velocity.getY())/100) / 150))) / 7.000;
//
//
//
//		//set min u_x
//		if (u_x < min_u_x && u_x > -min_u_x)
//		{
//			if (u_x > 0)
//			{
//				u_x = min_u_x;
//			}
//
//			if (u_x < 0)
//			{
//				u_x = -min_u_x;
//			}
//		}
//
//		//set min u_x
//		if (u_y < min_u_y && u_y > -min_u_y)
//		{
//			if (u_y > 0)
//			{
//				u_y = min_u_y;
//			}
//
//			if (u_y < 0)
//			{
//				u_y = -min_u_y;
//			}
//		}
//
//
//
//		v.setX(u_x);		//taein x bordar sorat
//		v.setY(u_y);		//taein y bordar sorat
//
//							///PD baraye rotation robat dar hein harekat va dar nazdiki maghsad
//		if (rotate)
//		{
//			for (int i = 0; i < world.numT; i++)
//			{
//				if (world.robotT[i].id == robot_number)
//				{
//					rc = world.robotT[i].angle;
//				}
//			}
//			if ((abs(xc - xd) < 300 && abs(yc - yd) < 300) && (rd - rc) > 0.08)
//			{
//				error_r = PD::cal_error_r(rc, rd);
//				if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) /*&& close_r*/)
//				{
//					kp_r = 0.03;		//0.06
//				}
//				else if (error_r < 0.3 && error_r > -0.3 /*&& close_r*/)
//				{
//					kp_r = 0.02;
//				}
//				else
//				{
//					kp_r = 0.015;
//				}
//
//				derivative_r = (error_r - p_error_r) / dt;
//				u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//				p_error_r = error_r;
//
//				//set min_r
//				if (u_r < min_u_r && u_r > -min_u_r)
//				{
//					if (u_r > 0)
//					{
//						u_r = min_u_r;
//					}
//
//					if (u_r < 0)
//					{
//						u_r = -min_u_r;
//					}
//				}
//
//				//set max u_r
//				if (u_r > max_u_r)
//				{
//					u_r = max_u_r;
//				}
//
//				nrf::go(v, robot_number, robot_id, world, 0);
//			}
//			else
//				nrf::go(v, robot_number, robot_id, world, 0);
//		}
//		else
//			nrf::go(v, robot_number, robot_id, world, 0);
//
//
//
//
//
//		//agar hadaf ghabl az residan robat be hadaf taghir kard,halghe ro dobare az avval shoroo kon.
//		/*if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//		break;*/
//
//		///PD baraye rotation robat dar maghsad
//		if (rotate)
//		{
//			if (abs(xc - xd) <= 60 && abs(yc - yd) <= 60)
//			{
//				v.setX(0);
//				v.setY(0);
//
//				for (int i = 0; i < world.numT; i++)
//				{
//					if (world.robotT[i].id == robot_number)
//					{
//						rc = world.robotT[i].angle;
//					}
//				}
//
//				error_r = PD::cal_error_r(rc, rd);
//
//				while (abs(error_r) > 0.1)
//				{
//					auto start_time_r = std::chrono::high_resolution_clock::now();
//
//					for (int i = 0; i < world.numT; i++)
//					{
//						if (world.robotT[i].id == robot_number)
//						{
//							rc = world.robotT[i].angle;
//						}
//					}
//
//					error_r = PD::cal_error_r(rc, rd);
//					p_error_r = error_r;
//
//					if (((error_r < 0.5 && error_r > 0.3) || (error_r > -0.5 && error_r < -0.3)) /*&& close_r*/)
//					{
//						kp_r = 0.03;		//0.06
//					}
//					else if (error_r < 0.3 && error_r > -0.3 /*&& close_r*/)
//					{
//						kp_r = 0.02;
//					}
//					else
//					{
//						kp_r = 0.015;
//					}
//
//					derivative_r = (error_r - p_error_r) / dt;
//					u_r = error_r * kp_r + kd_r * derivative_r;		//*mohasebe voroodi badi*
//					p_error_r = error_r;
//
//					//set min u_r
//					if (u_r < min_u_r && u_r > -min_u_r)
//					{
//						if (u_r > 0)
//						{
//							u_r = min_u_r;
//						}
//
//						if (u_r < 0)
//						{
//							u_r = -min_u_r;
//						}
//					}
//
//					//set max u_r
//					if (u_r > max_u_r)
//					{
//						u_r = max_u_r;
//					}
//
//					nrf::go(v, robot_number, robot_id, world, u_r);
//
//					auto end_time_r = std::chrono::high_resolution_clock::now();
//					auto time_r = end_time_r - start_time_r;
//					if (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() < 16000)
//					{
//						sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time_r).count() / 1000));
//					}
//					/*if (working_position_x != world.mouseX || working_position_x != world.mouseX)
//					break;*/
//
//					for (int i = 0; i < world.numT; i++)
//					{
//						if (world.robotT[i].id == robot_number)
//						{
//							rc = world.robotT[i].angle;
//						}
//					}
//
//					error_r = PD::cal_error_r(rc, rd);
//				}
//
//			}
//		}
//
//		auto end_time = std::chrono::high_resolution_clock::now();
//		auto time = end_time - start_time;
//
//		if (std::chrono::duration_cast<std::chrono::microseconds>(time).count() < 16000)
//		{
//			sleep(16 - (std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000));
//		}
//	}
//
//	v.setX(0);		//taein x bordar sorat
//	v.setY(0);		//taein y bordar sorat
//
//	nrf::go(v, robot_number, robot_id, world, 0);
//	//std::cout << "reached!" << endl;
//}

/*! PD METHODS*/



/*! TOOLS METHODS*/



/*! TOOLS METHODS*/



/*! SPEED DIAGRAM METHODS */

VecPosition SpeedDiagram::genVecVel_forGrsim (int robot_num, VecPosition destination_position)
{
	VecPosition position_error_vector;
	VecPosition position_error_unit_vector;
	VecPosition velocity_robot_desired;
	double error_vector_size;
	double velocity;
	int ifi;

	ifi = world.getIndexForRobotTNumber(robot_num);

	position_error_vector = destination_position - world.robotT[ifi].position;
	error_vector_size = position_error_vector.getMagnitude();
	if (error_vector_size == 0)
	{
		velocity_robot_desired.setX(0);
		velocity_robot_desired.setY(0);
		return velocity_robot_desired;
	}
	else
		position_error_unit_vector = position_error_vector / error_vector_size;

	if (careful)
		velocity = pow((error_vector_size / V_d_grsim), V_p_grsim)*Tools::u(-(error_vector_size - V_MEr_grsim + 1)) + V_MSp_grsim*Tools::u(error_vector_size - V_MEr_grsim);	///g function
	else
		velocity = (-pow(((sign(error_vector_size - V_MEr_grsim)*(error_vector_size - V_MEr_grsim)) / V_d_grsim), V_p_grsim) + V_MSp_grsim)*Tools::u(-(error_vector_size - V_MEr_grsim + 1)) + V_MSp_grsim*Tools::u(error_vector_size - V_MEr_grsim);	 ///f function

	velocity_robot_desired = position_error_unit_vector*velocity;

	return velocity_robot_desired;
}
VecPosition SpeedDiagram::genVecVel_forNrf(int robot_num, VecPosition destination_position)
{
	VecPosition position_error_vector;
	VecPosition position_error_unit_vector;
	VecPosition velocity_robot_desired;
	double error_vector_size;
	double velocity;
	int ifi;

	ifi = world.getIndexForRobotTNumber(robot_num);

	position_error_vector = destination_position - world.robotT[ifi].position;
	error_vector_size = position_error_vector.getMagnitude();
	if (error_vector_size == 0)
	{
		velocity_robot_desired.setX(0);
		velocity_robot_desired.setY(0);
		return velocity_robot_desired;
	}
	else
		position_error_unit_vector = position_error_vector / error_vector_size;

	if (careful)
		velocity = pow((error_vector_size / V_d_nrf), V_p_nrf)*Tools::u(-(error_vector_size - V_MEr_nrf + 1)) + V_MSp_nrf*Tools::u(error_vector_size - V_MEr_nrf);	///g function
	else
		velocity = (-pow(((sign(error_vector_size - V_MEr_nrf)*(error_vector_size - V_MEr_nrf)) / V_d_nrf), V_p_nrf) + V_MSp_nrf)*Tools::u(-(error_vector_size - V_MEr_nrf + 1)) + V_MSp_nrf*Tools::u(error_vector_size - V_MEr_nrf);	 ///f function

	velocity_robot_desired = position_error_unit_vector*velocity;

	return velocity_robot_desired;
}
double SpeedDiagram::genVecW_forGrsim(int robot_num, double angle)
{
	/// for both vision and grsim, angle is: 3 o'clock is 0, 12 o'clock is M_PI/2, 9 o'clock is M_PI or -M_PI, 6 o'clock is -M_PI/2
	///if robot is excatly in 0 or M_PI(-M_PI) and we have delay, it first move clockwise and reverse first,then mrotates good
	double angle_error;
	int ifi;
	double w;
	ifi = world.getIndexForRobotTNumber(robot_num);

	angle_error = cal_error_r(world.robotT[ifi].angle,angle);
	if (angle_error == 0)
		return 0;
	/*if (careful)
	{
		w = pow((angle_error / W_d_grsim), W_p_grsim)*Tools::u(-(angle_error - W_MEr_grsim + 1)) + W_MSp_grsim*Tools::u(angle_error - W_MEr_grsim);	///g function
	}
	else
	{*/
		////w = (-pow(((sign(angle_error - W_MEr_grsim)*(angle_error - W_MEr_grsim)) / W_d_grsim), W_p_grsim) + W_MSp_grsim)*Tools::u(-(angle_error - W_MEr_grsim + 1)) + W_MSp_grsim*Tools::u(angle_error - W_MEr_grsim);	 ///f function
		//w = cal_W(angle_error);
	//}
		//cout << w << endl;
	return cal_W_grsim(angle_error);
}
double SpeedDiagram::genVecW_forNrf(int robot_num, double angle)
{
	/// for both vision and grsim, angle is: 3 o'clock is 0, 12 o'clock is M_PI/2, 9 o'clock is M_PI or -M_PI, 6 o'clock is -M_PI/2
	///if robot is excatly in 0 or M_PI(-M_PI) and we have delay, it first move clockwise and reverse first,then rotates good
	double angle_error;
	int ifi;
	double w;

	ifi = world.getIndexForRobotTNumber(robot_num);

	angle_error = cal_error_r(world.robotT[ifi].angle, angle);
	if (angle_error == 0)
		return 0;
	/*if (careful)
	{
	w = pow((angle_error / W_d_grsim), W_p_grsim)*Tools::u(-(angle_error - W_MEr_grsim + 1)) + W_MSp_grsim*Tools::u(angle_error - W_MEr_grsim);	///g function
	}
	else
	{*/
	////w = (-pow(((sign(angle_error - W_MEr_grsim)*(angle_error - W_MEr_grsim)) / W_d_grsim), W_p_grsim) + W_MSp_grsim)*Tools::u(-(angle_error - W_MEr_grsim + 1)) + W_MSp_grsim*Tools::u(angle_error - W_MEr_grsim);	 ///f function
	//w = cal_W(angle_error);
	//}
	//cout << w << endl;
	return cal_W_nrf(angle_error)+0.005;
}

double SpeedDiagram::cal_error_r(double rc /*current angle*/, double rd /*destination angle*/)
{
	/*double error_r = abs(rd - rc);
	double front_side_rd;

	if (error_r > M_PI)
		error_r = 2 * M_PI - error_r;

	if (rd < 0)
		front_side_rd = rd + M_PI;
	else
		front_side_rd = rd - M_PI;

	if (rc < front_side_rd && rc<rd || rc > front_side_rd && rc>rd)
		return -error_r;
	else
		return error_r;*/
	double error = rd - rc;
	return abs(error) < M_PI ? error : -sign(error)*(2 * M_PI - abs(rd - rc));
}
double SpeedDiagram::cal_W_grsim(double ae)
{
	if (ae >= 0)
	{
		if (ae > W_MEr_grsim)
			return W_MSp_grsim;
		else
			return (-pow(((W_MEr_grsim - ae) / W_d_grsim), W_p_grsim) + W_MSp_grsim);
		//(-pow(((sign(angle_error - W_MEr_grsim)*(angle_error - W_MEr_grsim)) / W_d_grsim), W_p_grsim) + W_MSp_grsim);
	}
	else
	{
		if (ae < -W_MEr_grsim)
			return -W_MSp_grsim;
		else
			return -(-pow(((W_MEr_grsim + ae) / W_d_grsim), W_p_grsim) + W_MSp_grsim);
	}
}
double SpeedDiagram::cal_W_nrf(double ae)
{
	if (ae >= 0)
	{
		if (ae > W_MEr_nrf)
			return W_MSp_nrf;
		else
			return (-pow(((W_MEr_nrf - ae) / W_d_nrf), W_p_nrf) + W_MSp_nrf);
		//(-pow(((sign(angle_error - W_MEr_nrf)*(angle_error - W_MEr_nrf)) / W_d_nrf), W_p_nrf) + W_MSp_nrf);
	}
	else
	{
		if (ae < -W_MEr_nrf)
			return -W_MSp_nrf;
		else
			return -(-pow(((W_MEr_nrf + ae) / W_d_nrf), W_p_nrf) + W_MSp_nrf);
	}
}
/*!SPEED DIAGRAM METHODS*/