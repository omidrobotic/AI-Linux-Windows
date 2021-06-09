#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <math.h>
#include "geometry.h"
#include <chrono>
#include "Switches.h"

class World;
class GrsimMove;

class PD
{
public:
	static void GoPD(int robot_num,char robot_id,VecPosition destination_position,bool rotate,double rotation, World &world);
	static VecPosition GenVecVelPD(int robot_num, VecPosition destination_position, bool rotate, double rotation, World &world);	//generate VecPosition velocity PD
	static double cal_error_r(double rc, double rd);

#pragma region "For Drawing Chart"
#if DRAW_CHART == 1
	static void shift_AI_speeds();
	static void shift_ROBOT_speeds();
#endif
#pragma endregion
};



class SpeedDiagram
{
public:
	//VecPosition position_error_vector;
	//VecPosition position_error_unit_vector;
	//VecPosition velocity_robot_desired;
	//VecPosition previous_position_of_robot;

	//double angle_error;
	//double w_robot_desired;
	
	//double MEr_grsim = 750;	///grsim:750
	//double p_grsim = 1; //2	///grsim : 1
	//double MSp_grsim = 700;	//grsim : 700
	//double d_grsim = MEr_grsim / pow(MSp_grsim, (1 / p_grsim));

	//double MEr_nrf = 750;	///grsim:750
	//double p_nrf = 1; //2	///grsim : 1
	//double MSp_nrf = 400;	//grsim : 700
	//double d_nrf = MEr_nrf / pow(MSp_nrf, (1 / p_nrf));

	/* Velocity */
	double V_MEr_nrf = 400;	///grsim:750
	double V_p_nrf = 3; //2	///grsim : 1
	double V_MSp_nrf = 200;	//grsim : 700	///max nice speed : 600
	double V_d_nrf = V_MEr_nrf / pow(V_MSp_nrf, (1 / V_p_nrf));

	double V_MEr_grsim = 750;	///grsim:750
	double V_p_grsim = 1; //2	///grsim : 1
	double V_MSp_grsim = 400;	//grsim : 400
	double V_d_grsim = V_MEr_grsim / pow(V_MSp_grsim, (1 / V_p_grsim));
	/* Velocity */


	/* W */
	double W_MEr_nrf = M_PI/2;	///grsim:750
	double W_p_nrf = 1; //2	///grsim : 1
	double W_MSp_nrf = 0.02;	//grsim : 700	///max nice speed : 600
	double W_d_nrf = W_MEr_nrf / pow(W_MSp_nrf, (1 / W_p_nrf));

	double W_MEr_grsim = M_PI/2;	///grsim:750
	double W_p_grsim = 3; //2	///grsim : 1
	double W_MSp_grsim = M_PI;	//grsim : 700
	double W_d_grsim = W_MEr_grsim / pow(W_MSp_grsim, (1 / W_p_grsim));
	/* W */

	//double error_vector_size;
	//double velocity = 0;
	//double w = 0;
	//int ifi;	//index for id
	bool careful = false;
	//bool ftime = true;
	//double time_for_feedforward = 0;
	//chrono::time_point<chrono::steady_clock> start_time = chrono::high_resolution_clock::now();
	//chrono::time_point<chrono::steady_clock> end_time = chrono::high_resolution_clock::now();
	//chrono::duration<long long, nano> time_auto = end_time - start_time;
	//MatrixD V(4, 1);

	VecPosition genVecVel_forGrsim(int robot_num, VecPosition destination_position);
	VecPosition genVecVel_forNrf(int robot_num, VecPosition destination_position);
	double genVecW_forGrsim(int robot_num, double angle);
	double genVecW_forNrf(int robot_num, double angle);

private:
	double cal_error_r(double rc, double rd);
	double cal_W_grsim(double ae);
	double cal_W_nrf(double ae);
};
#endif // !MOTION_CONTROL_H

