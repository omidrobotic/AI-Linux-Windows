#include "Switches.h"
#include "Referee.h"
#include "Vision.h"
#include "estimation.h"
#include "Socket_udp.h"
#include <thread>
#include <mutex>
#include <ctime>
#include "GetSystemData.h"
#if SEND_COMMANDS_TO_ROBOTS==2
#else
#include "Protobuf/Grsim/grSim_Commands.pb.h"
#include "Protobuf/Grsim/grSim_Packet.pb.h"
#include "Protobuf/Grsim/grSim_Replacement.pb.h"
#endif
#include "FieldGeometry.h"
#include "world.h"
#include "graphical/glframe.h"
#include "nrf.h"
#include <fstream>
#include <random>
#include "RRT.h"
#include "MotionControl.h"
//#include "engine.h"
//#include "Matlab.h"
#include "Vision.h"
#include "string"
#include "Switches.h"


#define FOREGROUND_BLUE      0x0001 // text color contains blue.
#define FOREGROUND_GREEN     0x0002 // text color contains green.
#define FOREGROUND_RED       0x0004 // text color contains red.
#define FOREGROUND_INTENSITY 0x0008 // text color is intensified.
#define BACKGROUND_BLUE      0x0010 // background color contains blue.
#define BACKGROUND_GREEN     0x0020 // background color contains green.
#define BACKGROUND_RED       0x0040 // background color contains red.
#define BACKGROUND_INTENSITY 0x0080 // background color is intensified.




World world;
GLFrame glFrame;
//HINSTANCE ghInst;
LPCSTR  lpClassName = "ort";



MatrixD VFM(4, 1);
VecPosition SFM (0,0);
double SSFM = 200;
bool w_not_pressed=true, s_not_pressed=true, a_not_pressed=true, d_not_pressed=true;

#define wheels_manual false
#define move_manual false

#if wheels_manual
#define w_manual
#endif

#if move_manual
#define m_manual
#endif

void Robot_Move_Manual_Function(unsigned char key)
{
	switch (key)
	{
	case 'W':if (w_not_pressed) {
		SFM = SFM + VecPosition(SSFM, 0);
		w_not_pressed = false;
	}
		break;

	case 'S':if (s_not_pressed) {
		SFM = SFM + VecPosition(-SSFM, 0);
		s_not_pressed = false;
	}
		break;

	case 'A':if (a_not_pressed) {
		SFM = SFM + VecPosition(0, SSFM);
		a_not_pressed = false;
	}
		break;

	case 'D':if (d_not_pressed) {
		SFM = SFM + VecPosition(0, -SSFM);
		d_not_pressed = false;
	}
		break;


	case 'w':if (w_not_pressed) {
		SFM = SFM + VecPosition(SSFM, 0);
		w_not_pressed = false;
	}
		break;
	case 's':if (s_not_pressed) {
		SFM = SFM + VecPosition(-SSFM, 0);
		s_not_pressed = false;
	}
		break;
	case 'a':if (a_not_pressed) {
		SFM = SFM + VecPosition(0, SSFM);
		a_not_pressed = false;
	}
		break;
	case 'd':if (d_not_pressed) {
		SFM = SFM + VecPosition(0, -SSFM);
		d_not_pressed = false;
	}
		break;


	case 'H':
		SSFM += 100;
		break;
	case 'h':
		SSFM += 100;
		break;
	case 'L':
		if(SSFM>=100)
			SSFM -= 100;
		break;
	case 'l':
		if (SSFM >= 100)
			SSFM -= 100;
		break;
	}
}
void Robot_Move_ManualUp_Function(unsigned char key)
{
	switch (key)
	{
	case 'W':
		SFM = SFM + VecPosition(-SSFM, 0);
		w_not_pressed = true;
		break;
	case 'S':
		SFM = SFM + VecPosition(SSFM, 0);
		s_not_pressed = true;
		break;
	case 'A':
		SFM = SFM + VecPosition(0, -SSFM);
		a_not_pressed = true;
		break;
	case 'D':
		SFM = SFM + VecPosition(0, SSFM);
		d_not_pressed = true;
		break;


	case 'w':
		SFM = SFM + VecPosition(-SSFM, 0);
		w_not_pressed = true;
		break;
	case 's':
		SFM = SFM + VecPosition(SSFM, 0);
		s_not_pressed = true;
		break;
	case 'a':
		SFM = SFM + VecPosition(0, -SSFM);
		a_not_pressed = true;
		break;
	case 'd':
		SFM = SFM + VecPosition(0, SSFM);
		d_not_pressed = true;
		break;
	}
}
void Robot_Wheels_Manual_Function(unsigned char key)
{
	switch (key)
	{
	case '1':
		if(VFM(0, 0)<30)
		VFM(0, 0) += 1;
		break;
	case 'q':
		if (VFM(0, 0)>0)
		VFM(0, 0) -= 1;
		break;
	case 'Q':
		if (VFM(0, 0)>0)
		VFM(0, 0) -= 1;
		break;



	case '2':
		if (VFM(1, 0)<30)
		VFM(1, 0) += 1;
		break;
	case 'w':
		if (VFM(1, 0)>0)
		VFM(1, 0) -= 1;
		break;
	case 'W':
		if (VFM(1, 0)>0)
		VFM(1, 0) -= 1;
		break;



	case '3':
		if (VFM(2, 0)<30)
		VFM(2, 0) += 1;
		break;
	case 'e':
		if (VFM(2, 0)>0)
		VFM(2, 0) -= 1;
		break;
	case 'E':
		if (VFM(2, 0)>0)
		VFM(2, 0) -= 1;
		break;



	case '4':
		if (VFM(3, 0)<30)
		VFM(3, 0) += 1;
		break;
	case 'r':
		if (VFM(3, 0)>0)
		VFM(3, 0) -= 1;
		break;
	case 'R':
		if (VFM(3, 0)>0)
		VFM(3, 0) -= 1;
		break;
	}
}
static void redraw(void)
{
	glFrame.paintGL(world);
}
void timer(int time)
{
	glutPostRedisplay();
	glutTimerFunc(32, timer, 0);	//32
}
void mouseCB(int button, int state, int x, int y)
{
	glFrame.mouseCB(world, button, state, x, y);
}
void Keyboard(unsigned char key, int x, int y)
{
	glFrame.Keyboard(world, key, x, y);

#ifdef w_manual
	Robot_Wheels_Manual_Function(key);
#elif defined m_manual
	Robot_Move_Manual_Function(key);
#endif
}
void KeyboardUp(unsigned char key, int x, int y)
{
#ifdef m_manual
	Robot_Move_ManualUp_Function(key);
#endif
}



//strategy
void produceRobotsDestinations();

//rrt
void producePathsToDestinations();

//MC
void produceSpeedOfRobots();

//===== Main program ========================================================
int main(int argc, char **argv)
{
	GetSystemData mysystemip;
	const char* myip = mysystemip.GetIP();

	std::cout <<"Valid ip: "<< myip<<std::endl;

	Estimation estimation;
	world.glTimer.start();

	world.setTeamSide(TS_RightSide);

	mode_State mode_state;
	uint32_t stage_time_left;
	Refree refree;
	//Velocity_generate V_gen(world);
	Vision vision_parse(world);
	Socket_udp grsim_udp;
	Socket_udp app_udp;
	Socket_udp nrf_udp_send, nrf_udp_client;



	vision_parse.TrackingMode = false;//badan bayad entekhab shodani beshe

	vision_parse.use_camera[0] = true;
	vision_parse.use_camera[1] = true;
	vision_parse.use_camera[2] = true;
	vision_parse.use_camera[3] = true;


	world.setPlayMode(mode_State::PlayMode::Stop);
	world.setKickMode(mode_State::KickMode::NoKickMode);

	VecPosition temp_ball, temp_ball_vel;
	temp_ball.setVecPosition(0, 0);
	world.ball.setCurrentBallPosition(temp_ball, 0);
	temp_ball_vel.setVecPosition(0, 0);
	world.ball.setVelocity(temp_ball_vel);

	auto refree_func = [&]()
	{
		uint32_t m = 100;
		refree.recive_Init();
		while (true)
		{
			//auto start_time1 = std::chrono::high_resolution_clock::now();

			refree.Refree_parser(world);
			if (refree.m_counter != m)
			{
				cout << "\n KickMode :" << mode_state.getKickModeName(world.kickMode);
				cout << "\n PlayMode :" << mode_state.getPlayModeName(world.playMode);
				m = refree.m_counter;
			}
			//auto end_time1 = std::chrono::high_resolution_clock::now();
			//auto time1 = end_time1 - start_time1;
			//std::cout << "\n took loop refre \n" <<
			//std::chrono::duration_cast<std::chrono::microseconds>(time1).count();
           // sleep(0.016);
		}
	};
	auto vision_func = [&]()
	{
		int i = 0;

		VecPosition last_ball_for_switch_to_plying;
		vision_parse.recive_Init();


		while (true)
		{

			//auto start_time2 = std::chrono::high_resolution_clock::now();
			vision_parse.recievePacket();
			vision_parse.ProcessVision(world);
			estimation.RUN(world);
			switch (world.getInstance().playMode)
			{
			case mode_State::Stop:
			{
				last_ball_for_switch_to_plying = world.ball.getCurrentBallPosition();
			}
			case mode_State::Wait:
			{
				if (last_ball_for_switch_to_plying.getDistanceTo(world.ball.getCurrentBallPosition()) > 100)
				{
					world.setPlayMode(mode_State::Play);
					cout << "\n KickMode :" << mode_state.getKickModeName(world.kickMode);
					cout << "\n PlayMode :" << mode_state.getPlayModeName(world.playMode);
				}
				break;
			}
			break;
			}

			//auto end_time2 = std::chrono::high_resolution_clock::now();
			//auto time2 = end_time2 - start_time2;
			//std::cout << "\n took loop vision \n" <<
			//std::chrono::duration_cast<std::chrono::microseconds>(time2).count();



		}
	};
	auto GLUT_func = [&]()
	{
        glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH | BACKGROUND_INTENSITY );

		//glutInitWindowSize(850, 550);
		glutInitWindowSize(650, 650);
		glutCreateWindow("OMID  Robotic  Team ");
		glutDisplayFunc(redraw);
		glMatrixMode(GL_PROJECTION);
		glMatrixMode(GL_MODELVIEW);
		glutKeyboardFunc(Keyboard);
		glutKeyboardUpFunc(KeyboardUp);
		glutMouseFunc(mouseCB);
		glutTimerFunc(32, timer, 0);	//32
		glutMainLoop();
	};
	auto radio_func = [&]()
	{
		//!!!!!!!!!!!!!!!!!!!! not to be seperated from main thread because of nrf 30 byte !!!!!!!!!!!!!!!!! : no!
		sleep(6);
		///send command to real robots
#if SEND_COMMANDS_TO_ROBOTS == 1

		int rnfi;	///robot number for index
		while (true)
		{
			for (int i = 0; i < world.numT; i++)
			{
				if (world.robotT[i].send_command == true)
				{
					rnfi = world.getRobotTNumberForIndex(i);
                //    nrf::set_kick(world.robotT[i].shoot_or_chip, world.robotT[i].kick_power);
					nrf::go_withoutSend(world.robotT[i].velocityToGo, world.robotT[i].wToGo, rnfi, rnfi, world);
					nrf::write_on_port();
					///cout << "send for robot id " << rnfi << " index " << i << "number " << rnfi << endl;
				}
			}
			//world.exactSleep(16);
		}

		///send command to grsim robots
#elif SEND_COMMANDS_TO_ROBOTS == 0
		SimulatorMove gsm;
        auto set_color_change=world.getInstance().team_color;
		if (MANUAL_ADDR_GRSIM)
			gsm.initialize_port(GROUP_ADDR_SEND_GRSIM_COMMAND, PORT_NUM_SEND_GRSIM_COMMAND);
		else
			gsm.initialize_port(myip, PORT_NUM_SEND_GRSIM_COMMAND);

		gsm.initialize_robot_color_and_timestamp(0.0);
		int rnfi;	///robot number for index
		while (true)
		{
            if (set_color_change!=world.getInstance().team_color) {
                gsm.initialize_robot_color_and_timestamp(0.0);
                set_color_change=world.getInstance().team_color;
            }
			//auto start_time = std::chrono::high_resolution_clock::now();
			for (int i = 0; i < world.numT; i++)
			{
				if (world.robotT[i].send_command)
				{
					rnfi = world.getRobotTNumberForIndex(i);
					gsm.go_setKick_setSpinBack_withoutSend(world.robotT[i].velocityToGo, world.robotT[i].wToGo, world.robotT[i].shoot_or_chip, world.robotT[i].kick_power,world.robotT[i].spinBack, rnfi, world, SimulatorMove::wheels_speed);

				}
			}
			gsm.send_to_grsim();
			sleep(0.005);

			/*auto end_time = std::chrono::high_resolution_clock::now();
			auto time = end_time - start_time;
			cout << time.count() / 1000000.0 << endl;*/
		}
#else
        SimulatorMove ERF;
		int port_name=world.team_T.sendDataPort;
        ERF.initialize_port(GROUP_ADDR_SEND_ERforce_COMMAND, world.team_T.sendDataPort);
        int rnfi;	///robot number for index
        while (true)
        {
            if(port_name!=world.team_T.sendDataPort) {
                ERF.closeUDP();
                sleep(1);
                ERF.initialize_port(GROUP_ADDR_SEND_ERforce_COMMAND, world.team_T.sendDataPort);
                port_name=world.team_T.sendDataPort;
            }
            //auto start_time = std::chrono::high_resolution_clock::now();
            for (int i = 0; i < 11; i++)
            {

                    rnfi = world.getRobotTNumberForIndex(i);
                  //  ERF.go_setKick_setSpinBack_withoutSend(world.robotT[i].velocityToGo, world.robotT[i].wToGo, world.robotT[i].shoot_or_chip, world.robotT[i].kick_power,world.robotT[i].spinBack, rnfi, world, SimulatorMove::robot_speed);
                //  ERF.set_spinBack(false,i);
              //    ERF.set_kick(false,0,i);
               //   ERF.set_velocity_and_W(VecPosition(0,0), 2, i);
              /// world.robotT[i].destination_position=VecPosition(0,0);
                ERF.setAndSend(world.robotT[i].velocityToGo,world.robotT[i].wToGo,world.robotT[i].shoot_or_chip,world.robotT[i].kick_power,world.robotT[i].spinBack,i,world);
               sleep(0.01);


            }
            //ERF.testy();
            //ERF.send_to_ERforce();

            /*auto end_time = std::chrono::high_resolution_clock::now();
            auto time = end_time - start_time;
            cout << time.count() / 1000000.0 << endl;*/
        }
#endif
	};
	auto main_func = [&]()
	{

		sleep(0.005);
		world.team_T.Goalie = 3;
		MatrixD V(4, 1);

		//V(0, 0) = 0;
		//V(1, 0) = 4;
		//V(2, 0) = 4;
		//V(3, 0) = 0;

		//MatrixD q(4, 1);

		//q(0, 0) = 0;
		//q(1, 0) = 0;
		//q(2, 0) = 0;
		//q(3, 0) = 0;
		//while (true)
		//{
		//	if (world.mouseX > 0)
		//	{
		//		nrf::set_velocity(V, 1);
		//		nrf::write_on_port();
		//	}
		//	else
		//	{
		//		nrf::set_velocity(q, 1);
		//		nrf::write_on_port();
		//	}

		//}
		//CWorld = world;
		while (true)
		{
            ///std::cout<<"running...";
			//auto start_time = std::chrono::high_resolution_clock::now();
			//CWorld = world;			//erases everything of CWorld

			///determine wich robot need to get command
			/*for (size_t i = 0; i < world.numT; i++)
			{
				world.robotT[i].send_command = true;
			}*/
			//world.robotT[0].send_command = true;
			//world.robotT[world.getIndexForRobotTNumber(1)].ballBalkMode = Ball::BalkMode::balk;
			///strategy
			/*char id = 2;
			nrf::output[29] = id;
			nrf::output[25] = 0b00100000;
			nrf::write_on_port();*/
           /* for (int i = 0; i < 11; ++i) {
                world.robotT[i].destination_angle=M_PI;
                world.robotT[i].destination_position=world.robotT[i].position;
            }*/
			produceRobotsDestinations();

			///rrt
			producePathsToDestinations();
			//world.robotT[0].pathToDestination[0] = world.robotT[0].destination_position;
			//world.robotT[0].sizeOfPathToDestination = 1;
			//DrawShape::DrawDot(world.robotT[0].destination_position);
			///MC
			produceSpeedOfRobots();
			//Sleep(16);

			//auto end_time = std::chrono::high_resolution_clock::now();
			//auto time = end_time - start_time;
			//cout << time.count() / 1000000.0 << endl;
			//sleep(0.016);
		}

		//#if USE_FEEDFORWARD == 1
//		if (!ftime)
//		{
//			end_time = std::chrono::high_resolution_clock::now();
//			time_auto = end_time - start_time;
//			time_for_feedforward = std::chrono::duration_cast<std::chrono::milliseconds>(time_auto).count();
//		}
//		else
//		{
//			ftime = false;
//			time_for_feedforward = 0;
//		}
//		world.robot_movement_calculated_by_commands[index_for_id][0] = (robot_speed_desired / 304.347)*time_for_feedforward;
//#endif
		//#if USE_FEEDFORWARD == 1
//		world.shift_robot_positions_calculated_by_commands();  //gir
//		world.shift_robot_positions_seen_by_vision();
//
//		world.robot_movement_seen_by_vision[index_for_id][0] = world.robotT[index_for_id].uncorrected_position - previous_position_of_robot;
//		cout << (world.robotT[index_for_id].position - world.robotT[index_for_id].uncorrected_position).getMagnitude() << endl;
//		previous_position_of_robot = world.robotT[index_for_id].uncorrected_position;
//#endif
		//gsm.go(robot_speed_desired, 0, id, world, gsm.wheels_speed);
		//start_time = std::chrono::high_resolution_clock::now();
		//previous_position_of_robot = world.robotT[index_for_id].position;
		//Sleep(3000);
		//while (true)
		//{
		//	//cout << world.robotT[0].pathToDestination<< endl;
		////}
	};



#if DRAW_MATLAB_DIAGRAM == 1
	auto matlab_diagrams = [&]()
	{	
		Sleep(3);

		Matlab first_diagram(0, 1000, 0, 1000 ,1);
		//Matlab second_diagram(0, 1000, -10, 10, 1);
		//Matlab third_diagram(0, 1000, -2000, 2000, 1);
		//Matlab fourth_diagram(0, 1000, -2000, 2000, 1);
		//double k;
		while (true)
		{
			first_diagram.DrawDiagram(world.robotT[0].velocity.getMagnitude());
			//second_diagram.DrawDiagram(world.robotT[0].position.m_y);
		}
	};
#endif
	auto robot_move_manual = [&]()
	{
		sleep(3);
		VFM(0, 0) = 0;
		VFM(1, 0) = 0;
		VFM(2, 0) = 0;
		VFM(3, 0) = 0;
		while (true)
		{
			nrf::convert_robot_velocity_to_wheels_velocity(SFM, 0, 0, VFM);
			nrf::set_velocity(VFM, 1);
			nrf::write_on_port();
		}
	};
	auto robot_wheels_manual = [&]()
	{
		while (true)
		{
			nrf::set_velocity(VFM, 1);
			nrf::write_on_port();
		}
	};
	auto recieve_from_nrf = [&]()
	{

		char input[1000] = {};
		HANDLE hComm;
		DWORD numOfBytesToRead = sizeof(input);
		DWORD numOfBytesRead = 0;
#ifdef _WIN32
        LPCSTR portName = "\\\\.\\COM4";
#elif __linux__
        int serial_port=0;
        LPCSTR portName = "/dev/ttyUSB0";
#endif
		DCB dcbSerialParams = { 0 };


#ifdef _WIN32
		hComm = CreateFile(portName,
			GENERIC_READ | GENERIC_WRITE,
			0,
			NULL,
			OPEN_EXISTING,
			0,
			NULL);

		dcbSerialParams.BaudRate = CBR_256000;
		dcbSerialParams.ByteSize = 8;
		dcbSerialParams.StopBits = ONESTOPBIT;
		dcbSerialParams.Parity = NOPARITY;

		SetCommState(hComm, &dcbSerialParams);


		while (true)
		{
			ReadFile(hComm,
				input,
				numOfBytesToRead,
				&numOfBytesRead,
				NULL);
		}
#elif __linux__
		while(true)
		{
            read(serial_port, &input, numOfBytesToRead);
        }
#endif
	};
	auto motion_control = [&]()
	{
		/*Sleep(4000);
		VecPosition robot_speed;
		MatrixD wheels_speed;
		while (true)
		{
		auto start_time = std::chrono::high_resolution_clock::now();
		robot_speed = PD::GenVecVelPD(1, VecPosition(world.mouseX, world.mouseY), false, 0, world);
		nrf::convert_robot_velocity_to_wheels_velocity(robot_speed, 0, world.robotT[world.getIndexForId(1)].angle,wheels_speed);
		nrf::set_velocity(wheels_speed, 1);
		auto end_time = std::chrono::high_resolution_clock::now();
		auto time = end_time - start_time;
		nrf::write_on_port();
		system("cls");
		int temp = std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000;
		if(16-temp>0)
		Sleep(16 - temp);
		}*/



		//Sleep(5000);
		//VecPosition robot_speed;
		//VecPosition robot_speed_yekke;
		//VecPosition robot_speed_zarbdar_andaze;
		//MatrixD wheels_speed;
		//
		//while (true)
		//{
		//	auto start_time = std::chrono::high_resolution_clock::now();
		//	robot_speed = (VecPosition(world.mouseX, world.mouseY) - world.robotT[world.getIndexForId(1)].position);
		//	/*if (abs(robot_speed.getX()) < 100 && abs(robot_speed.getY()) < 100)
		//	{
		//		robot_speed.setY(0);
		//		robot_speed.setX(0);
		//		robot_speed_yekke.setX(0);
		//		robot_speed_yekke.setY(0);
		//	}
		//	else
		//	{
		//		robot_speed_yekke = robot_speed / sqrt(pow(robot_speed.getX(), 2) + pow(robot_speed.getY(), 2));
		//	}
		//	robot_speed_zarbdar_andaze = robot_speed_yekke * 800;*/
		//	robot_speed_zarbdar_andaze = PD::GenVecVelPD(1, VecPosition(world.mouseX, world.mouseY), false, 0, world);
		//	nrf::convert_robot_velocity_to_wheels_velocity(robot_speed_zarbdar_andaze, 0, world.robotT[world.getIndexForId(1)].angle,wheels_speed);
		//	nrf::set_velocity(wheels_speed, 1);
		//	auto end_time = std::chrono::high_resolution_clock::now();
		//	auto time = end_time - start_time;
		//	nrf::write_on_port();
		//	/*cout << wheels_speed(0, 0) << endl;
		//	cout << wheels_speed(1, 0) << endl;
		//	cout << wheels_speed(2, 0) << endl;
		//	cout << wheels_speed(3, 0);
		//	system("cls");*/
		//	int temp = std::chrono::duration_cast<std::chrono::microseconds>(time).count() / 1000;
		//	if(temp<16)
		//		Sleep(16 - temp);
		//}




		//9/jun/2018

		/*Sleep(5000);
		VecPosition robot_speed;
		VecPosition robot_speed_andaze;
		VecPosition robot_speed_yekke;
		VecPosition robot_speed_zarbdar_andaze;
		VecPosition robot_speed_desired;
		MatrixD V(4,1);
		while (true)
		{
		robot_speed = VecPosition(world.mouseX, world.mouseY) - world.robotT[world.getIndexForId(1)].position;
		while (abs(robot_speed.getX()) > 100 || abs(robot_speed.getY()) > 100)
		{
		robot_speed = VecPosition(world.mouseX, world.mouseY) - world.robotT[world.getIndexForId(1)].position;
		robot_speed_andaze = sqrt(pow(robot_speed.getX(), 2) + pow(robot_speed.getY(), 2));
		robot_speed_yekke = robot_speed / robot_speed_andaze;
		robot_speed_desired = robot_speed_yekke*speed;
		nrf::convert_robot_velocity_to_wheels_velocity(robot_speed_desired, 0, world.robotT[world.getIndexForId(1)].angle, V);
		nrf::set_velocity(V, 1);
		nrf::write_on_port();
		}

		V(0, 0) = 0;
		V(1, 0) = 0;
		V(2, 0) = 0;
		V(3, 0) = 0;
		nrf::set_velocity(V, 1);
		nrf::write_on_port();
		}*/




		/*Sleep(5000);
		VecPosition robot_speed;
		VecPosition robot_speed_andaze;
		VecPosition robot_speed_yekke;
		VecPosition robot_speed_zarbdar_andaze;
		VecPosition robot_speed_desired;
		MatrixD V(4,1);
		while (true)
		{
		robot_speed = VecPosition(world.mouseX, world.mouseY) - world.robotT[world.getIndexForId(1)].position;
		while (sqrt(pow(robot_speed.getX(), 2) + pow(robot_speed.getY(), 2)) > 700)
		{
		robot_speed = VecPosition(world.mouseX, world.mouseY) - world.robotT[world.getIndexForId(1)].position;
		robot_speed_andaze = sqrt(pow(robot_speed.getX(), 2) + pow(robot_speed.getY(), 2));
		robot_speed_yekke = robot_speed / robot_speed_andaze;
		robot_speed_desired = robot_speed_yekke*speed;
		nrf::convert_robot_velocity_to_wheels_velocity(robot_speed_desired, 0, world.robotT[world.getIndexForId(1)].angle, V);
		nrf::set_velocity(V, 1);
		nrf::write_on_port();
		}



		robot_speed = PD::GenVecVelPD(1, VecPosition(world.mouseX, world.mouseY), false, 0, world);
		nrf::convert_robot_velocity_to_wheels_velocity(robot_speed, 0, world.robotT[world.getIndexForId(1)].angle, V);
		nrf::set_velocity(V, 1);
		nrf::write_on_port();

		}*/



		//10/jun/2018-11:09
		//exponential function

#pragma region  "exp"
		/*Sleep(5000);
		VecPosition robot_speed;
		double robot_speed_andaze;
		VecPosition robot_speed_yekke;
		VecPosition robot_speed_zarbdar_andaze;
		VecPosition robot_speed_desired;
		MatrixD V(4,1);

		double MEr=3000;
		double MSp= 600;
		double a = MEr / log(MSp);
		double speed;

		while (true)
		{
		robot_speed = VecPosition(world.mouseX, world.mouseY) - world.robotT[world.getIndexForId(1)].position; //also known as error!
		//sqrt(pow(robot_speed.getX(), 2) + pow(robot_speed.getY(), 2)) > 60
		while (sqrt(pow(robot_speed.getX(), 2) + pow(robot_speed.getY(), 2)) > 60)
		{
		robot_speed = VecPosition(world.mouseX, world.mouseY) - world.robotT[world.getIndexForId(1)].position;
		robot_speed_andaze = sqrt(pow(robot_speed.getX(), 2) + pow(robot_speed.getY(), 2));
		robot_speed_yekke = robot_speed / robot_speed_andaze;
		speed = -exp((-robot_speed_andaze + MEr) / a) + MSp;
		robot_speed_desired = robot_speed_yekke*speed;
		nrf::convert_robot_velocity_to_wheels_velocity(robot_speed_desired, 0, world.robotT[world.getIndexForId(1)].angle, V);
		nrf::set_velocity(V, 1);
		nrf::write_on_port();
		cout << speed<<endl;

		}
		V(0, 0) = 0;
		V(1, 0) = 0;
		V(2, 0) = 0;
		V(3, 0) = 0;
		nrf::set_velocity(V, 1);
		nrf::write_on_port();
		}*/

#pragma region












		//12/jun/2018

		sleep(3);
		VecPosition error_vector;
		VecPosition error_unit_vector;
		VecPosition robot_speed_desired;
		MatrixD V(4, 1);
		double error_vector_size;
		char id = 2;
		double MEr = 750;
		double p = 1; //2
		double MSp = 700;	//700
		double d = MEr / pow(MSp, (1 / p));
		double speed = 0;
		bool careful = true;
		int index_for_id = world.getIndexForRobotTNumber(id);
		bool ftime = true;
		auto start_time = std::chrono::high_resolution_clock::now();
		auto end_time = std::chrono::high_resolution_clock::now();
		auto time_auto = end_time - start_time;
		double time_for_feedforward = 0;

		//SimulatorMove gsm;
		//gsm.initialize_port("192.168.0.3", 20011);
		//gsm.initialize_robot_color_and_timestamp(0.0);

		VecPosition previous_position_of_robot = world.robotT[index_for_id].position;
		index_for_id = world.getIndexForRobotTNumber(id);

		while (true)
		{
			error_vector = VecPosition(world.mouseX, world.mouseY) - world.robotT[index_for_id].position; //also known as error!
																										  //sqrt(pow(error_vector.getX(), 2) + pow(error_vector.getY(), 2)) > 60
			while (/*error_vector.getMagnitude() > 30*/true)
			{
				index_for_id = world.getIndexForRobotTNumber(id);

#if USE_FEEDFORWARD == 1
				world.shift_robot_positions_calculated_by_commands();  //gir
				world.shift_robot_positions_seen_by_vision();

				world.robot_movement_seen_by_vision[index_for_id][0] = world.robotT[index_for_id].uncorrected_position - previous_position_of_robot;
				//cout << (world.robotT[index_for_id].position - world.robotT[index_for_id].uncorrected_position).getMagnitude() << endl;
				previous_position_of_robot = world.robotT[index_for_id].uncorrected_position;
#endif

				error_vector = VecPosition(world.mouseX, world.mouseY) - world.robotT[index_for_id].position;
				error_vector_size = error_vector.getMagnitude();
				/*if (error_vector_size == 0)
				{
				error_unit_vector.setX(0);
				error_unit_vector.setY(0);
				}
				else*/
				error_unit_vector = error_vector / error_vector_size;
				/*double my_test1 = pow(((-(error_vector_size - MEr)) / d), (p));
				double my_test2 = Tools::u(-(error_vector_size - MEr + 1));*/
				if (careful)	///g function
					speed = pow((error_vector_size / d), p)*Tools::u(-(error_vector_size - MEr + 1)) + MSp*Tools::u(error_vector_size - MEr);
				else    ///f function
					speed = (-pow(((sign(error_vector_size - MEr)*(error_vector_size - MEr)) / d), p) + MSp)*Tools::u(-(error_vector_size - MEr + 1)) + MSp*Tools::u(error_vector_size - MEr);

				/*if (speed > MSp)
				continue;*/

				///changing for grsim
				/*nrf::convert_robot_velocity_to_wheels_velocity(robot_speed_desired, 0, world.robotT[world.getIndexForId(1)].angle, V);
				nrf::set_velocity(V, 1);
				nrf::write_on_port();*/

				if (!ftime)
				{
					end_time = std::chrono::high_resolution_clock::now();
					time_auto = end_time - start_time;
					time_for_feedforward = std::chrono::duration_cast<std::chrono::milliseconds>(time_auto).count();
				}
				else
				{
					ftime = false;
					time_for_feedforward = 0;
				}
				//cout << time_for_feedforward << endl;
#if USE_FEEDFORWARD == 1
				world.robot_movement_calculated_by_commands[index_for_id][0] = (robot_speed_desired / 304.347)*time_for_feedforward;
#endif
				robot_speed_desired = error_unit_vector*speed;
				/*gsm.go(robot_speed_desired, 0, id, world, gsm.wheels_speed);*/
				nrf::go(robot_speed_desired, 2, 2, world, 0);
				start_time = std::chrono::high_resolution_clock::now();

				world.exactSleep(16);
			}
			ftime = true;
			robot_speed_desired = VecPosition(0, 0);

			///changing for grsim
			/*V(0, 0) = 0;
			V(1, 0) = 0;
			V(2, 0) = 0;
			V(3, 0) = 0;
			nrf::set_velocity(V, id);
			nrf::write_on_port();*/

			/*gsm.go(robot_speed_desired, 0, 1, world, gsm.wheels_speed);*/
			nrf::go(robot_speed_desired, 2, 1, world, 0);

#if USE_FEEDFORWARD == 1
			world.shift_robot_positions_calculated_by_commands();  //gir
			world.shift_robot_positions_seen_by_vision();

			world.robot_movement_seen_by_vision[index_for_id][0] = world.robotT[index_for_id].uncorrected_position - previous_position_of_robot;
			previous_position_of_robot = world.robotT[index_for_id].uncorrected_position;

			world.robot_movement_calculated_by_commands[index_for_id][0] = VecPosition(0, 0);
#endif
			world.exactSleep(16);

		}


	};
	auto turn_on_led = [&]()
	{
		/*Sleep(3000);
		int sop;
		while (true)
		{
		cout << RRT::MakeRRT(1, VecPosition(world.mouseX, world.mouseY), world.robotT[0].sizeOfPathToDestination, world, true, true, true, false)[1] << endl;
		}*/

		while (true)
		{
			for (int i = 0; i<180; i++)
				nrf::output[i] = 1;
			nrf::write_on_port();
		}
	};
	auto test_func = [&]()
	{

		//Sleep(6000);
		/*while (true)
		{
			nrf::output[29] = 1;
			nrf::output[25] = 0b11100000;
			nrf::write_on_port();
		}*/
		//MatrixD V(4, 1);
		//bool flag = true;
		//while(true)
		//cout << Field::isInBehindGoal(MOUSE_AS_VECPOSITION);

		while (true)
		{
			//if (flag == false)
			//{
			//	/*V(0, 0) = 0;
			//	V(1, 0) = 0;
			//	V(2, 0) = 0;
			//	V(3, 0) = 0;*/
			//	nrf::go(VecPosition(0, 0), 2);
			//	flag = true;
			//}
			//else
			//{
			//	/*V(0, 0) = -1;
			//	V(1, 0) = -1;
			//	V(2, 0) = 1;
			//	V(3, 0) = 1;*/
			//	nrf::go(VecPosition(100, 0), 2);
			//	flag = false;
			//}
			////nrf::set_velocity(V, 2);
			////nrf::write_on_port();
			//Sleep(700);


			//nrf::go(VecPosition(100, 0),1);

			MatrixD V(4, 1);
			V(0, 0) = -3;
			V(1, 0) = -3;
			V(2, 0) =-3 ;
			V(3, 0) = -3;
			nrf::set_velocity(V, 1);
			nrf::write_on_port();
            sleep(0.1);
       //     cout<<"hi\n";
			//cout << world.robotT[0].w << endl;

			//Paraline pl = Paraline(MOUSE_AS_VECPOSITION, VecPosition(0, 0));
			//DrawShape::DrawParaline(pl);
			//DrawShape::DrawParaline(Field::getRightParaline_LeftPenaltyArea());

		}

	};
	auto send_nrf_temp = [&]()
	{
		nrf::output[0] = 'h';
		nrf::output[1] = 'e';
		nrf::output[2] = 'l';
		nrf::output[3] = 'l';
		nrf::output[4] = 'o';
		while(true)
			nrf::write_on_port();
	};
#pragma region "commented threads"

	/*auto APP_func = [&]()
	{
		ghInst = hInstance;

		WNDCLASSEX ex;

		ex.cbSize = sizeof(WNDCLASSEX);
		ex.cbClsExtra = 0;
		ex.cbWndExtra = 0;
		ex.hInstance = ghInst;
		ex.style = 0;
		ex.lpszMenuName = NULL;
		ex.hbrBackground = CreateSolidBrush(RGB(240, 255, 240));  //To Change Background Color
		ex.hCursor = LoadCursor(NULL, IDC_ARROW);
		ex.hIcon = LoadIcon(NULL, IDI_APPLICATION);
		ex.hIconSm = LoadIcon(NULL, IDI_APPLICATION);
		ex.lpszClassName = lpClassName;
		ex.style = NULL;
		ex.lpfnWndProc = WndProc;

		RegisterClassEx(&ex);

		HWND hwnd = CreateWindowEx(NULL, lpClassName, "omidguard",
			WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
			0, 0, 1500, 750, NULL, NULL, ghInst, NULL);

		ShowWindow(hwnd, nShowCmd);


		MSG msg;

		while (GetMessage(&msg, NULL, 0, 0))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}

		exit(0);
	};*/

	//auto cal_robot_speed = [&]()
	//{
	//	Sleep(3000);
	//	int i = 0;
	//	//int j = 0;
	//	while (true)
	//	{
	//		if (i == 0)
	//		{
	//			World::previous_vision_time = world.robotT[0].timeCaptured;
	//			World::previous_robot_location.setX(world.robotT[0].position.getX());
	//			World::previous_robot_location.setY(world.robotT[0].position.getY());
	//			i++;
	//			/*while (j != 10)
	//			{
	//				if ((world.robotT[0].timeCaptured - World::previous_vision_time) >= 0.015)
	//				{
	//					j++;
	//					World::previous_robot_location += world.robotT[0].position;
	//				}
	//			}
	//			World::previous_robot_location /= j;
	//			World::previous_vision_time = world.robotT[0].timeCaptured;
	//			j = 0;*/
	//		}
	//
	//		if (i != 0 && (world.robotT[0].timeCaptured - World::previous_vision_time) >= 0.016)
	//		{
	//			World::previous_robot_speed = World::robot_speed;
	//			World::robot_speed.setX(((world.robotT[0].position.getX() - World::previous_robot_location.getX()) / (world.robotT[0].timeCaptured - World::previous_vision_time)));
	//			World::robot_speed.setY((world.robotT[0].position.getY() - World::previous_robot_location.getY()) / (world.robotT[0].timeCaptured - World::previous_vision_time));
	//			World::previous_vision_time = world.robotT[0].timeCaptured;
	//			World::previous_robot_location = world.robotT[0].position;
	//			World::ROBOT_speeds[0][0] = (3 * (World::robot_speed.getX())) / 700.000;
	//			World::ROBOT_speeds[0][1] = (3 * (World::robot_speed.getY())) / 700.000;
	//			PD::shift_ROBOT_speeds();
	//		}
	//	}
	//};

	/*auto test_go = [&]()
	{
	Sleep(4);
	int i = 0;

	VecPosition front;
	front.setX(500);
	front.setY(0);

	VecPosition back;
	back.setX(-300);
	back.setY(0);

	while (true)
	{
	while (i < 60)
	{
	PD::shift_AI_speeds();
	PD::shift_ROBOT_speeds();

	nrf::go(front,1);

	World::AI_speeds[0][0] = 3 * (front.getX() / 2000.000);
	World::AI_speeds[0][1] = 3 * (front.getY() / 2000.000);
	World::ROBOT_speeds[0][0] = (3 * (World::robot_speed.getX())) / 700.000;
	World::ROBOT_speeds[0][1] = (3 * (World::robot_speed.getY())) / 700.000;

	i++;

	Sleep(0.016);
	}

	while (i > 0)
	{
	PD::shift_AI_speeds();
	PD::shift_ROBOT_speeds();

	nrf::go(back, 1);

	World::AI_speeds[0][0] = 3 * (back.getX() / 2000.000);
	World::AI_speeds[0][1] = 3 * (back.getY() / 2000.000);
	World::ROBOT_speeds[0][0] = (3 * (World::robot_speed.getX())) / 700.000;
	World::ROBOT_speeds[0][1] = (3 * (World::robot_speed.getY())) / 700.000;

	i--;

	Sleep(0.016);
	}
	}
	};*/

	/*auto test_rrt = [&]()
	{
	Cartesian_Coordinates destination;
	int a;
	double previous_mouse_x = 2000;
	double previous_mouse_y = 2000;

	Balk::set_defualt_penalty_area();

	while (true)
	{
	if (world.mouseX != previous_mouse_x || world.mouseY != previous_mouse_y)
	{
	destination.x = world.mouseX;
	destination.y = world.mouseY;
	RRT::MakeRRT(1, world, destination,a,true,true,true,false);
	previous_mouse_x = world.mouseX;
	previous_mouse_y = world.mouseY;
	}
	}
	};*/

#pragma endregion
	
	std::thread refree_thread(refree_func);
	std::thread vision_thread(vision_func);
#if GLUT_ENABLE
	std::thread glut_thread(GLUT_func);
#endif
	std::thread radio_thread(radio_func);
	//.std::thread Send_Nrf_Temp(send_nrf_temp);

#ifdef w_manual
	std::thread Robot_Wheels_Manual(robot_wheels_manual);
#elif defined m_manual
	std::thread Robot_Move_Manual(robot_move_manual);
#else
	std::thread main_thread(main_func);
#endif

#if DRAW_MATLAB_DIAGRAM == 1
	std::thread Matlab_Diagrams(matlab_diagrams);
#endif

	//std::thread test_thread(test_func);
	//std::thread Recieve_From_Nrf(recieve_from_nrf);
	//std::thread app_thread(APP_func);
	//std::thread Turn_On_LED(turn_on_led);
	//std::thread MC(motion_control);


	refree_thread.join();
	vision_thread.join();
#if GLUT_ENABLE
	glut_thread.join();
#endif
	radio_thread.join();
	///Send_Nrf_Temp.join();

#ifdef w_manual
	Robot_Wheels_Manual.join();
#elif defined m_manual
	Robot_Move_Manual.join();
#else
	main_thread.join();
#endif

#if DRAW_MATLAB_DIAGRAM == 1
	Matlab_Diagrams.join();
#endif

	//Recieve_From_Nrf.join();
	//Turn_On_LED.join();
	//MC.join();
	//test_thread.join();
}