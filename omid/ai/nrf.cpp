#include "nrf.h"
#include <iostream>

#ifdef _WIN32
#include <io.h>
   #include <Windows.h>
#elif __linux__
#include <inttypes.h>
// C library headers
#include <stdio.h>
#include <string.h>
// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
///#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <stdlib.h>
#include <stdint.h>     /* uint32_t */
#include <sys/stat.h>


#define __int64 int64_t
#define _close close
#define _read read
#define _lseek64 lseek64
#define _O_RDONLY O_RDONLY
#define _open open
#define _lseeki64 lseek64
#define _lseek lseek
#define stricmp strcasecmp
#include <stdlib.h>
#include <math.h>
#include <GL/gl.h>
#include <GL/glu.h>
//#include <GL/glx.h>
#include <X11/Xlib.h>
// #include "myheader.h"    // can no longer use windows.h or conio.h
// #include "myheader2.h"
#endif
#include "math.h"
#include "matrix.h"
#include <chrono>
#include <thread>
#include "geometry.h"
#include "world.h"
#include <random>
#include "RRT.h"
#include "Socket_udp.h"
#include "Switches.h"

#if SEND_COMMANDS_TO_ROBOTS==2

#include "Protobuf/ER-force/ssl_simulation_control.pb.h"
#else
#include "Protobuf/Grsim/grSim_Packet.pb.h"
#endif
#include <string>
#include "Switches.h"
///convert arguments of functions to const World &world


/*! NRF VARIABLES */
char nrf::output[180] = {};
char nrf::input[1000] = {};
char nrf::ID;
//MatrixD V(4, 1);	//velocity matrix
HANDLE nrf::hComm;
DWORD nrf::numOfBytesToWrite = sizeof(output);
DWORD nrf::numOfBytesWritten = 0;
DWORD nrf::numOfBytesToRead = sizeof(input);
DWORD nrf::numOfBytesRead = 0;
#ifdef _WIN32
LPCSTR nrf::portName = "\\\\.\\COM4";
#elif __linux__
int serial_port=0;
LPCSTR nrf::portName = "/dev/ttyUSB1";
#endif
DCB nrf::dcbSerialParams = { 0 };
MatrixD nrf::V(4, 1);
bool nrf::statusNrf = false;
bool nrf::ftime = true;
/*! NRF VARIABLES */
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
//#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls

int fd; // file description for the serial port


/*! GrsimMove VARIABLES */

/*! GrsimMove VARIABLES */




/*!NRF METHODS */
void nrf::go(VecPosition vp, char id)
{
	MatrixD V(4, 1);
	convert_robot_velocity_to_wheels_velocity(vp, 0, 0, V);
	set_velocity(V, id);
	write_on_port();
};

void nrf::go(VecPosition vp, int robot_number, char id, World world, double ww)
{
	
	//SetConsoleCtrlHandler((PHANDLER_ROUTINE)(ctrl_handler), true);
	for (int i = 0; i < world.numT; i++)
	{
		if (world.robotT[i].id == robot_number)
		{
			convert_robot_velocity_to_wheels_velocity(vp, ww, world.robotT[i].angle, V);
		}
	}
	set_velocity(V, id);
	write_on_port();
}
void nrf::go_withoutSend(VecPosition vp, double ww, int robot_number, char id, World world)
{
	//SetConsoleCtrlHandler((PHANDLER_ROUTINE)(ctrl_handler), true);
	for (int i = 0; i < world.numT; i++)
	{
		if (world.robotT[i].id == robot_number)
		{
			convert_robot_velocity_to_wheels_velocity(vp, ww, world.robotT[i].angle, V);
		}
	}
	set_velocity(V, id);
}

///for real robots
void nrf::convert_robot_velocity_to_wheels_velocity(VecPosition vv, double ww, AngRad rteta, MatrixD &V_send_out)
{
	//ww -> charkhesh hole mehvar khod.

	MatrixD A(4, 3), V(4, 1), VW1(2, 1), VW(3, 1);

	double R = 90;
	VW(0, 0) = (vv.getX() / 64.5);
	VW(1, 0) = (vv.getY() / 64.5);
	VW(2, 0) = ww;// (ww / 180);

	double modified_coeficient = 1;//0.68
	double bb0 = Deg2Rad(300), bb1 = Deg2Rad(225), bb2 = Deg2Rad(135), bb3 = Deg2Rad(60);

	for (int k = 0; k < 2; k++)
	{
		A(0, 0) = (cos(rteta)*sin(bb0) + sin(rteta)*cos(bb0));     A(0, 1) = (sin(rteta)*sin(bb0) - cos(rteta)*cos(bb0));    A(0, 2) = -R;
		A(1, 0) = (cos(rteta)*sin(bb1) + sin(rteta)*cos(bb1));     A(1, 1) = (sin(rteta)*sin(bb1) - cos(rteta)*cos(bb1));    A(1, 2) = -R;
		A(2, 0) = (cos(rteta)*sin(bb2) + sin(rteta)*cos(bb2));     A(2, 1) = (sin(rteta)*sin(bb2) - cos(rteta)*cos(bb2));    A(2, 2) = -R;
		A(3, 0) = (cos(rteta)*sin(bb3) + sin(rteta)*cos(bb3));     A(3, 1) = (sin(rteta)*sin(bb3) - cos(rteta)*cos(bb3));    A(3, 2) = -R;

		/*MatrixD rotation(3, 3), B(4, 3);
		rotation(0, 0) = cos(rteta); = -sin(rteta ); rotation(0, 2) = 0;
		rotation(1, 0) = sin(rteta); rotation(1, 1) = cos(rteta ); rotation(1, 2) = 0;
		rotation(2, 0) = 0; rotation(2, 1) = 0; rotation(2, 2) = 1;
		double pi = M_PI;
		B(0, 0) = (-0.5 / (cos(pi / 3) + cos(pi / 4)));     B(0, 1) = (0.5 / (sin(pi / 3) + sin(pi / 4)));   B(0, 2) = cos(pi / 4) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
		B(1, 0) = (0.5 / (cos(pi / 3) + cos(pi / 4)));      B(1, 1) = (0.5 / (sin(pi / 3) + sin(pi / 4)));    B(1, 2) = cos(pi / 3) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
		B(2, 0) = (0.5 / (cos(pi / 3) + cos(pi / 4)));     B(2, 1) = (-0.5 / (sin(pi / 3) + sin(pi / 4)));   B(2, 2) = cos(pi / 3) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
		B(3, 0) = (-0.5 / (cos(pi / 3) + cos(pi / 4)));    B(3, 1) = (-0.5 / (sin(pi / 3) + sin(pi / 4)));  B(3, 2) = cos(pi / 4) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
		A = B*rotation;*/

		V = modified_coeficient* A * VW;

		double limit_v = 30;
		double swap_v;
		swap_v = limit_v;
		if (fabs(V(0, 0)) > limit_v || fabs(V(1, 0)) > limit_v || fabs(V(2, 0)) > limit_v || fabs(V(3, 0)) > limit_v)
		{

			for (int i = 0; i < 4; i++)
			{
				if (fabs(V(i, 0)) > swap_v)
				{
					swap_v = fabs(V(i, 0));
				}
			}
			modified_coeficient = (limit_v) / (swap_v);

		}
	}

	for (int m = 0; m < 4; m++)
	{
		V(m, 0) = floor((ceil(2 * V(m, 0))) / 2);
	}
	V_send_out = V;
}
void nrf::set_velocity(MatrixD V, char id)
{
	char data[3] = {};	//wheel velocity
	/*cout << V(0, 0) << endl;
	cout << V(1, 0) << endl;
	cout << V(2, 0) << endl;
	cout << V(3, 0) << endl;*/
	data[0] = (((int)V(0, 0) & 0x3F) << 2) | (((int)V(1, 0) & 0x30) >> 4);
	data[1] = (((int)V(1, 0) & 0x0F) << 4) | (((int)V(2, 0) & 0x3C) >> 2);
	data[2] = (((int)V(2, 0) & 0x03) << 6) | (((int)V(3, 0) & 0x3F) << 0);

	output[30 - 1] = id;
	output[30 - 2] = data[0];
	output[30 - 3] = data[1];
	output[30 - 4] = data[2];
	
}
void nrf::write_on_port() {


    if (ftime) {
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

        statusNrf = SetCommState(hComm, &dcbSerialParams);
        ftime = false;
#elif __linux__
        serial_port = open("/dev/ttyUSB1", O_RDWR| O_NOCTTY/*O_RDWR | O_NOCTTY | O_NDELAY*/);
        sleep(1);
        close(serial_port);
        sleep(1);
        serial_port = open("/dev/ttyUSB1", O_RDWR| O_NOCTTY/*O_RDWR | O_NOCTTY | O_NDELAY*/);
        if(serial_port == -1) // if open is unsucessful
        {
            printf("open_port: Unable to open /dev/ttyUSB1. \n");
        }
        else
        {
            fcntl(serial_port, F_SETFL, 0);
            printf("port is open.\n");
        }
   /*     struct termios2 tty;      // structure to store the port settings in
        ioctl(serial_port, TCGETS2, &tty);

        //================= (.c_cflag) ===============//

        tty.c_cflag     &=  ~PARENB;           // No parity bit is added to the output characters
        tty.c_cflag     &=  ~CSTOPB;        // Only one stop-bit is used
        //tty.c_cflag     &=  ~CSIZE;            // CSIZE is a mask for the number of bits per character
        tty.c_cflag     |=  CS8;            // Set to 8 bits per character
        //tty.c_cflag     &=  ~CRTSCTS;       // Disable hadrware flow control (RTS/CTS)
        //tty.c_cflag     |=  CREAD | CLOCAL;
        tty.c_cflag &= ~CBAUD;
        tty.c_cflag |= CBAUDEX;
        //  tty.c_cflag |= BOTHER;
        tty.c_ispeed = 256000;
        tty.c_ospeed = 256000;
        //  tty.c_oflag     =   0;              // No remapping, no delays
        //  tty.c_oflag     &=  ~OPOST;            // Make raw
        ioctl(serial_port, TCSETS2, &tty);*/
        ftime = false;
#endif


    } else {
#ifdef _WIN32
        WriteFile(hComm,
            output,
            numOfBytesToWrite,
            &numOfBytesWritten,
            NULL);
#elif __linux__

#endif
        output[1]=1;
        output[2]=1;
        output[3]=1;
        output[4]=1;
        for (int i = 0; i < 180; ++i) {
           output[i] = 1;
        }
        write(fd, output, sizeof(output));  //Send data
       printf("Wrote the bytes. \n");
    }
}
void nrf::read_from_port()
{
#ifdef _WIN32
    ReadFile(hComm,
		input,
		numOfBytesToRead,
		&numOfBytesRead,
		NULL);
#elif __linux__
  //  read(serial_port, &input, numOfBytesToRead);
#endif

}

/*!NRF METHODS */

#if SEND_COMMANDS_TO_ROBOTS==2
SimulatorMove::SimulatorMove()
{
    for(int i=0;i<MAX_ROBOTS_PER_TEAM_IN_THE_FIELD;i++)
    {
        command[i] = packet.add_robot_commands();
    }
}
void SimulatorMove::initialize_port(const char * ip, int port)
{
    ERforce.Init_Socket_Server(ip, port); //"192.168.0.255"
}
void SimulatorMove::closeUDP()
{
    ERforce.Close_Socket();
}
void SimulatorMove::go(VecPosition vv, double w, char id, World world, move_type mt)
{
    int index_for_id = world.getIndexForRobotTNumber(id);

    command[id]->set_id(id);
    if (mt == wheels_speed)
    {
        MatrixD V;
        convert_robot_velocity_to_wheels_velocity(vv, w, world.robotT[index_for_id].angle, V);
        set_wheels_velocity(V, id);
        send_to_ERforce();
    }

    else
    {
        VecPosition velocity;
        velocity = convert_robot_velocity_from_field_to_robot_coord(vv, world.robotT[index_for_id].angle);
        set_velocity_and_W(velocity, w, id);
        send_to_ERforce();
    }


}


///shootOrChip == 1 for shoot, shootOrChip == 0 for chip
void SimulatorMove::go_setKick_setSpinBack_withoutSend(VecPosition vv, double w, bool shootOrChip, short int kickPower, bool spinBack, char id, World world, move_type mt)
{
    int index_for_id = world.getIndexForRobotTNumber(id);

    command[id]->set_id(id);
    if (mt == wheels_speed)
    {
        MatrixD V;
        convert_robot_velocity_to_wheels_velocity(vv, w, world.robotT[index_for_id].angle, V);
        set_wheels_velocity(V, id);
        set_kick(shootOrChip, kickPower, id);
        set_spinBack(spinBack, id);
    }

    else
    {
        VecPosition velocity;
        velocity = convert_robot_velocity_from_field_to_robot_coord(vv, world.robotT[index_for_id].angle);
        set_velocity_and_W(velocity, w, id);
        set_kick(shootOrChip, kickPower, id);
        set_spinBack(spinBack, id);
    }
}



VecPosition SimulatorMove::convert_robot_velocity_from_field_to_robot_coord(VecPosition RFV , AngRad rteta)
{
    return VecPosition(RFV.InerMultiply(VecPosition::directVector(rteta)), RFV.InerMultiply(VecPosition::directVector(rteta + (M_PI / 2.0))));
}

void SimulatorMove::set_wheels_velocity(MatrixD V,char id)
{
    command[id]->mutable_move_command()->mutable_wheel_velocity()->set_back_left((float)-V(2, 0));
    command[id]->mutable_move_command()->mutable_wheel_velocity()->set_back_right((float)-V(1, 0));
    command[id]->mutable_move_command()->mutable_wheel_velocity()->set_front_left((float)-V(3, 0));
    command[id]->mutable_move_command()->mutable_wheel_velocity()->set_front_right((float)-V(0, 0));
}
void SimulatorMove::set_velocity_and_W(VecPosition velocity, double w, char id)
{
    command[id]->mutable_move_command()->mutable_local_velocity()->set_forward(velocity.getX()/300.000);
    command[id]->mutable_move_command()->mutable_local_velocity()->set_left(velocity.getY()/300.000);
    command[id]->mutable_move_command()->mutable_local_velocity()->set_angular(w);
}

///shootOrChip == 0 for shoot, shootOrChip == 1 for chip
void SimulatorMove::set_kick(bool shootOrChip, short int kickPower, char id)
{
    if (shootOrChip == 0)
    {
        command[id]->set_kick_speed(3 / 2.0*kickPower);
        command[id]->set_kick_angle(0);
    }
    else if (shootOrChip == 1)
    {
        command[id]->set_kick_angle(45);
        command[id]->set_kick_speed(3 / 2.0*kickPower*0.70710);	/// ...*cos(M_PI/4)
        ///command[id]->set_kick_speed(3 / 2.0*kickPower*0.70710); /// ...*sin(M_PI/4)
    }
}

void SimulatorMove::set_spinBack(bool set,char id)
{
    if(set)
        command[id]->set_dribbler_speed(10);
    else
        command[id]->set_dribbler_speed(0);
}
void SimulatorMove::convert_robot_velocity_to_wheels_velocity(VecPosition RV, double w, AngRad rteta, MatrixD &V_send_out)
{
    ///ww -> charkhesh hole mehvar khod.

    MatrixD A(4, 3), V(4, 1), VW1(2, 1), VW(3, 1);

    double R = 90;
    VW(0, 0) = (RV.getX()/7.5 /* / 64.5*/);
    VW(1, 0) = (RV.getY()/7.5 /* / 64.5*/);

    VW(2, 0) = w/27; //(ww / 180) 	///alireza : w/27

    double modified_coeficient = 1;//0.68
    //double bb0 = Deg2Rad(315), bb1 = Deg2Rad(225), bb2 = Deg2Rad(135), bb3 = Deg2Rad(45);	//for grsim
    double bb0 = Deg2Rad(300), bb1 = Deg2Rad(225), bb2 = Deg2Rad(135), bb3 = Deg2Rad(60);	//for omid robots

    for (int k = 0; k < 2; k++)
    {
        A(0, 0) = (cos(rteta)*sin(bb0) + sin(rteta)*cos(bb0));     A(0, 1) = (sin(rteta)*sin(bb0) - cos(rteta)*cos(bb0));    A(0, 2) = -R;
        A(1, 0) = (cos(rteta)*sin(bb1) + sin(rteta)*cos(bb1));     A(1, 1) = (sin(rteta)*sin(bb1) - cos(rteta)*cos(bb1));    A(1, 2) = -R;
        A(2, 0) = (cos(rteta)*sin(bb2) + sin(rteta)*cos(bb2));     A(2, 1) = (sin(rteta)*sin(bb2) - cos(rteta)*cos(bb2));    A(2, 2) = -R;
        A(3, 0) = (cos(rteta)*sin(bb3) + sin(rteta)*cos(bb3));     A(3, 1) = (sin(rteta)*sin(bb3) - cos(rteta)*cos(bb3));    A(3, 2) = -R;

        /*MatrixD rotation(3, 3), B(4, 3);
        rotation(0, 0) = cos(rteta); rotation(0, 1) = -sin(rteta ); rotation(0, 2) = 0;
        rotation(1, 0) = sin(rteta); rotation(1, 1) = cos(rteta ); rotation(1, 2) = 0;
        rotation(2, 0) = 0; rotation(2, 1) = 0; rotation(2, 2) = 1;
        double pi = M_PI;
        B(0, 0) = (-0.5 / (cos(pi / 3) + cos(pi / 4)));     B(0, 1) = (0.5 / (sin(pi / 3) + sin(pi / 4)));   B(0, 2) = cos(pi / 4) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
        B(1, 0) = (0.5 / (cos(pi / 3) + cos(pi / 4)));      B(1, 1) = (0.5 / (sin(pi / 3) + sin(pi / 4)));    B(1, 2) = cos(pi / 3) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
        B(2, 0) = (0.5 / (cos(pi / 3) + cos(pi / 4)));     B(2, 1) = (-0.5 / (sin(pi / 3) + sin(pi / 4)));   B(2, 2) = cos(pi / 3) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
        B(3, 0) = (-0.5 / (cos(pi / 3) + cos(pi / 4)));    B(3, 1) = (-0.5 / (sin(pi / 3) + sin(pi / 4)));  B(3, 2) = cos(pi / 4) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
        A = B*rotation;*/

        V = modified_coeficient* A * VW;

        /*double limit_v = 30;
        double swap_v;
        swap_v = limit_v;
        if (fabs(V(0, 0)) > limit_v || fabs(V(1, 0)) > limit_v || fabs(V(2, 0)) > limit_v || fabs(V(3, 0)) > limit_v)
        {

            for (int i = 0; i < 4; i++)
            {
                if (fabs(V(i, 0)) > swap_v)
                {
                    swap_v = fabs(V(i, 0));
                }
            }
            modified_coeficient = (limit_v) / (swap_v);

        }*/
    }


    /*for (int m = 0; m < 4; m++)
    {
        V(m, 0) = (V(m, 0) / max_wheel_speed) * 32.000;
    }*/

    for (int m = 0; m < 4; m++)
    {
        V(m, 0) = floor((ceil(2 * V(m, 0))) / 2);
    }
    /*for (int m = 0; m < 4; m++)
    {
        V(m, 0) *= max_wheel_speed/32.000;
    }*/
    V_send_out = V;
}

void SimulatorMove::send_to_ERforce()
{
    char _data[packet.ByteSize()];
    packet.SerializePartialToArray(s_data, sizeof(s_data));
    ERforce.send(s_data, sizeof(s_data));
}



void SimulatorMove::setAndSend(VecPosition velocity, double w, bool shootOrChip, short int kickPower, bool spinBack,
                               int index,World world)
{
    int id=world.getRobotTNumberForIndex(index);
    auto control = RobotControl();
    auto *robotCommand = control.add_robot_commands();
    robotCommand->set_id(id);
    if(world.robotT[index].kick_power>0) {
        robotCommand->set_dribbler_speed(0); // convert from 1 - 0 to rpm, where 1 is 150 rad/s
        robotCommand->set_kick_speed((shootOrChip)?(int)(3 / 2.0*kickPower):(int)(3 / 2.0*kickPower*0.70710));
    } else {
        robotCommand->set_dribbler_speed(1); // convert from 1 - 0 to rpm, where 1 is 150 rad/s
        robotCommand->set_kick_speed(0);
    }
    robotCommand->set_kick_angle((shootOrChip)?0:45);
/*
    robotCommand->set_dribbler_speed((spinBack)?1:0); // convert from 1 - 0 to rpm, where 1 is 150 rad/s
*/

    auto *moveCommand = robotCommand->mutable_move_command()->mutable_local_velocity();

    VecPosition velocityLocal;
    velocityLocal = convert_robot_velocity_from_field_to_robot_coord(velocity, world.robotT[index].angle);
    moveCommand->set_forward(velocityLocal.getX()/300.000);
    moveCommand->set_left(velocityLocal.getY()/300.000);
    moveCommand->set_angular(w*2);
   /// if(id==10)
   /// cout<<robotCommand->id()<<" : "<<robotCommand->kick_speed()<<" : "<<robotCommand->dribbler_speed()<<" : "<<robotCommand->kick_angle()<<"\n";

    char _data[control.ByteSize()];
    control.SerializePartialToArray(_data, sizeof(_data));
    ERforce.send(_data, sizeof(_data));

}
void SimulatorMove::testy()
{
//packet.mutable_robot_commands(1)->mutable_move_command()->mutable_wheel_velocity()->set_front_right(-2);
  //  packet.mutable_robot_commands(1)->mutable_move_command()->mutable_wheel_velocity()->set_front_left(2);
    //packet.robot_commands().Add(command[0],command[7]);
   // command[0] = packet.add_robot_commands();
/*
   RobotCommand hello;
   RobotMoveCommand movehello;
   MoveWheelVelocity whellhello;
   whellhello.set_front_left(-4);
   whellhello.set_front_right(4);
   hello.set_id(1);
   movehello.set_allocated_wheel_velocity(&whellhello);
   hello.set_allocated_move_command(&movehello);
   hello.SerializePartialToArray(s_data,sizeof(s_data));

*/


  /*  command[0] = packet.mutable_robot_commands()->Add();
    command[0]->set_id(1);
    command[0]->set_kick_speed(0);
    command[0]->set_kick_angle(0);
    command[0]->set_dribbler_speed(0);
//    set_spinBack(command[0],0);
    command[0]->mutable_move_command()->mutable_local_velocity()->set_forward(5);
    command[0]->mutable_move_command()->mutable_local_velocity()->set_angular(0.5);
    command[0]->mutable_move_command()->mutable_local_velocity()->set_left(0);*/


//Robot
  /* auto control =RobotControl();
//    control.default_instance();
        auto* robotCommand = control.add_robot_commands();
      robotCommand->set_id(5);
      robotCommand->set_kick_speed(0);
      robotCommand->set_kick_angle(45);
      robotCommand->set_dribbler_speed(0 ); // convert from 1 - 0 to rpm, where 1 is 150 rad/s
      auto* moveCommand  = robotCommand->mutable_move_command()->mutable_local_velocity();
      moveCommand->set_forward(0);
      moveCommand->set_left(0);
      moveCommand->set_angular(50);*/
  /* for (int i = 0; i <=10; ++i) {
        command[i]->set_id(i);
        command[i]->mutable_move_command()->mutable_local_velocity()->set_forward(-2);
        command[i]->mutable_move_command()->mutable_local_velocity()->set_left(2);
        command[i]->mutable_move_command()->mutable_local_velocity()->set_angular(1);
        command[i]->set_kick_speed(0);
        command[i]->set_dribbler_speed(0);
        command[i]->set_kick_angle(45);
    }*/
 //   auto control =RobotControl();
  //  control.mutable_robot_commands()->AddAllocated(command[1]);


/*    auto control =RobotControl();
//    control.default_instance();
    auto* robotCommand = control.add_robot_commands();
    robotCommand->set_id(5);
    robotCommand->set_kick_speed(0);
    robotCommand->set_kick_angle(45);
    robotCommand->set_dribbler_speed(0 ); // convert from 1 - 0 to rpm, where 1 is 150 rad/s
    auto* moveCommand  = robotCommand->mutable_move_command()->mutable_local_velocity();
    moveCommand->set_forward(0);
    moveCommand->set_left(0);
    moveCommand->set_angular(50);*/

   /*   char _data[packet.ByteSize()];
    cout<<packet.ByteSize()<<'\n';
    packet.SerializePartialToArray(_data,sizeof(_data));
     ERforce.send(_data,sizeof(_data));*/


    for (int i = 0; i < 11; ++i) {
        auto control = RobotControl();
        auto *robotCommand = control.add_robot_commands();
        robotCommand->set_id(i);
        robotCommand->set_kick_speed(0);
        robotCommand->set_kick_angle(45);
        robotCommand->set_dribbler_speed(0); // convert from 1 - 0 to rpm, where 1 is 150 rad/s
        auto *moveCommand = robotCommand->mutable_move_command()->mutable_local_velocity();
        moveCommand->set_forward(2);
        moveCommand->set_left(0);
        moveCommand->set_angular(4);
        char _data[control.ByteSize()];
        control.SerializePartialToArray(_data, sizeof(_data));
        ERforce.send(_data, sizeof(_data));
    }
    cout<<"____------_____-----\n";
   //cout<<"____------_____-----\n";
 //   boost::asio::streambuf b;
 //   std::ostream a(&b);
 //   control.SerializeToOstream(&a);
    //const char* header=boost::asio::buffer_cast<const char*>(b.data());
//    cout<<sizeof(s_data)<<'\n';
/*    std::string test;
    control.SerializeToString(&test);*/
 //ERforce.send2ERforce((string *) &header, b.data().size());






//  ERforce.send2ERforce(&test,sizeof (test));


    //packet.mutable_robot_commands()->AddAllocated(command[0]);
     // packet.SerializePartialToArray(s_data, sizeof(s_data));
//packet.SerializeToArray(s_data, sizeof(s_data));
//packet.robot_commands().begin().;

//ERforce.send(s_data, sizeof(s_data));
    //ERforce.send(packet.robot_commands()., sizeof(packet));
 //   cout<<ERforce.recive()<<'\n';

    //SimulatorError test;
    //cout<<test.mutable_message()<<'\n';




   //packet.add_robot_commands()->set_id(1);
   //packet.mutable_robot_commands()->Add(&hello)
    //packet.add_robot_commands()->set_id(1);
    //packet.mutable_robot_commands(1)->mutable_move_command()->local_velocity(;
     // packet.add_robot_commands()->set_kick_speed(0);
    //packet.add_robot_commands()->mutable_move_command()->mutable_wheel_velocity()->set_front_right(4);
    //packet.add_robot_commands()->mutable_move_command()->mutable_wheel_velocity()->set_front_left(4);
   // command[0]->set_id(1);
   // command[0]->set_kick_speed(0);
    //RobotMoveCommand test;
    //test.mutable_wheel_velocity()->set_front_left(4);
    //test.mutable_wheel_velocity()->set_front_right(-4);
    //command[0]->set_allocated_move_command(&test);
    //RobotCommand packet2 ;
    //packet2.set_id(1);
    //packet2.mutable_move_command()->local_velocity().New()->set_forward(4);
    //new RobotCommand(packet2);
    //packet2.set_id(1);
    //packet2.mutable_move_command()->mutable_wheel_velocity()->set_front_right(4);
   // packet2.mutable_move_command()->mutable_wheel_velocity()->set_front_left(-4);
   // command[0]->mutable_move_command()->mutable_wheel_velocity()->set_front_left(4);
   //command[0]->mutable_move_command()->mutable_wheel_velocity()->set_front_right(-4);
    //testi.set_id(2);
    //testi.set_kick_speed(0);
    //testi.mutable_move_command()->mutable_wheel_velocity()->set_front_left(4);
    //testi.mutable_move_command()->mutable_wheel_velocity()->set_front_left(-4);
    //testi.SerializePartialToArray(s_data,sizeof s_data);

  //  packet.SerializePartialToArray(s_data, sizeof(s_data));
  //  cout<<s_data;
    //s_data[0]='h';
  //  ERforce.send(s_data, sizeof(s_data));
    //char data[3]={'h','e','l'};
   // ERforce.send(data, sizeof(data));
    //cout<<"send data\n";
//packet.mutable_robot_commands().
   // ERforce.send(s_data, sizeof(s_data));
}

#else
/*!GrsimMove METHODS */

///generate a "command" that is a set of attributes for a one robot and somehow! connected to packet
SimulatorMove::SimulatorMove()
{
	for(int i=0;i<MAX_ROBOTS_PER_TEAM_IN_THE_FIELD;i++)
		command[i] = packet.mutable_commands()->add_robot_commands();
}

void SimulatorMove::initialize_port(const char* ip, int port)
{
	grsim_udp.Init_Socket_Server(ip, port); //"192.168.0.255"
	
}

void SimulatorMove::initialize_robot_color_and_timestamp(double timestamp)
{
	packet.mutable_commands()->set_isteamyellow(World::team_color == TC_Yellow);
	packet.mutable_commands()->set_timestamp(timestamp);
}

void SimulatorMove::go(VecPosition vv, double w, char id, World world, move_type mt)
{
	int index_for_id = world.getIndexForRobotTNumber(id);

	command[id]->set_id(id);
	if (mt == wheels_speed)
	{
		MatrixD V;
		command[id]->set_wheelsspeed(true);

		convert_robot_velocity_to_wheels_velocity(vv, w, world.robotT[index_for_id].angle, V);
		set_wheels_velocity(V, id);
		send_to_grsim();
	}

	else
	{
		VecPosition velocity;
		command[id]->set_wheelsspeed(false);

		velocity = convert_robot_velocity_from_field_to_robot_coord(vv, world.robotT[index_for_id].angle);
		set_velocity_and_W(velocity, w, id);
		send_to_grsim();
	}


}

void SimulatorMove::go_withoutSend(VecPosition vv, double w, char id,  World world, move_type mt)
{
	int index_for_id = world.getIndexForRobotTNumber(id);

	command[id]->set_id(id);
	if (mt == wheels_speed)
	{
		MatrixD V;
		command[id]->set_wheelsspeed(true);

		convert_robot_velocity_to_wheels_velocity(vv,w, world.robotT[index_for_id].angle, V);
		set_wheels_velocity(V,id);
	}

	else
	{
		VecPosition velocity;
		command[id]->set_wheelsspeed(false);

		velocity = convert_robot_velocity_from_field_to_robot_coord(vv, world.robotT[index_for_id].angle);
		set_velocity_and_W(velocity,w,id);
	}


}

///shootOrChip == 1 for shoot, shootOrChip == 0 for chip 
void SimulatorMove::go_setKick_setSpinBack_withoutSend(VecPosition vv, double w, bool shootOrChip, short int kickPower, bool spinBack, char id, World world, move_type mt)
{
	int index_for_id = world.getIndexForRobotTNumber(id);

	command[id]->set_id(id);
	if (mt == wheels_speed)
	{
		MatrixD V;
		command[id]->set_wheelsspeed(true);

		convert_robot_velocity_to_wheels_velocity(vv, w, world.robotT[index_for_id].angle, V);
		set_wheels_velocity(V, id);
		set_kick(shootOrChip, kickPower, id);
		set_spinBack(spinBack, id);
	}

	else
	{
		VecPosition velocity;
		command[id]->set_wheelsspeed(false);

		velocity = convert_robot_velocity_from_field_to_robot_coord(vv, world.robotT[index_for_id].angle);
		set_velocity_and_W(velocity, w, id);
		set_kick(shootOrChip, kickPower, id);
		set_spinBack(spinBack, id);
	}
}
//void SimulatorMove::convert_robot_velocity_to_wheels_velocity(VecPosition RV, double w, AngRad rteta, MatrixD &V_send_out)
//{
//	//ww -> charkhesh hole mehvar khod.
//
//	MatrixD A(4, 3), V(4, 1), VW1(2, 1), VW(3, 1);
//
//	double R = 90;
//	VW(0, 0) = (RV.getX() / 64.5);
//	VW(1, 0) = (RV.getY() / 64.5);
//	VW(2, 0) = w;// (ww / 180);
//
//	double modified_coeficient = 1;//0.68
//	double bb0 = Deg2Rad(300), bb1 = Deg2Rad(225), bb2 = Deg2Rad(135), bb3 = Deg2Rad(60);
//
//	for (int k = 0; k < 2; k++)
//	{
//		A(0, 0) = (cos(rteta)*sin(bb0) + sin(rteta)*cos(bb0));     A(0, 1) = (sin(rteta)*sin(bb0) - cos(rteta)*cos(bb0));    A(0, 2) = -R;
//		A(1, 0) = (cos(rteta)*sin(bb1) + sin(rteta)*cos(bb1));     A(1, 1) = (sin(rteta)*sin(bb1) - cos(rteta)*cos(bb1));    A(1, 2) = -R;
//		A(2, 0) = (cos(rteta)*sin(bb2) + sin(rteta)*cos(bb2));     A(2, 1) = (sin(rteta)*sin(bb2) - cos(rteta)*cos(bb2));    A(2, 2) = -R;
//		A(3, 0) = (cos(rteta)*sin(bb3) + sin(rteta)*cos(bb3));     A(3, 1) = (sin(rteta)*sin(bb3) - cos(rteta)*cos(bb3));    A(3, 2) = -R;
//
//		/*MatrixD rotation(3, 3), B(4, 3);
//		rotation(0, 0) = cos(rteta); rotation(0, 1) = -sin(rteta ); rotation(0, 2) = 0;
//		rotation(1, 0) = sin(rteta); rotation(1, 1) = cos(rteta ); rotation(1, 2) = 0;
//		rotation(2, 0) = 0; rotation(2, 1) = 0; rotation(2, 2) = 1;
//		double pi = M_PI;
//		B(0, 0) = (-0.5 / (cos(pi / 3) + cos(pi / 4)));     B(0, 1) = (0.5 / (sin(pi / 3) + sin(pi / 4)));   B(0, 2) = cos(pi / 4) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
//		B(1, 0) = (0.5 / (cos(pi / 3) + cos(pi / 4)));      B(1, 1) = (0.5 / (sin(pi / 3) + sin(pi / 4)));    B(1, 2) = cos(pi / 3) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
//		B(2, 0) = (0.5 / (cos(pi / 3) + cos(pi / 4)));     B(2, 1) = (-0.5 / (sin(pi / 3) + sin(pi / 4)));   B(2, 2) = cos(pi / 3) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
//		B(3, 0) = (-0.5 / (cos(pi / 3) + cos(pi / 4)));    B(3, 1) = (-0.5 / (sin(pi / 3) + sin(pi / 4)));  B(3, 2) = cos(pi / 4) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
//		A = B*rotation;*/
//
//		V = modified_coeficient* A * VW;
//
//		double limit_v = 30;
//		double swap_v;
//		swap_v = limit_v;
//		if (fabs(V(0, 0)) > limit_v || fabs(V(1, 0)) > limit_v || fabs(V(2, 0)) > limit_v || fabs(V(3, 0)) > limit_v)
//		{
//
//			for (int i = 0; i < 4; i++)
//			{
//				if (fabs(V(i, 0)) > swap_v)
//				{
//					swap_v = fabs(V(i, 0));
//				}
//			}
//			modified_coeficient = (limit_v) / (swap_v);
//
//		}
//	}
//
//	for (int m = 0; m < 4; m++)
//	{
//		V(m, 0) = floor((ceil(2 * V(m, 0))) / 2);
//	}
//	V_send_out = V;
//}

///for grsim
void SimulatorMove::convert_robot_velocity_to_wheels_velocity(VecPosition RV, double w, AngRad rteta, MatrixD &V_send_out)
{
	///ww -> charkhesh hole mehvar khod.

	MatrixD A(4, 3), V(4, 1), VW1(2, 1), VW(3, 1);

	double R = 90;
	VW(0, 0) = (RV.getX()/7.5 /* / 64.5*/);
	VW(1, 0) = (RV.getY()/7.5 /* / 64.5*/);
	
	VW(2, 0) = w/27; //(ww / 180) 	///alireza : w/27

	double modified_coeficient = 1;//0.68
	//double bb0 = Deg2Rad(315), bb1 = Deg2Rad(225), bb2 = Deg2Rad(135), bb3 = Deg2Rad(45);	//for grsim
	double bb0 = Deg2Rad(300), bb1 = Deg2Rad(225), bb2 = Deg2Rad(135), bb3 = Deg2Rad(60);	//for omid robots

	for (int k = 0; k < 2; k++)
	{
		A(0, 0) = (cos(rteta)*sin(bb0) + sin(rteta)*cos(bb0));     A(0, 1) = (sin(rteta)*sin(bb0) - cos(rteta)*cos(bb0));    A(0, 2) = -R;
		A(1, 0) = (cos(rteta)*sin(bb1) + sin(rteta)*cos(bb1));     A(1, 1) = (sin(rteta)*sin(bb1) - cos(rteta)*cos(bb1));    A(1, 2) = -R;
		A(2, 0) = (cos(rteta)*sin(bb2) + sin(rteta)*cos(bb2));     A(2, 1) = (sin(rteta)*sin(bb2) - cos(rteta)*cos(bb2));    A(2, 2) = -R;
		A(3, 0) = (cos(rteta)*sin(bb3) + sin(rteta)*cos(bb3));     A(3, 1) = (sin(rteta)*sin(bb3) - cos(rteta)*cos(bb3));    A(3, 2) = -R;

		/*MatrixD rotation(3, 3), B(4, 3);
		rotation(0, 0) = cos(rteta); rotation(0, 1) = -sin(rteta ); rotation(0, 2) = 0;
		rotation(1, 0) = sin(rteta); rotation(1, 1) = cos(rteta ); rotation(1, 2) = 0;
		rotation(2, 0) = 0; rotation(2, 1) = 0; rotation(2, 2) = 1;
		double pi = M_PI;
		B(0, 0) = (-0.5 / (cos(pi / 3) + cos(pi / 4)));     B(0, 1) = (0.5 / (sin(pi / 3) + sin(pi / 4)));   B(0, 2) = cos(pi / 4) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
		B(1, 0) = (0.5 / (cos(pi / 3) + cos(pi / 4)));      B(1, 1) = (0.5 / (sin(pi / 3) + sin(pi / 4)));    B(1, 2) = cos(pi / 3) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
		B(2, 0) = (0.5 / (cos(pi / 3) + cos(pi / 4)));     B(2, 1) = (-0.5 / (sin(pi / 3) + sin(pi / 4)));   B(2, 2) = cos(pi / 3) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
		B(3, 0) = (-0.5 / (cos(pi / 3) + cos(pi / 4)));    B(3, 1) = (-0.5 / (sin(pi / 3) + sin(pi / 4)));  B(3, 2) = cos(pi / 4) / ((cos(pi / 3) + cos(pi / 4)) * 2 * R);
		A = B*rotation;*/

		V = modified_coeficient* A * VW;

		/*double limit_v = 30;
		double swap_v;
		swap_v = limit_v;
		if (fabs(V(0, 0)) > limit_v || fabs(V(1, 0)) > limit_v || fabs(V(2, 0)) > limit_v || fabs(V(3, 0)) > limit_v)
		{

			for (int i = 0; i < 4; i++)
			{
				if (fabs(V(i, 0)) > swap_v)
				{
					swap_v = fabs(V(i, 0));
				}
			}
			modified_coeficient = (limit_v) / (swap_v);

		}*/
	}


	/*for (int m = 0; m < 4; m++)
	{
		V(m, 0) = (V(m, 0) / max_wheel_speed) * 32.000;
	}*/

	for (int m = 0; m < 4; m++)
	{
		V(m, 0) = floor((ceil(2 * V(m, 0))) / 2);
	}
	/*for (int m = 0; m < 4; m++)
	{
		V(m, 0) *= max_wheel_speed/32.000;
	}*/
	V_send_out = V;
}

VecPosition SimulatorMove::convert_robot_velocity_from_field_to_robot_coord(VecPosition RFV , AngRad rteta)
{
	return VecPosition(RFV.InerMultiply(VecPosition::directVector(rteta)), RFV.InerMultiply(VecPosition::directVector(rteta + (M_PI / 2.0))));
}

void SimulatorMove::set_wheels_velocity(MatrixD V,char id)
{
	command[id]->set_wheel1((float)-V(3, 0));	//left front
	command[id]->set_wheel2((float)-V(2, 0));	//left behind
	command[id]->set_wheel3((float)-V(1, 0));	//right behind
	command[id]->set_wheel4((float)-V(0, 0));	//right front	
}

void SimulatorMove::set_velocity_and_W(VecPosition velocity, double w, char id)
{
	command[id]->set_veltangent(velocity.getX() / 300.000);
	command[id]->set_velnormal(velocity.getY() / 300.000);
	command[id]->set_velangular(w);				//need to change

}

///shootOrChip == 0 for shoot, shootOrChip == 1 for chip 
void SimulatorMove::set_kick(bool shootOrChip, short int kickPower, char id)
{
	if (shootOrChip == 0)
	{
		command[id]->set_kickspeedx(3 / 2.0*kickPower);
		command[id]->set_kickspeedz(0);
	}
	else if (shootOrChip == 1)
	{
		command[id]->set_kickspeedx(3 / 2.0*kickPower*0.70710);	/// ...*cos(M_PI/4)
		command[id]->set_kickspeedz(3 / 2.0*kickPower*0.70710); /// ...*sin(M_PI/4)
	}
}

void SimulatorMove::set_spinBack(bool set,char id)
{
	command[id]->set_spinner(set);
}

void SimulatorMove::send_to_grsim()
{
        packet.SerializePartialToArray(s_data, sizeof(s_data));
        grsim_udp.send(s_data, sizeof(s_data));
}

/*!GrsimMove METHODS */


///ctrl_handler
//BOOL nrf::ctrl_handler(DWORD event)
//	{
//
//		if (event == CTRL_CLOSE_EVENT)
//		{
//			input[29] = 9;
//			input[28] = 0;
//			input[27] = 0;
//			input[26] = 0;
//
//			WriteFile(hComm,
//				input,
//				numOfBytesToWrite,
//				&numOfBytesWritten,
//				NULL);
//
//			return true;
//		}
//		return false;
//	}


///old main for testing nrf
//int main()
//{		
//		
//	/*input[29] = 1;
//	input[28] = 0;
//	input[27] = 0;
//	input[26] = 0;
//	input[25] = 0;*/
//
//	/*input[29] = 1;
//	input[28] = 4;
//	input[27] = 31;
//	input[26] = 255;*/
//
//	//1_5_10_41
//
//	/*00000001   (000001)(00   0001)(0000   01)(000001)
//	00000001   (000001)(00   0001)(1111   11)(111111)*/
//	
//	/*SetConsoleCtrlHandler((PHANDLER_ROUTINE)(ctrl_handler), true);*/
//
//	/*MatrixD V(4, 1);
//	VecPosition a;*/
//	
//
//	/*hComm = CreateFile(portName,
//		GENERIC_READ | GENERIC_WRITE,
//		0,
//		NULL,
//		OPEN_EXISTING,
//		0,
//		NULL);*/
//
//	
//	/*dcbSerialParams.BaudRate = CBR_256000;
//	dcbSerialParams.ByteSize = 8;
//	dcbSerialParams.StopBits = ONESTOPBIT;
//	dcbSerialParams.Parity = NOPARITY;
//
//	status = SetCommState(hComm, &dcbSerialParams);*/
//
//	//a.setX(0);
//	//a.setY(0);
//
//	////200y->chap
//	////200x->jelo
//
//	//convert_velocity_to_wheel(a, 0, 0, V);
//
//	//cout << V(0, 0) << endl;
//	//cout << V(1, 0) << endl;
//	//cout << V(2, 0) << endl;
//	//cout << V(3, 0) << endl;
//
//	//set_velocity(V);
//	//write_on_port();
//}
#endif

