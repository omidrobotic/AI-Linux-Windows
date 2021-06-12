#ifndef NRF_H
#define NRF_H


#include "math.h"
#include "matrix.h"
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
    #include <termios.h> // Contains POSIX terminal control definitions
    #include <unistd.h> // write(), read(), close()

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
   //#include <X11/Xlib.h>
   typedef const char* LPCSTR;
   typedef char* LPSTR;
   typedef wchar_t* LPWSTR;
   typedef const wchar_t* LPCWSTR;
   typedef const char* LPCTSTR;
   typedef char* LPTSTR;
   typedef unsigned long int DWORD;
   typedef unsigned int HANDLE; 
   typedef unsigned int DCB;  
   //#include "myheader.h"    // can no longer use windows.h or conio.h
   //#include "myheader2.h"
#endif
#include "geometry.h"
#include "Socket_udp.h"
#include "Switches.h"
#if SEND_COMMANDS_TO_ROBOTS==2
#include "Protobuf/ER-force/ssl_simulation_robot_control.pb.h"
#else
#include "Protobuf/Grsim/grSim_Packet.pb.h"
#endif


class VecPosition;
class World;


using namespace math;
typedef matrix<double> MatrixD; //matrix double

class nrf
{

public:
	static char input[1000];

	static void go(VecPosition vp, char id);
	static void go(VecPosition vp, int robot_number, char id, World world, double ww);
	static void go_withoutSend(VecPosition vp, double ww, int robot_number, char id, World world);
	static void convert_robot_velocity_to_wheels_velocity(VecPosition vv, double ww, AngRad rteta, MatrixD &V_send_out);	///for real robots
	static void set_velocity(MatrixD V, char id);
	static void write_on_port();
	static void read_from_port();
	static char output[180];
private:
	
	static char ID;
	//MatrixD V(4, 1);	//velocity matrix
	static HANDLE hComm;
	static DWORD numOfBytesToWrite;
	static DWORD numOfBytesWritten;
	static DWORD numOfBytesToRead;
	static DWORD numOfBytesRead;
	static LPCSTR portName;
	static DCB dcbSerialParams;
	static MatrixD V;
	static bool statusNrf;
	static bool ftime;

	//static bool ctrl_handler(DWORD event);

};

class SimulatorMove {
public:

    enum move_type {

        wheels_speed = 1,
        robot_speed = 2
    };



    void initialize_port(const char *ip, int port);

    void initialize_robot_color_and_timestamp(double timestamp);

    void go_withoutSend(VecPosition vp, double ww, char id, World world, move_type mt);

    void go(VecPosition vp, double ww, char id, World world, move_type mt);

    void
    go_setKick_setSpinBack_withoutSend(VecPosition vp, double ww, bool shootOrChip, short int kickPower, bool spinBack,
                                       char id, World world, move_type mt);

    //void convert_robot_velocity_to_wheels_velocity(VecPosition vv, double ww, AngRad rteta, MatrixD &V_send_out);
    void convert_robot_velocity_to_wheels_velocity(VecPosition vv, double ww, AngRad rteta,
                                                   MatrixD &V_send_out);    ///for grsim
    VecPosition convert_robot_velocity_from_field_to_robot_coord(VecPosition vv, AngRad rteta);

    void set_wheels_velocity(MatrixD V, char id);

    void set_velocity_and_W(VecPosition Velocity, double w, char id);

    void set_kick(bool shootOrChip, short int kickPower, char id);

    void set_spinBack(bool spinBack, char id);


    char s_data[1024];     ///whole data that will be send.
    double max_wheel_speed = 80;
    SimulatorMove();
#if SEND_COMMANDS_TO_ROBOTS==2
    Socket_udp ERforce;
    RobotControl packet;
    RobotCommand *command[MAX_ROBOTS_PER_TEAM_IN_THE_FIELD];
    //RobotMoveCommand *speed;
    void send_to_ERforce();
    void testy();
#else
    Socket_udp grsim_udp;
    void send_to_grsim();
    grSim_Packet packet;     ///packet has a set of commands that each command is attributes of one robot,such speed of each wheel, etc.
    grSim_Robot_Command *command[8];    /// 8: need to be changed !!!!!!!!!	///command is a set of attributes for a robot.such as speed of each wheel, etc.
#endif

};
#endif // NRF_H









