#ifndef SOCKET_UDP_H
#define SOCKET_UDP_H


//----- Include files -------------------------------------------------------
#include <stdio.h>          // Needed for printf()
#include <stdlib.h>         // Needed for memcpy()
#include <string>
#ifdef _WIN32
#include <winsock.h>      // Needed for all Windows stuff
#elif __linux__
#include<stdio.h>	//printf
#include<string.h> //memset
#include<stdlib.h> //exit(0);
#include<arpa/inet.h>
#include <sys/types.h>    // Needed for system defined identifiers.
#include <netinet/in.h>   // Needed for internet address structure.
#include <sys/socket.h>   // Needed for socket(), bind(), etc...
#include <arpa/inet.h>    // Needed for inet_ntoa()
#include <fcntl.h>
#include <netdb.h>
#include <unistd.h>
#include <netinet/in.h>
#endif
#include "Switches.h"
class Socket_udp
{

public:
    void send2ERforce(std::string *buffer_send,int buffer_send_len);
#ifdef _WIN32
explicit Socket_udp(void);
	 void Init_Socket_Server(const char * Group_Addr, int Port_Num,char *_udp_client_interface);
     void Init_Socket_Client(const char * Group_Addr, int Port_Num);
	 void send(char buffer_send[2048], int buffer_send_len);
	 int recive(void);
	 void Close_Socket(void);
#elif __linux__
    explicit Socket_udp(void);
    void Init_Socket_Server(in_addr_t Group_Addr, int Port_Num,const char * _udp_client_interface);
    void Init_Socket_Client(const char * Group_Addr, int Port_Num);
    void send(char buffer_send[100000], int buffer_send_len);
    int recive(void);
    void Close_Socket(void);
#endif

public:
#ifdef WIN
	  WORD wVersionRequested; // Stuff for WSA functions
	 WSADATA wsaData;                        // Stuff for WSA functions


#elif __linux__

#endif

      char *inteface="defult";
	  unsigned int         Multi_Server_Sock;
	  struct sockaddr_in   Client_Addr;
	  struct sockaddr_in   Server_Addr;
	  unsigned char        TTL;
	  char                 buffer_recive[100000] ;
	  struct ip_mreq       mreq;              // Multicast group structure
	  unsigned int                  addr_len;          // Internet address length
	  int                  retcode;           // Return code

};
#endif //SOCKET_UDP_H
