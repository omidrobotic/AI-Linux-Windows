#include "Socket_udp.h"
//omidguard dont touch
Socket_udp::Socket_udp(void)
{

}
void Socket_udp::Init_Socket_Server(const char * Group_Addr, int Port_Num)
{
#ifdef WIN
// This stuff initializes winsock
WSAStartup(wVersionRequested, &wsaData);
#elif __linux__

#endif

// Create a multicast socket
Multi_Server_Sock = socket(AF_INET, SOCK_DGRAM, 0);
/*    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse,sizeof(reuse))!=0)*/
if (Multi_Server_Sock < 0)
{
	printf("*** ERROR - Create a multicast socket= %d \n", Multi_Server_Sock);
	return;
}
// Create multicast group address information
Server_Addr.sin_family = AF_INET;
Server_Addr.sin_addr.s_addr = inet_addr(Group_Addr);
Server_Addr.sin_port = htons(Port_Num);

// Set the TTL for the sends using a setsockopt()
TTL = 1;
retcode = setsockopt(Multi_Server_Sock, IPPROTO_IP, IP_MULTICAST_TTL,
	(char *)&TTL, sizeof(TTL));
if (retcode < 0)
{
	printf("*** ERROR - setsockopt() failed with retcode = %d \n", retcode);
	return;
}

// Set addr_len
addr_len = sizeof(Server_Addr);
printf("*** ready to Sending multicast datagrams to '%s' (port = %d) \n",
	Group_Addr, Port_Num);}

void Socket_udp::Init_Socket_Client(const char * Group_Addr, int Port_Num)
{

#ifdef WIN
	wVersionRequested = MAKEWORD(1, 1); // Stuff for WSA functions										// This stuff initializes winsock
	WSAStartup(wVersionRequested, &wsaData);
#endif
bool blocking= true;
bool share_port_for_multicasting= true;
bool multicast_include_localhost= true;
const int TTL_test=3;
	// Create a multicast socket and fill-in multicast address information
	if(Multi_Server_Sock>0)
    {
        close(Multi_Server_Sock);
    }
	Multi_Server_Sock = socket(AF_INET, SOCK_DGRAM/*pfinet*/, 0);
// set socket as non-blocking
  int flags = fcntl(Multi_Server_Sock, F_GETFL, 0);
  if(flags < 0) flags = 0;
  fcntl(Multi_Server_Sock, F_SETFL, flags | (blocking ? 0 : O_NONBLOCK));

  if (share_port_for_multicasting) {
    int reuse=1;
    if(setsockopt(Multi_Server_Sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse,sizeof(reuse))!=0) {
      fprintf(stderr,"ERROR WHEN SETTING SO_REUSEADDR ON UDP SOCKET\n");
      fflush(stderr);
    }
    fcntl(Multi_Server_Sock, F_SETFL, flags | (blocking ? 0 : O_NONBLOCK));
      /*if(setsockopt(fd, SOL_SOCKET, SO_REUSEPORT, 1)!=0) {
           fprintf(stderr,"ERROR WHEN SETTING SO_REUSEPORT ON UDP SOCKET\n");
           fflush(stderr);
         }*/
  }

    if (multicast_include_localhost) {
        int yes = 1;
        // allow packets to be received on this host
        if (setsockopt(Multi_Server_Sock, IPPROTO_IP, IP_MULTICAST_LOOP, (const char*)&yes, sizeof(yes))!=0) {
            fprintf(stderr,"ERROR WHEN SETTING IP_MULTICAST_LOOP ON UDP SOCKET\n");
            fflush(stderr);
        }
    }
    int ret = setsockopt(Multi_Server_Sock, IPPROTO_IP, IP_MULTICAST_TTL, &TTL_test, sizeof(TTL_test));
    if(ret != 0)
    {
        printf("ERROR %d WHEN SETTING IP_MULTICAST_TTL\n", ret);
    }





    // Create client address information and bind the multicast socket
	Client_Addr.sin_family = AF_INET;
	Client_Addr.sin_port = htons(Port_Num);
	Client_Addr.sin_addr.s_addr = INADDR_ANY;

	retcode = bind(Multi_Server_Sock, (struct sockaddr *)&Client_Addr,
		sizeof(struct sockaddr));
	if (retcode < 0)
	{
		printf("*** ERROR - bind() failed with retcode = %d \n", retcode);
		getchar();
		//return;
	}





	// Have the multicast socket join the multicast group
	/*mreq.imr_multiaddr.s_addr = inet_addr(Group_Addr);
	mreq.imr_interface.s_addr = INADDR_ANY;
	retcode = setsockopt(Multi_Server_Sock, IPPROTO_IP, IP_ADD_MEMBERSHIP,
		(char *)&mreq, sizeof(mreq));
	if (retcode < 0)

	{
		printf("*** ERROR - setsockopt() failed with retcode = %d \n", retcode);
		getchar();
		// return;
	}
*/
}
void Socket_udp::send(char buffer_send[100000],int buffer_send_len)
{

	    addr_len = sizeof(Server_Addr);
		// Send buffer as a datagram to the multicast group
		sendto(Multi_Server_Sock, buffer_send, buffer_send_len, 0,(struct sockaddr*)&Server_Addr, addr_len);


}
void Socket_udp::send2ERforce(std::string *buffer_send,int buffer_send_len)
{

    addr_len = sizeof(Server_Addr);
    // Send buffer as a datagram to the multicast group
    sendto(Multi_Server_Sock, &buffer_send, buffer_send_len, 0,(struct sockaddr*)&Server_Addr, addr_len);


}
int Socket_udp::recive(void)
{

	
	addr_len = sizeof(Client_Addr);
	
		// Receive a datagram from the multicast server
		if ((retcode = recvfrom(Multi_Server_Sock, buffer_recive, sizeof(buffer_recive), 0,
			(struct sockaddr *)&Client_Addr, &addr_len)) < 0) {
			printf("*** ERROR - recvfrom() failed %d \n ", addr_len);
			//getchar();
		}

		// Output the received buffer to the screen as a string
		//printf("%s\n", buffer);
		return retcode;
}
void Socket_udp::Close_Socket(void)
{
	// Close and clean-up
#ifdef WIN
	closesocket(Multi_Server_Sock);
WSACleanup();
#elif __linux__
close(Multi_Server_Sock);
#endif
}


