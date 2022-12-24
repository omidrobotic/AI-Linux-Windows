#include <iostream>     ///< cout

#include "../omid/ai/GetSystemData.h"
#include <cstring>      ///< memset
#include <string>
#include <errno.h>      ///< errno
#include <sys/socket.h> ///< socket
#include <netinet/in.h> ///< sockaddr_in
#include <arpa/inet.h>  ///< getsockname
#include <unistd.h>     ///< close

using namespace std;

int main()
{
	GetSystemData mysystemip;
	const char* myip = mysystemip.GetIP();
	cout << myip;
}