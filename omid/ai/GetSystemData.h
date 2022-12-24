#ifndef GETSYSTEMDATA.H
#define GETSYSTEMDATA.H
#include <iostream>     ///< cout
#include <cstring>      ///< memset
#include <string>
#include <errno.h>      ///< errno
#include <sys/socket.h> ///< socket
#include <netinet/in.h> ///< sockaddr_in
#include <arpa/inet.h>  ///< getsockname
#include <unistd.h>     ///< close

class GetSystemData{
    public:
    const char* GetIP()
    {
        const char* google_dns_server = "8.8.8.8";
        int dns_port = 53;

        struct sockaddr_in serv;
        int sock = socket(AF_INET, SOCK_DGRAM, 0);

        //Socket could not be created
        if(sock < 0)
        {
            std::cout << "Socket error" << std::endl;
            return "Socket error" ;
        }

        memset(&serv, 0, sizeof(serv));
        serv.sin_family = AF_INET;
        serv.sin_addr.s_addr = inet_addr(google_dns_server);
        serv.sin_port = htons(dns_port);

        int err = connect(sock, (const struct sockaddr*)&serv, sizeof(serv));
        if (err < 0)
        {
            std::cout << "Error number: " << errno
                << ". Error message: " << strerror(errno) << std::endl;
            close(sock);
            return strerror(errno) ;

        }

        struct sockaddr_in name;
        socklen_t namelen = sizeof(name);
        err = getsockname(sock, (struct sockaddr*)&name, &namelen);

        auto buffer = new char[80];
        const char* p = inet_ntop(AF_INET, &name.sin_addr, buffer, 80);
        if(p != NULL)
        {
            // std::cout << "Local IP address is: " << buffer << std::endl;
            close(sock);
            //std::cout<<"&&**&*"<<buffer<<"**&*&*";
            return buffer;
        }
        else
        {
            std::cout << "Error number: " << errno
                << ". Error message: " << strerror(errno) << std::endl;
            close(sock);
            return strerror(errno);
        }

        close(sock);
        return "0";
    }

};

#endif

