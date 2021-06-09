#include "Vision.h"
#include "Switches.h"

void Vision::recive_Init(void)
{
	vision_udp.Init_Socket_Client(GROUP_ADDR_Vision, PORT_NUM_Vision);
}

bool Vision::recievePacket(void)
{
	
	int ct=vision_udp.recive();
	//cout << "\n vision ln:" << ct;
	packet.ParseFromArray(vision_udp.buffer_recive, ct);
	
	if (packet.has_detection())
	{
		
		frame[packet.detection().camera_id()] = packet.detection();
		packet_recieved[packet.detection().camera_id()] = true;
	}
	else
	{
		return false;
	}
	return true;
}
