
#ifndef REFEREE_H
#define REFEREE_H
#include "Switches.h"
#include "Protobuf/Refree/referee.pb.h"
#include "GameState.h"
#include <stdint.h>
#include "Socket_udp.h"
#include "world.h"
#include "GameState.h"

class Refree
{
public:
    explicit Refree(void);
    void Refree_parser(World &world);
    void recive_Init(void);

public:
    Socket_udp refree_udp;
    SSL_Referee packet;
    //Game_State::State return_referee_inputstate;
    uint32_t  m_counter;
    uint32_t stage_time_left;

};
#endif
