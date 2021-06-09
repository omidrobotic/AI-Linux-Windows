#ifndef TIMER_H
#define TIMER_H
#include <stdio.h>
#include <stdlib.h>
#include <ctime>
#include <chrono>

using namespace std;

class Timer {
public:
	Timer();
	void           start();
	void           stop();
	void           reset();
	bool           isRunning();
	unsigned long  getTime();
	bool           isOver(unsigned long seconds);
private:
	bool           resetted;
	bool           running;
	unsigned long  beg;
	unsigned long  end;
};
#endif // TIMER_H
