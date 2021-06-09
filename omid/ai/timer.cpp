#include "timer.h"
//#include <conio.h>
#include <time.h>	// class needs this inclusion






//////////////////////////////////////////
// class implementation:


Timer::Timer() {
	resetted = true;
	running = false;
	beg = 0;
	end = 0;
}


void Timer::start() {
	if (!running) {
		if (resetted)
			beg = (unsigned long)clock();
		else
			beg -= end - (unsigned long)clock();
		running = true;
		resetted = false;
	}
}


void Timer::stop() {
	if (running) {
		end = (unsigned long)clock();
		running = false;
	}
}


void Timer::reset() {
	bool wereRunning = running;
	if (wereRunning)
		stop();
	resetted = true;
	beg = 0;
	end = 0;
	if (wereRunning)
		start();
}


bool Timer::isRunning() {
	return running;
}


unsigned long Timer::getTime() {
	if (running)
		return ((unsigned long)clock() - beg) / CLOCKS_PER_SEC;
	else
		return end - beg;
}


bool Timer::isOver(unsigned long seconds) {
	return seconds >= getTime();
}
