#include "simulation.h"

Robot robots[TEAM_TOTAL][MAX_ROBOTS];
Ball ball;
int playersTotal[TEAM_TOTAL] = {MAX_ROBOTS, MAX_ROBOTS};
b2World* world;

int main(int argc, char** argv) {

	initWorld();
	initObjects();

	printf("Simulator Running!\n");

	//radiotosim.open(false);
	openradiotosim();

	printf("Press <Enter> to open connection with client...\n");
	getchar();
	//simtotracker.open();
	opensimtotracker();

	initGlut(argc, argv);

	/*
	clrscr();
	int scrCount = 0;
	while(1) {
	    scrCount++;
	    if(scrCount == SCR_CLEAR_DELAY) {
	        scrCount = 0;
	        clrscr();
	    }
		rewindscr();
		receive();
		process();
		send();
	}*/
}
