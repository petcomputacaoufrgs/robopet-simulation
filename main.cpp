#include "simulation.h"

int main(int argc, char** argv) {

	initWorld();
	initObjects();

	printf("Simulator Running!\n");

	openRadiotosim();

	printf("Press <Enter> to open connection with client...\n"); getchar();

	openSimtotracker();

	initGlut(argc, argv); //open renderer

	/* //this is the original main loop, without glut loop
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
