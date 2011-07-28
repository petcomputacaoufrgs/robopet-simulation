#include "simulation.h"

int main(int argc, char** argv) {
	
	printf("CONTROLS:\n");
	printf("  > WASD - Move the playeres\n");
	printf("  > E Q - Spin the playeres\n");
	printf("  > SPACE - Kick\n");
	printf("  > V - Drible\n");
	printf("  > TAB - Select player\n");
	printf("  > IJKL - Move the ball\n\n");

	parseOptions(argc,argv);
	
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
