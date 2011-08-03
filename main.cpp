#include "simulation.h"
#include "utils.h"

void printControls()
{
	printf("CONTROLS:\n");
	printf("  > WASD - Move the playeres\n");
	printf("  > E Q - Spin the playeres\n");
	printf("  > SPACE - Kick\n");
	printf("  > V - Drible\n");
	printf("  > TAB - Select player\n");
	printf("  > IJKL - Move the ball\n\n");
}

int main(int argc, char** argv) {
	
	parseOptions(argc,argv);
	
	printControls();

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
