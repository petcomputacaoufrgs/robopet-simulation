#include <stdio.h>
#include <iostream>
using namespace std;

#include "rp_client.h"
#include "rp_server.h"
#include "vector.h"
#include "Box2D.h"
#include <GL/glut.h>
#include "constants.h"

#define drawLine(x1,y1,x2,y2) glBegin(GL_LINES); glVertex2f ((x1),(y1)); glVertex2f ((x2),(y2)); glEnd();

#define MM_TO_M(x) (x/100.)
#define M_TO_MM(x) (100.*x)

//-------------

class Robot {

	public:
		Robot() { isUpdated=false; };
		~Robot() {};

		b2Body* body;
		Vector forces;
		float displacement_angle;
		int doKick;
		int doDribble;
		int id;
		bool isUpdated;

		bool pointingToBall();
		bool closeToBall();
};


struct Ball {
	b2Body* body;
	Vector  _forces;
};


// parse command line options
void parseOptions(int argc, char **argv);

void receive();

void send();

//makes each step of simulation
void process();

// create a box physics object which will be used for the walls
b2Body* newWall(float x, float y, float sizex, float sizey);

// create a physics circular object which will be used for the ball and robots
b2Body* newDynamicCircle(float x, float y, float radius, float density, float friction, float restitution, float damping, b2Color color);

void initObjects();

// glut keyboard keys
void keyboardFunc(unsigned char key, int xmouse, int ymouse);

// glut drawing function
void drawScene();

void initWorld();

void initGlut(int argc, char** argv);


void openRadiotosim();
void openSimtotracker();
