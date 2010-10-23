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

#define MAX_ROBOTS 5
#define CONSTANTE_DESLOCAMENTO 0.01 //deve ser menor que 1 e maior que 0
#define CONSTANTE_POSICIONAMENTO_INICIAL 100
#define TRESHOLD 0.5
#define K_TRESHOLD 3 //define o quão próximo o robô deve estar da bola para chutá-la

//-------------
#define WORLD_X 5000
#define WORLD_Y 5000

#define ROBOT_R (ROBOT_RADIUS_MM)
#define BALL_R (BALL_RADIUS_MM)

#define ROBOT_DENSITY 5
#define BALL_DENSITY 0.2

//-------------

class Robot {

	public:
		Robot() { isUpdated=false; };
		~Robot() {};
		
		b2Body* body;
		Vector forces;
		float displacement_angle;
		bool doKick;
		bool doDrible;
		int id;
		bool isUpdated;

		bool pointingToBall();
		bool closeToBall();
};


struct Ball {
	b2Body*  body;
	Vector   _forces;
};


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


// Gambis

void openRadiotosim();
void openSimtotracker();
