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

//-------------
#define WORLD_X 100
#define WORLD_Y 100

#define ROBOT_RADIUS_CM (ROBOT_RADIUS_MM/10)
#define BALL_RADIUS_CM (BALL_RADIUS_MM/10)

#define ROBOT_DENSITY 5
#define BALL_DENSITY 2

//-------------

RoboPETServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboPETClient radiotosim(PORT_RADIO_TO_SIM, IP_RADIO_TO_SIM);

b2World* world;


struct Robot {

	b2Body* body;
	Vector forces;
	float displacement_angle;
	bool doKick;
	bool doDrible;
};

Robot robots[TEAM_TOTAL][MAX_ROBOTS];

int playersTotal[TEAM_TOTAL] = {1, 0};

struct Ball {
	b2Body*  body;
	Vector   _forces;
} ball;


void receive()
{
	RoboPET_WrapperPacket packet;
	if (radiotosim.receive(packet) && packet.has_radiotosim()) {
		printf("----------------------------");
		printf("Received Radio-To-SIM! --\n");

		RadioToSim data = packet.radiotosim();
		
		playersTotal[TEAM_BLUE] = data.yellow_robots_size();
		playersTotal[TEAM_YELLOW] = data.blue_robots_size();
		
		for(int i = 0; i < playersTotal[TEAM_YELLOW]; i++) {
			robots[0][i].forces = Vector(data.yellow_robots(i).force_x(), data.yellow_robots(i).force_y());
			robots[0][i].displacement_angle = data.yellow_robots(i).displacement_theta();
			robots[0][i].doKick = data.yellow_robots(i).kick();
			robots[0][i].doDrible = data.yellow_robots(i).drible();
		}
		for(int i = 0; i < playersTotal[TEAM_BLUE]; i++) {
			robots[1][i].forces = Vector(data.blue_robots(i).force_x(), data.blue_robots(i).force_y());
			robots[1][i].displacement_angle = data.blue_robots(i).displacement_theta();
			robots[1][i].doKick = data.blue_robots(i).kick();
			robots[1][i].doDrible = data.blue_robots(i).drible();
		}
	}

}

void send()
{
	RoboPET_WrapperPacket packet;

	SimToTracker *simtotrackerPacket = packet.mutable_simtotracker();
	SimToTracker::Ball *b = simtotrackerPacket->mutable_ball();

	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < playersTotal[team]; i++) {
	 		SimToTracker::Robot *r =
	 			(team == TEAM_BLUE ?
	 				simtotrackerPacket->add_blue_robots() :
	 				simtotrackerPacket->add_yellow_robots());

	 		r->set_x( robots[team][i].body->GetPosition().x );
	 		r->set_y( robots[team][i].body->GetPosition().y );
	 		r->set_theta(0.0);
	 }

	 b->set_x( ball.body->GetPosition().x );
	 b->set_y( ball.body->GetPosition().y );

	 simtotracker.send(packet);

	 printf("packet.robots_blue_size(): %5i --\n", simtotrackerPacket->blue_robots_size());
	 printf("packet.robots_yellow_size(): %5i --\n", simtotrackerPacket->yellow_robots_size());

	printf("Sent Sim-To-Tracker\n");
}

bool pointingToBall(int i, int j) {

	//ball_vec is the vector from the bot center to the ball center
	RP::Vector ball_vec(robots[i][j].body->GetPosition().x - ball.body->GetPosition().x,
						robots[i][j].body->GetPosition().y - ball.body->GetPosition().y);

	RP::Vector norm_ball_vec = ball_vec.normalize();

	RP::Vector bot_vec( sin(robots[i][j].body->GetAngle())*ROBOT_RADIUS_CM,
						cos(robots[i][j].body->GetAngle())*ROBOT_RADIUS_CM);

	RP::Vector norm_bot_vec =  bot_vec.normalize();
	//ball_vec (dot_product) bot_vec = |bal_vec| * |bot_vec| * cos(theta)
	//|bal_vec| == |bot_vec| == 1
	//if theta == 0, the bot points to the ball

	if(1-TRESHOLD <= ball_vec.dotProduct(norm_bot_vec) && ball_vec.dotProduct(norm_bot_vec) <= 1) {
		return true;
	}

	return false;

}

void process()
{
	b2Vec2 bot_move;
	b2Vec2 ball_move;
	int i, j;

	//iterate to move the bots
	for (i = 0; i < TEAM_TOTAL; i++) {
		for (j = 0; j < playersTotal[i]; j++) {

			bot_move = b2Vec2(robots[i][j].forces.getX(), robots[i][j].forces.getY());

			robots[i][j].body->ApplyForce(bot_move, robots[i][j].body->GetWorldCenter());

			//rotates the bot
			//bool SetTransform(const b2Vec2& position, float32 angle);
			robots[i][j].body->SetTransform(robots[i][j].body->GetPosition(), robots[i][j].body->GetAngle()+robots[i][j].displacement_angle);


			if(robots[i][j].doKick) {

				//The bot needs to "point" to the ball to kick it
				if(pointingToBall(i, j)) {
					//crazy values. We need to measure them afer
					Vector ball_force(ball.body->GetPosition().x - robots[i][j].body->GetPosition().x,
					   				  ball.body->GetPosition().y - robots[i][j].body->GetPosition().y);

					int kickForce = 10;
					ball_move = b2Vec2(ball_force.getX() * kickForce, ball_force.getY() * kickForce);
					ball.body->ApplyForce(ball_move, ball.body->GetPosition());
				}
			}
		}
	}

	//Here, move the ball


	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	float32 timeStep = 1.0f / 60.0f;
	int32 velocityIterations = 10;
	int32 positionIterations = 10;


	// Instruct the world to perform a single step of simulation. It is
	// generally best to keep the time step and iterations fixed.
	world->Step(timeStep, velocityIterations, positionIterations);
	
	// Clear applied body forces. We didn't apply any forces, but you
	// should know about this function.
	world->ClearForces();
}


// create a box physics object which will be used for the walls
b2Body* newWall(float x, float y, float sizex, float sizey)
{
	b2BodyDef bodyDef;
	bodyDef.position.Set(x, y);

	b2Body* body = world->CreateBody(&bodyDef);

	b2PolygonShape bodyShapeDef;
	bodyShapeDef.SetAsBox(sizex, sizey);

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &bodyShapeDef;
	fixtureDef.density = 1.0f;
	fixtureDef.restitution = 0.8;
	body->CreateFixture(&fixtureDef);


	return body;
}

// create a physics circular object which will be used for the ball and robots
b2Body* newDynamicCircle(float x, float y, float radius, float density, float friction, float restitution, float damping, b2Color color)
{
	b2Vec2 pos(x,y);
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = pos;
    bodyDef.linearDamping = damping;
    bodyDef.angularDamping = damping;

	b2Body* body = world->CreateBody(&bodyDef);

	b2CircleShape circle;
	circle.m_radius = radius;
	circle.m_p.Set(0,0);

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &circle;
	fixtureDef.friction = friction;
	fixtureDef.density = density;
	fixtureDef.restitution = 0.8;
	body->CreateFixture(&fixtureDef);

	cout << "mass: " << body->GetMass()/100 << "kg" << endl;

    return body;
}


void initObjects()
{
	newWall(0,WORLD_Y, WORLD_X,1); 	// top wall
	newWall(0,-WORLD_Y, WORLD_X,1); // bottom wall
	newWall(WORLD_X,0, 1,WORLD_Y); 	// right wall
	newWall(-WORLD_X,0, 1,WORLD_Y); // left wall

	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < playersTotal[team]; i++) {
			robots[team][i].body = newDynamicCircle( //((i + 1) * CONSTANTE_POSICIONAMENTO_INICIAL + team * MAX_ROBOTS * CONSTANTE_POSICIONAMENTO_INICIAL),
															  //((MAX_ROBOTS - i) * CONSTANTE_POSICIONAMENTO_INICIAL + team * MAX_ROBOTS * CONSTANTE_POSICIONAMENTO_INICIAL),
															  0,0,
															  ROBOT_RADIUS_CM, ROBOT_DENSITY, 1, 0.5, 0.05, b2Color(1,0,0));
		}

    ball.body = newDynamicCircle( 0, 0,BALL_RADIUS_CM, BALL_DENSITY, 1, 0.9, 0.03, b2Color(1,0,0));
}

// glut keyboard keys
void keyboardFunc(unsigned char key, int xmouse, int ymouse)
{
		b2Vec2 fv;
		int force = 100000; // Newtons/100
		
        if( key == 'a' ) {
			fv = b2Vec2(-force,0);
        }
        
        if( key == 'd' ) {
	        fv = b2Vec2(force,0);
        }
        
        if( key == 's' ) {
            fv = b2Vec2(0,-force);
        }
        
        if( key == 'w' ) {
            fv = b2Vec2(0,force);
        }
        			
		robots[0][0].body->ApplyForce(fv,robots[0][0].body->GetWorldCenter());
		
		if( key == 'i' ) {
			robots[0][0].doKick = !robots[0][0].doKick;
			cout<<"kick="<<robots[0][0].doKick<<endl;
		}
		
		if( key == 'k' ) {
			robots[0][0].doDrible = !robots[0][0].doDrible;
			cout<<"drible="<<robots[0][0].doDrible<<endl;
		}		
		
		if( key == 'j' ) {
			robots[0][0].body->ApplyTorque( force );
		}
		
		if( key == 'l' ) {
			robots[0][0].body->ApplyTorque( -force );
		}
}

// glut drawing function
void drawScene()
{
	glMatrixMode (GL_PROJECTION);
	glViewport(0,0, 500,500);
	glLoadIdentity ();
	gluOrtho2D(-WORLD_X, WORLD_X, -WORLD_Y, WORLD_Y);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity ();
	
	glClear(GL_COLOR_BUFFER_BIT);


	// draw players
    for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < playersTotal[team]; i++)
		{
			b2Vec2 position = robots[team][i].body->GetPosition();
			float angle = robots[team][i].body->GetAngle();

			float arad;
			glBegin(GL_LINE_LOOP);
				for(float ang = 0; ang < 360; ang+=10)
				{
					arad = ang * M_PI / 180.0;
					glVertex2f( position.x + cos(arad)*ROBOT_RADIUS_CM,
								position.y + sin(arad)*ROBOT_RADIUS_CM);
				}
			glEnd();
			drawLine(position.x , position.y, position.x + cos(angle) * ROBOT_RADIUS_CM , position.y + sin(angle) * ROBOT_RADIUS_CM);
		}
	
	// draw ball
	float arad;
	glBegin(GL_LINE_LOOP);
		for(float ang = 0; ang < 360; ang+=10)
		{
			b2Vec2 position = ball.body->GetPosition();
			
			arad = ang * M_PI / 180.0;
			glVertex2f( position.x + cos(arad)*BALL_RADIUS_CM,
						position.y + sin(arad)*BALL_RADIUS_CM);
		}
	glEnd();
	
	glutSwapBuffers();
	
	// SIMULATOR MAIN LOOP
		/*clrscr();
		static int scrCount = 0;
		scrCount++;
		if(scrCount == SCR_CLEAR_DELAY) {
			scrCount = 0;
			clrscr();
		}
		rewindscr();
		receive();
		*/
		process();
		//send();
		
}

void initWorld()
{
	// b2World( b2Vec2 &gravityVector, bool doSleep )
	world = new b2World(b2Vec2(0,0), false);
}

void initGlut(int argc, char** argv)
{
	glutInit (&argc, argv);
    glutInitDisplayMode ( GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA );
    glutInitWindowSize (500,500);
    glutInitWindowPosition (500, 500);
    glutCreateWindow("RoboPET Simulator - controls: WASD, IJKL");
    
	glutIdleFunc (drawScene);
	glutKeyboardFunc(keyboardFunc);
	
	glutMainLoop();
}

int main(int argc, char** argv)
{
	initWorld();
	initObjects();

	printf("Simulator Running!\n");

	radiotosim.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();
	simtotracker.open();

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
