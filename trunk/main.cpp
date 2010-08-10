#include <stdio.h>
#include <iostream>
using namespace std;

#include "rp_client.h"
#include "rp_server.h"
#include "vector.h"
#include "Box2D.h"

#define MAX_ROBOTS 5
#define CONSTANTE_DESLOCAMENTO 0.01 //deve ser menor que 1 e maior que 0
#define ROBOT_RADIUS 100
#define CONSTANTE_POSICIONAMENTO_INICIAL 100

//-------------
#define WORLD_X 1000
#define WORLD_Y 1000

#define ROBOT_R 50
#define ROBOT_DENSITY 0.1

#define BALL_R 10
//-------------

RoboPETServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboPETClient radiotosim(PORT_RADIO_TO_SIM, IP_RADIO_TO_SIM);

b2World* world;


struct Robot {
	b2Body* body;
	Vector forces;
	float angle;
	bool doKick;
	bool doDrible;
};

Robot robots[TEAM_TOTAL][MAX_ROBOTS];

int robot_total[TEAM_TOTAL] = {1, 1};

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
		//Blue team == 0, Yellow team == 1 ??
		//Yellow team == 0, Blue team == 1 ??
		int team = data.team();
		for(int i = 0; i < data.robots_size(); i++) {
			robots[team][i].forces = Vector(data.robots(i).force_x(), data.robots(i).force_y());
			robots[team][i].angle = data.robots(i).displacement_theta();
			robots[team][i].doKick = data.robots(i).kick();
			robots[team][i].doDrible = data.robots(i).drible();
		}
	}

}

void send()
{
	RoboPET_WrapperPacket packet;

	SimToTracker *simtotrackerPacket = packet.mutable_simtotracker();
	SimToTracker::Ball *b = simtotrackerPacket->mutable_ball();

	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < robot_total[team]; i++) {
	 		SimToTracker::Robot *r =
	 			(team == TEAM_BLUE ?
	 				simtotrackerPacket->add_blue_robots() :
	 				simtotrackerPacket->add_yellow_robots());

	 		r->set_x( robots[team][i].body->GetWorldPoint(b2Vec2(0,0)).x );
	 		r->set_y( robots[team][i].body->GetWorldPoint(b2Vec2(0,0)).y );
	 		r->set_theta(0.0);
	 }

	 b->set_x( ball.body->GetWorldPoint(b2Vec2(0,0)).x );
	 b->set_y( ball.body->GetWorldPoint(b2Vec2(0,0)).y );

	 simtotracker.send(packet);

	 printf("packet.robots_blue_size(): %5i --\n", simtotrackerPacket->blue_robots_size());
	 printf("packet.robots_yellow_size(): %5i --\n", simtotrackerPacket->yellow_robots_size());

	printf("Sent Sim-To-Tracker\n");
}


void process()
{
	b2Vec2 bot_move;

	int i, j;

	//iterate to move the bots
	for (i = 0; i < TEAM_TOTAL; i++) {
		for (j = 0; j < MAX_ROBOTS; j++) {

			bot_move = b2Vec2( robots[i][j].forces.getX(),robots[i][j].forces.getY() );

			robots[i][j].body->ApplyForce(bot_move,robots[i][j].body->GetWorldCenter());
			//robots[i][j].body->ApplyImpulse(bot_move,robots[i][j].body->GetWorldCenter());
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
	b2BodyDef bd;
	bd.position = pos;
    bd.linearDamping = damping;

	b2Body* body = world->CreateBody(&bd);
	
	b2CircleShape circle;
	circle.m_radius = radius;
	circle.m_p.Set(0,0);
	
	b2FixtureDef fixtureDef;
	fixtureDef.shape = &circle;
	fixtureDef.friction = friction;
	fixtureDef.density = density;
	fixtureDef.restitution = 0.8;
	body->CreateFixture(&fixtureDef);

	cout << "mass: " << body->GetMass() <<endl;

    return body;
}


void initObjects()
{
	newWall(0,WORLD_Y, WORLD_X,1); 	// top wall
	newWall(0,-WORLD_Y, WORLD_X,1); // bottom wall
	newWall(WORLD_X,0, 1,WORLD_Y); 	// right wall
	newWall(-WORLD_X,0, 1,WORLD_Y); // left wall

	int objCounter = 0;
	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < robot_total[team]; i++) {
			robots[team][objCounter].body = newDynamicCircle( //((i + 1) * CONSTANTE_POSICIONAMENTO_INICIAL + team * MAX_ROBOTS * CONSTANTE_POSICIONAMENTO_INICIAL),
															  //((MAX_ROBOTS - i) * CONSTANTE_POSICIONAMENTO_INICIAL + team * MAX_ROBOTS * CONSTANTE_POSICIONAMENTO_INICIAL),
															  0,0,
															  ROBOT_R, ROBOT_DENSITY, 1, 0.5, 0.05, b2Color(1,0,0));
			objCounter++;
		}

    ball.body = newDynamicCircle( 0, 0,BALL_R, 0.5, 1, 0.9, 0.03, b2Color(1,0,0));
}

void initWorld()
{
	// b2Vec2 &gravityVector, bool doSleep
	world = new b2World(b2Vec2(0,0), false);
}

int main()
{
	initWorld();
	initObjects();

	printf("Simulator Running!\n");

	radiotosim.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();
	simtotracker.open();

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
	}
}
