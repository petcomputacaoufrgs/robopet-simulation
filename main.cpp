#include <stdio.h>

#include "ssl_client.h"
#include "ssl_server.h"
#include "vector.h"

#define MAX_ROBOTS 10
#define CONSTANTE_DESLOCAMENTO 0.1 //deve ser menor que 1 e maior que 0
#define ROBOT_RADIUS 1
#define CONSTANTE_POSICIONAMENTO_INICIAL 10

RoboCupSSLServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboCupSSLClient aitosim(PORT_AI_TO_SIM, IP_AI_TO_SIM);

int DEBUG = 1;

struct Robot {
   Vector pos, future_pos;
} robot[TEAM_TOTAL][MAX_ROBOTS];

struct Ball {
	Vector pos;
} ball;

void receive()
{
	SSL_WrapperPacket packet;
	if (aitosim.receive(packet) && packet.has_aitosim()) {
		printf("----------------------------");
		printf("Received AI-To-SIM!\n");

		AIToSim data = packet.aitosim();
		int team = data.team();
		for(int i = 0; i < data.robots_size(); i++) {
			robot[team][i].future_pos = Vector(data.robots(i).future_x(), data.robots(i).future_y());
		}
	}
}

void process()
{
	for(int i = 0; i < MAX_ROBOTS; i++)
		for(int team = 0; team < TEAM_TOTAL; team++) {
	   Vector dir = (robot[team][i].future_pos - robot[team][i].pos).normalize();
		robot[team][i].pos = robot[team][i].pos + dir * CONSTANTE_DESLOCAMENTO;
	}
	
	bool changed = true;
	while(changed) { //collision ball <-> robots
		changed = false;
		for(int i = 0; i < MAX_ROBOTS && !changed; i++) 
			for(int team = 0; team < TEAM_TOTAL && !changed; team++) {
				if(robot[team][i].pos.getDistance(ball.pos) < ROBOT_RADIUS) {
					Vector dir = (robot[team][i].future_pos - robot[team][i].pos).normalize();
					ball.pos = robot[team][i].pos + dir * ROBOT_RADIUS;
					changed = true;
				}
			}
	}
	//TODO: collision robots <-> robots
}

void send()
{
	 SSL_WrapperPacket packet;

	 SimToTracker *simtotrackerPacket = packet.mutable_simtotracker();
	 BallSim *b = simtotrackerPacket->mutable_ball();
	 
	 for(int i = 0; i < MAX_ROBOTS; i++)
	 	for(int team = 0; team < TEAM_TOTAL; team++) {
	 		RobotSim *r = 
	 			(team == TEAM_BLUE ?
	 				simtotrackerPacket->add_robots_blue() :
	 				simtotrackerPacket->add_robots_yellow());
	 				
	 		r->set_x(robot[team][i].pos.getX());
	 		r->set_y(robot[team][i].pos.getY());
	 		r->set_theta(0.0);
	 }
	 
	 b->set_x(ball.pos.getX());
	 b->set_y(ball.pos.getY());

	 simtotracker.send(packet);
	printf("Sent Sim-To-Tracker\n");
}

int main()
{
	printf("Simulator Running!\n");

	aitosim.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();
	simtotracker.open();

	for(int i = 0; i < MAX_ROBOTS; i++) 
		for(int team = 0; team < TEAM_TOTAL; team++) {
			robot[team][i].pos = robot[team][i].future_pos =
				Vector(i * CONSTANTE_POSICIONAMENTO_INICIAL + team * MAX_ROBOTS * CONSTANTE_POSICIONAMENTO_INICIAL,
						 (MAX_ROBOTS - i) * CONSTANTE_POSICIONAMENTO_INICIAL + team * MAX_ROBOTS * CONSTANTE_POSICIONAMENTO_INICIAL);
		}
	ball.pos = Vector(17, 17);

	while(1) {
		receive();
		process();
		send();
	}
}
