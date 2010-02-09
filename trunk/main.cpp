#include <stdio.h>

#include "ssl_client.h"
#include "ssl_server.h"
#include "vector.h"

#define MAX_ROBOTS 10
#define CONSTANTE_DESLOCAMENTO 0.01 //deve ser menor que 1 e maior que 0
#define ROBOT_RADIUS 100
#define CONSTANTE_POSICIONAMENTO_INICIAL 100

RoboCupSSLServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboCupSSLClient aitosim(PORT_AI_TO_SIM, IP_AI_TO_SIM);

int DEBUG = 1;

struct Robot {
   Vector pos, future_pos;
} robot[TEAM_TOTAL][MAX_ROBOTS];
int robot_total[TEAM_TOTAL] = {1, 0}; //0 blue robots and 0 yellow robots

struct Ball {
	Vector pos;
} ball;

void receive()
{
	SSL_WrapperPacket packet;
	if (aitosim.receive(packet) && packet.has_aitosim()) {
		//printf("----------------------------");
		//printf("Received AI-To-SIM! --\n");

		AIToSim data = packet.aitosim();
		int team = data.team();
		for(int i = 0; i < data.robots_size(); i++) {
			robot[team][i].future_pos = Vector(data.robots(i).future_x(), data.robots(i).future_y());
		}
	}
}

void process()
{
    //Vector robot_old_pos[TEAM_TOTAL][MAX_ROBOTS];

	bool changed = true;
	while(changed) { //collision ball <-> robots
		changed = false;
		for(int team = 0; team < TEAM_TOTAL; team++)
			for(int i = 0; i < robot_total[team]; i++) {
			    printf("team %i robot %i distanceToBall %f --\n", team, i, robot[team][i].pos.getDistance(ball.pos));
				if(robot[team][i].pos.getDistance(ball.pos) < ROBOT_RADIUS) {
					Vector dir = (robot[team][i].future_pos - robot[team][i].pos).normalize();
					printf("dir: %f, %f --\n", dir.getX(), dir.getY());
					ball.pos = robot[team][i].pos + dir * ROBOT_RADIUS * 1.1;
					printf("distance: %f --\n", robot[team][i].pos.getDistance(ball.pos));
					assert(robot[team][i].pos.getDistance(ball.pos) >= ROBOT_RADIUS);
					changed = true;
				}
			}
	}

	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < robot_total[team]; i++) {
		    //robot_old_pos[team][i] = robot[team][i].pos;

			Vector dir = (robot[team][i].future_pos - robot[team][i].pos).normalize();
			robot[team][i].pos = robot[team][i].pos + dir * CONSTANTE_DESLOCAMENTO;

			Vector u = robot[team][i].pos;
			printf("cur_pos[%5i](%5i, %5i) --\n", i, (int) u.getX(), (int) u.getY());
			Vector v = robot[team][i].future_pos;
			printf("fut_pos[%5i](%5i, %5i) --\n", i, (int) v.getX(), (int) v.getY());
	}

	printf("ball(%5i, %5i) --\n", (int) ball.pos.getX(), (int) ball.pos.getY());
	//TODO: collision robots <-> robots
}

void send()
{
	 SSL_WrapperPacket packet;

	 SimToTracker *simtotrackerPacket = packet.mutable_simtotracker();
	 BallSim *b = simtotrackerPacket->mutable_ball();

	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < robot_total[team]; i++) {
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

	 printf("packet.robots_blue_size(): %5i --\n", simtotrackerPacket->robots_blue_size());
	 printf("packet.robots_yellow_size(): %5i --\n", simtotrackerPacket->robots_yellow_size());

	printf("Sent Sim-To-Tracker\n");
}

int main()
{
	printf("Simulator Running!\n");

	aitosim.open(false);

	printf("Press <Enter> to open connection with client...\n");
	getchar();
	simtotracker.open();

	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < robot_total[team]; i++) {
			robot[team][i].pos = robot[team][i].future_pos =
				Vector((i + 1) * CONSTANTE_POSICIONAMENTO_INICIAL + team * MAX_ROBOTS * CONSTANTE_POSICIONAMENTO_INICIAL,
						 (MAX_ROBOTS - i) * CONSTANTE_POSICIONAMENTO_INICIAL + team * MAX_ROBOTS * CONSTANTE_POSICIONAMENTO_INICIAL);
		}
	ball.pos = Vector(200, 1100);

	clrscr();
	while(1) {
		//rewindscr();
		receive();
		process();
		send();
	}
}

