#include "simulation.h"

RoboPETServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboPETClient radiotosim(PORT_RADIO_TO_SIM, IP_RADIO_TO_SIM);

Robot robots[TEAM_TOTAL][MAX_ROBOTS];
Ball ball;
int playersTotal[TEAM_TOTAL] = {1, 0};
b2World* world;

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
			cout<<"kick="<<robots[0][0].doKick<<endl;
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

int main(int argc, char** argv) {

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
