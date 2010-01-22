#include <stdio.h>

#include "ssl_client.h"
#include "ssl_server.h"

RoboCupSSLServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboCupSSLClient aitosim(PORT_AI_TO_SIM, IP_AI_TO_SIM);

int DEBUG = 1;

void receive()
{
	SSL_WrapperPacket packet;
	if (aitosim.receive(packet) && packet.has_aitosim()) {
		printf("----------------------------");
		printf("Received AI-To-SIM!\n");
	}
}

void send()
{
	 SSL_WrapperPacket packet;

	 SimToTracker *simtotrackerPacket = packet.mutable_simtotracker();
	 BallSim *b = simtotrackerPacket->mutable_ball();

	 b->set_x(77);
	 b->set_y(666);

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

	while(1) {
		receive();
		send();
	}
}
