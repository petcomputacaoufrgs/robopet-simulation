#include "simulation.h"


Robot robots[TEAM_TOTAL][MAX_ROBOTS];
Ball ball;
int playersTotal[TEAM_TOTAL] = {MAX_ROBOTS, MAX_ROBOTS};
b2World* world;

RoboPETServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboPETClient radiotosim(PORT_RADIO_TO_SIM, IP_RADIO_TO_SIM);

bool Robot::pointingToBall() {

	//ball_vec is the vector from the bot center to the ball center
	RP::Vector ball_vec(ball.body->GetPosition().x - body->GetPosition().x,
						ball.body->GetPosition().y - body->GetPosition().y);

	ball_vec.normalizeMe();

	//bot_vec indicates where the bot is pointing
	RP::Vector bot_vec( cos(body->GetAngle())*ROBOT_R,
						sin(body->GetAngle())*ROBOT_R);

	bot_vec.normalizeMe();
	//ball_vec (dot_product) bot_vec = |bal_vec| * |bot_vec| * cos(theta)
	//|bal_vec| == |bot_vec| == 1, because both are normalized
	//if theta ~ 0, the bot points to the ball

	if(1-TRESHOLD <= ball_vec.dotProduct(bot_vec) &&
	   ball_vec.dotProduct(bot_vec) <= 1) {
		return true;
	}

	return false;

}

bool Robot::closeToBall() {

	//ball_vec is the vector from the bot center to the ball center
	RP::Vector ball_vec(ball.body->GetPosition().x - body->GetPosition().x,
						ball.body->GetPosition().y - body->GetPosition().y);

	//if the distance from the bot center to the ball center is (almost) zero, bot's close to the ball
	if(ball_vec.getNorm() <= ROBOT_R + BALL_R + K_TRESHOLD) {
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
			//cout << i << "||" << j << endl;
			bot_move = b2Vec2(robots[i][j].forces.getX(), robots[i][j].forces.getY());


			robots[i][j].body->ApplyForce(bot_move, robots[i][j].body->GetWorldCenter());
			//rotates the botRobot robots[TEAM_TOTAL][MAX_ROBOTS];
			//bool SetTransform(const b2Vec2& position, float32 angle);
			robots[i][j].body->SetTransform(robots[i][j].body->GetPosition(), robots[i][j].body->GetAngle()+robots[i][j].displacement_angle);

			if(robots[i][j].closeToBall()) {

				//The bot needs to "point" to the ball to kick it
				if(robots[i][j].pointingToBall()) {

					if(robots[i][j].doKick) {
						//crazy values. We need to measure them afer
						Vector ball_force(	ball.body->GetPosition().x - robots[i][j].body->GetPosition().x,
					   				  		ball.body->GetPosition().y - robots[i][j].body->GetPosition().y);

						int kickForce = 10000; //newtons*100
						ball_move = b2Vec2(ball_force.getX() * kickForce, ball_force.getY() * kickForce);
						ball.body->ApplyForce(ball_move, ball.body->GetPosition());
						robots[i][j].doKick = 0;
					}
					if(robots[i][j].doDrible){
						Vector ball_force(	robots[i][j].body->GetPosition().x - ball.body->GetPosition().x,
					   				  		robots[i][j].body->GetPosition().y - ball.body->GetPosition().y);
						int dribleForce = 100/4;
						ball_move = b2Vec2(ball_force.getX() * dribleForce, ball_force.getY() * dribleForce);
						ball.body->ApplyForce(ball_move, ball.body->GetPosition());
					}
				}
			}
			robots[i][j].doKick = 0;
		}
	}

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

	cout << "calculated mass: " << body->GetMass()*100 << "kg" << endl;

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
															  ROBOT_R, ROBOT_DENSITY, 1, 0.5, 0.1, b2Color(1,0,0));
		}

    ball.body = newDynamicCircle( 0, 0,BALL_R, BALL_DENSITY, 1, 0.9, 0.1, b2Color(1,0,0));
}

void keyboardFunc(unsigned char key, int xmouse, int ymouse)
{
		b2Vec2 fv;
		int force = 100000000/5; // Newtons*100

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
			robots[0][0].doKick = 1;
			cout<<"kick="<<robots[0][0].doKick<<endl;
		}

		if( key == 'k' ) {
			robots[0][0].doDrible = !robots[0][0].doDrible;
			cout<<"drible="<<robots[0][0].doDrible<<endl;
		}
		#include "Box2D.h"
		if( key == 'j' ) {
			robots[0][0].body->ApplyTorque( force );
		}

		if( key == 'l' ) {
			robots[0][0].body->ApplyTorque( -force );
		}
}

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
					glVertex2f( position.x + cos(arad)*ROBOT_R,
								position.y + sin(arad)*ROBOT_R);
				}
			glEnd();
			drawLine(position.x , position.y, position.x + cos(angle) * ROBOT_R , position.y + sin(angle) * ROBOT_R);
		}

	// draw ball
	float arad;
	glBegin(GL_LINE_LOOP);
		for(float ang = 0; ang < 360; ang+=10)
		{
			b2Vec2 position = ball.body->GetPosition();

			arad = ang * M_PI / 180.0;
			glVertex2f( position.x + cos(arad)*BALL_R,
						position.y + sin(arad)*BALL_R);
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
		rewindscr();*/
		receive();
		process();
		send();

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

void receive()
{
	RoboPET_WrapperPacket packet;
	if (radiotosim.receive(packet) && packet.has_radiotosim()) {
		printf("----------------------------");
		printf("Received Radio-To-SIM!\n");

		RadioToSim data = packet.radiotosim();

		playersTotal[data.team_id()] = data.robots_size();

		for(int i = 0; i < playersTotal[data.team_id()]; i++) {
			robots[data.team_id()][i].forces = Vector(data.robots(i).force_x(), data.robots(i).force_y());
			robots[data.team_id()][i].displacement_angle = data.robots(i).displacement_theta();
			robots[data.team_id()][i].doKick = data.robots(i).kick();
			robots[data.team_id()][i].doDrible = data.robots(i).drible();
			robots[data.team_id()][i].id = data.robots(i).id();
		}
		
		printf("oi. recebi %i jogadores do time %i.\n",playersTotal[data.team_id()],data.team_id());
		
		
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
	 		r->set_id( robots[team][i].id );
	 }

	 b->set_x( ball.body->GetPosition().x );
	 b->set_y( ball.body->GetPosition().y );

	 simtotracker.send(packet);

	 //printf("packet.robots_blue_size() = %5i\n", simtotrackerPacket->blue_robots_size());
	 //printf("packet.robots_yellow_size() = %5i\n", simtotrackerPacket->yellow_robots_size());

	printf("Sent Sim-To-Tracker\n");
}


void openRadiotosim() {

	radiotosim.open(false);
}
void openSimtotracker() {

	simtotracker.open();
}
