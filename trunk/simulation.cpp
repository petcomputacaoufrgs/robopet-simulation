#include "simulation.h"
#include "math.h"


////////////////////////////////////////////////////////////////////////

#define MAX_ROBOTS 10
#define K_TRESHOLD 3 //how close to the ball the bot should be to kick it
#define KICKFORCE 50 //how strong it should be?
#define DRIBBLEFORCE 5 

//-------------

#define ROBOT_R MM_TO_M(ROBOT_RADIUS_MM)
#define BALL_R MM_TO_M(BALL_RADIUS_MM)

//-------------

#define WORLD_X MM_TO_M(ARENA_WIDTH_MM)
#define WORLD_Y MM_TO_M(ARENA_HEIGHT_MM)
#define FIELD_X MM_TO_M(FIELD_WIDTH_MM)
#define FIELD_Y MM_TO_M(FIELD_HEIGHT_MM)
#define ARENA_BORDER MM_TO_M(BORDER)

#define WINDOW_X 740
#define WINDOW_Y 540


////////////////////////////////////////////////////////////////////////

float MOTOR_FORCE = 10;

float ROBOT_DENSITY = 0.02;
float BALL_DENSITY = 0.002; // the ball should weigh approximately 46 g
float BALL_DAMP = 1; // 0 - 1
float ROBOT_DAMP = 1;

//-------------

Robot robots[TEAM_TOTAL][MAX_ROBOTS];
Ball ball;
int playersTotal[TEAM_TOTAL] = {2,5};
b2World* world;

RoboPETServer simtotracker(PORT_SIM_TO_TRACKER, IP_SIM_TO_TRACKER);
RoboPETClient radiotosim(PORT_RADIO_TO_SIM, IP_RADIO_TO_SIM);


////////////////////////////////////////////////////////////////////////

bool Robot::pointingToBall() 
{
    //NOTE:both cmath and box2d computes trigonometric functions using angles in rad.

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
	//if theta ~ 0 (cos(theta) ~ 1), the bot points to the ball

	if(1-TRESHOLD <= ball_vec.dotProduct(bot_vec)) {
		return true;
	}

	return false;
}

bool Robot::closeToBall() 
{
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

					//this is the vector of the bot[i][j] movement
					bot_move = b2Vec2(MOTOR_FORCE*robots[i][j].forces.getX(), MOTOR_FORCE*robots[i][j].forces.getY());
					robots[i][j].body->ApplyForce(bot_move, robots[i][j].body->GetWorldCenter());
					
					//printf("APPLIED FORCE: robot %i - <%f, %f>\n",j, bot_move.x,bot_move.y);
					//printf("VELOCIDADE: robot %i - <%f, %f>\n", j, robots[i][j].body->GetLinearVelocityFromWorldPoint(b2Vec2(0,0)).x, robots[i][j].body->GetLinearVelocityFromWorldPoint(b2Vec2(0,0)).y);

					//rotates the botRobot
					robots[i][j].body->SetTransform(robots[i][j].body->GetPosition(), robots[i][j].body->GetAngle()+robots[i][j].displacement_angle);

					//Here we test if the bot is close to the ball and near it.
					//If so, and it wants to kick or dribble, we do it!
					if(robots[i][j].closeToBall() && robots[i][j].pointingToBall()) {
							if(robots[i][j].doKick) {
									//crazy values. We need to measure them afer
									Vector ball_force(	ball.body->GetPosition().x - robots[i][j].body->GetPosition().x,
													    ball.body->GetPosition().y - robots[i][j].body->GetPosition().y);

									ball_move = b2Vec2(ball_force.getX() * KICKFORCE, ball_force.getY() * KICKFORCE);
									ball.body->ApplyForce(ball_move, ball.body->GetPosition());
									//we need to clear the kick command to let the bot kick again
									robots[i][j].doKick = 0;
							}
							else if(robots[i][j].doDribble){
									Vector ball_force(robots[i][j].body->GetPosition().x - ball.body->GetPosition().x,
													  robots[i][j].body->GetPosition().y - ball.body->GetPosition().y);

									ball_move = b2Vec2(ball_force.getX() * DRIBBLEFORCE, ball_force.getY() * DRIBBLEFORCE);
									ball.body->ApplyForce(ball_move, ball.body->GetPosition());
							}
					}
			}
	}

	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	float32 timeStep = 1.0f / 60.0f;
	int32 velocityIterations = 10;
	int positionIterations = 10;

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

b2Body* newDynamicCircle(float x, float y, float radius, float density, float friction, float restitution, float damping, float lindamping)
{
	b2Vec2 pos(x,y);
	b2BodyDef bodyDef;
	bodyDef.type = b2_dynamicBody;
	bodyDef.position = pos;
    bodyDef.linearDamping = lindamping;
    bodyDef.angularDamping = damping;

	b2Body* body = world->CreateBody(&bodyDef);

	b2CircleShape circle;
	circle.m_radius = radius;
	circle.m_p.Set(0,0);

	b2FixtureDef fixtureDef;
	fixtureDef.shape = &circle;
	fixtureDef.friction = friction;
	fixtureDef.density = density;
	fixtureDef.restitution = restitution;
	body->CreateFixture(&fixtureDef);

	cout << "calculated mass: " << body->GetMass()*100 << "kg" << endl;

    return body;
}

void initObjects()
{
	// world walls
	newWall(0,WORLD_Y+1, WORLD_X,1); 	// top wall
	newWall(0,-1, WORLD_X,1); 			// bottom wall
	newWall(WORLD_X+1,0, 1,WORLD_Y); 	// right wall
	newWall(-1, 0, 1, WORLD_Y); 	    // left wall
	
    // left goal walls
	newWall(ARENA_BORDER-1, (WORLD_Y/2)-3.7, 1, .2);	// top wall
	newWall(ARENA_BORDER-1, (WORLD_Y/2)+3.5, 1, .2);	// botton wall
	newWall(ARENA_BORDER-1.9, (WORLD_Y/2), .2, 3.5);	// vertical wall

	// right goal walls
	newWall(FIELD_X+ARENA_BORDER+1, (WORLD_Y/2)-3.7, 1, .2);	// top wall
	newWall(FIELD_X+ARENA_BORDER+1, (WORLD_Y/2)+3.5, 1, .2);	// bottom wall
	newWall((FIELD_X+ARENA_BORDER)+1.9, (WORLD_Y/2), .2, 3.5);	// vertical wall
	
	// players
	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < playersTotal[team]; i++) {  // robots must be initialized inside the field boundaries
			robots[team][i].body = newDynamicCircle((rand()%(int)FIELD_X)+ARENA_BORDER, 
													(rand()%(int)FIELD_Y)+ARENA_BORDER,
													ROBOT_R, ROBOT_DENSITY, 1, 1, ROBOT_DAMP, .8);
			robots[team][i].id = i;
		}

	// ball (initialized at the center of the field)
    ball.body = newDynamicCircle( WORLD_X/2, WORLD_Y/2,BALL_R, BALL_DENSITY, 1, 1, BALL_DAMP, 0.05);
}

void resetBall()
{	
	ball.body->SetTransform( b2Vec2(WORLD_X/2, WORLD_Y/2), 0 );
	ball.body->SetLinearVelocity( b2Vec2(0,0) );	
}

void resetPlayers() 
{
	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < playersTotal[team]; i++) { // robots must be initialized inside the field boundaries
			robots[team][i].body->SetTransform( b2Vec2((rand()%(int)FIELD_X)+ARENA_BORDER, (rand()%(int)FIELD_Y)+ARENA_BORDER), 0 );
			robots[team][i].body->SetLinearVelocity( b2Vec2(0,0) );
		}		
}

void keyboardFunc(unsigned char key, int xmouse, int ymouse)
{
		// Ball manual control

		b2Vec2 fv;
		float force = 0.02; // Newtons*100

        if( key == 'a' ) {
			fv = b2Vec2(-force, 0);
        }

        if( key == 'd' ) {
			fv = b2Vec2(force, 0);
        }

        if( key == 's' ) {
            fv = b2Vec2(0, force);
        }

        if( key == 'w' ) {
            fv = b2Vec2(0, -force);
        }

        ball.body->ApplyForce(fv,ball.body->GetWorldCenter());
		
        // Other
        if( key == 'r' ) {
            resetBall();
        }
        
        if( key == 'R' ) {
			resetBall();
			resetPlayers();
		}

		/*
		// robot manual control
		if( key == 'i' ) {
				robots[0][0].doKick = 1;
				cout<<"kick="<<robots[0][0].doKick<<endl;
		}

		if( key == 'k' ) {
				robots[0][0].doDribble = !robots[0][0].doDribble;
				cout<<"dribble="<<robots[0][0].doDribble<<endl;
		}
		if( key == 'j' ) {
				robots[0][0].body->ApplyTorque( force );
		}

		if( key == 'l' ) {
				robots[0][0].body->ApplyTorque( -force );
		}*/
}

void drawField()
{
	drawLine(ARENA_BORDER, FIELD_Y+ARENA_BORDER, FIELD_X+ARENA_BORDER, FIELD_Y+ARENA_BORDER); // top line
    drawLine(ARENA_BORDER, ARENA_BORDER, FIELD_X+ARENA_BORDER, ARENA_BORDER); // botton line
    
    drawLine(FIELD_X+ARENA_BORDER, ARENA_BORDER, FIELD_X+ARENA_BORDER, (WORLD_Y/2)-3.5); // right line down
    drawLine(FIELD_X+ARENA_BORDER, (WORLD_Y/2)+3.5, FIELD_X+ARENA_BORDER, FIELD_Y+ARENA_BORDER); // right line up
    
    // right goal
    drawLine(FIELD_X+ARENA_BORDER, (WORLD_Y/2)-3.5, (FIELD_X+ARENA_BORDER)+1.8, (WORLD_Y/2)-3.5); // horizontal bottom line
    drawLine(FIELD_X+ARENA_BORDER, (WORLD_Y/2)+3.5, (FIELD_X+ARENA_BORDER)+1.8, (WORLD_Y/2)+3.5); // horizontal top line
    drawLine((FIELD_X+ARENA_BORDER)+1.8, (WORLD_Y/2)-3.5, (FIELD_X+ARENA_BORDER)+1.8, (WORLD_Y/2)+3.5); // vertical line
    
    // right area
    drawLine(FIELD_X+ARENA_BORDER, (WORLD_Y/2)-6.5, (FIELD_X+ARENA_BORDER)-4.5, (WORLD_Y/2)-6.5); // horizontal line down
    drawLine(FIELD_X+ARENA_BORDER, (WORLD_Y/2)+6.5, (FIELD_X+ARENA_BORDER)-4.5, (WORLD_Y/2)+6.5); // horizontal line up
    drawLine((FIELD_X+ARENA_BORDER)-4.5, (WORLD_Y/2)-6.5, (FIELD_X+ARENA_BORDER)-4.5, (WORLD_Y/2)+6.5); // vertical line
    
    drawLine(ARENA_BORDER, ARENA_BORDER, ARENA_BORDER, (WORLD_Y/2)-3.5); // left line down
    drawLine(ARENA_BORDER, (WORLD_Y/2)+3.5, ARENA_BORDER, FIELD_Y+ARENA_BORDER); // left line up
    
    // Left goal
    drawLine(ARENA_BORDER, (WORLD_Y/2)-3.5, ARENA_BORDER-1.8, (WORLD_Y/2)-3.5); // horizontal botton line
    drawLine(ARENA_BORDER, (WORLD_Y/2)+3.5, ARENA_BORDER-1.8, (WORLD_Y/2)+3.5); // horizontal line up
    drawLine(ARENA_BORDER-1.8, (WORLD_Y/2)-3.5, ARENA_BORDER-1.8, (WORLD_Y/2)+3.5); // vertical line
    
	// left area
	drawLine(ARENA_BORDER, (WORLD_Y/2)-6.5, ARENA_BORDER+4.5, (WORLD_Y/2)-6.5); // horizontal line down
    drawLine(ARENA_BORDER, (WORLD_Y/2)+6.5, ARENA_BORDER+4.5, (WORLD_Y/2)+6.5); // horizontal line up
    drawLine(ARENA_BORDER+4.5, (WORLD_Y/2)-6.5, ARENA_BORDER+4.5, (WORLD_Y/2)+6.5); // vertical line
            
    drawLine(WORLD_X/2, ARENA_BORDER, WORLD_X/2, WORLD_Y-ARENA_BORDER) // center line
    
    // center cirlce
    float arad = 0.0;
    glBegin(GL_LINE_LOOP);
		for(float ang = 0; ang < 360; ang+=10) {
			arad = ang * M_PI / 180.0;
			glVertex2f( (WORLD_X/2) + cos(arad)*5,
						(WORLD_Y/2) + sin(arad)*5);
		}
	glEnd();
}

void iterate()
{
	float arad;
	
	glMatrixMode (GL_PROJECTION);
	glViewport(0,0, WINDOW_X,WINDOW_Y);
    glLoadIdentity ();
    gluOrtho2D(0, WORLD_X, 0, WORLD_Y);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity ();

	glClear(GL_COLOR_BUFFER_BIT);
	
	//draw the field
	drawField();

	// draw players
    for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < playersTotal[team]; i++) {

			b2Vec2 position = robots[team][i].body->GetPosition();
			position.y = WORLD_Y - position.y; //inverte o Y pra que ele cresça pra baixo
			float angle = robots[team][i].body->GetAngle();

			// draw body
			arad = 0.0;
			glBegin(GL_LINE_LOOP);
				for(float ang = 0; ang < 360; ang+=10)
				{
					arad = ang * M_PI / 180.0;
					glVertex2f( position.x + cos(arad)*ROBOT_R,
								position.y + sin(arad)*ROBOT_R);
				}
			glEnd();
			
			// draw radius
			drawLine(position.x , position.y,
					position.x + cos(angle) * ROBOT_R , position.y + sin(angle) * ROBOT_R);
			
			// draw force vector
			//float vsize = 2.5;
			//drawLine(position.x , position.y,
				//	position.x + vsize*robots[team][i].forces.getX(), position.y - vsize*robots[team][i].forces.getY());
			glColor3f(1,1,1);
		}

	// draw ball
	arad = 0.0;
	glBegin(GL_LINE_LOOP);
		for(float ang = 0; ang < 360; ang+=10) {
			
			b2Vec2 position = ball.body->GetPosition();
			position.y = WORLD_Y - position.y; //inverte o Y pra que ele cresça pra baixo

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
	world = new b2World(b2Vec2(0,0), false);
}

void initGlut(int argc, char** argv)
{
	glutInit (&argc, argv);
    glutInitDisplayMode ( GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA );
    glutInitWindowSize (WINDOW_X,WINDOW_Y);
    glutCreateWindow("RoboPET Simulator - controls: WASD, IJKL");

	glutIdleFunc (iterate);
	glutKeyboardFunc(keyboardFunc);

	glutMainLoop();
}

void receive()
{
	RoboPET_WrapperPacket packet;
	if (radiotosim.receive(packet) && packet.has_radiotosim()) {
		printf("----------------------------\n");
		printf("Received Radio-To-SIM\n");

		RadioToSim data = packet.radiotosim();

		//playersTotal[data.team_id()] = data.robots_size();

		for(int i = 0; i < data.robots_size(); i++) {
			robots[data.team_id()][i].forces = Vector(data.robots(i).force_x(), data.robots(i).force_y());
			robots[data.team_id()][i].forces.rotate(-90);
			robots[data.team_id()][i].displacement_angle = data.robots(i).displacement_theta();
			robots[data.team_id()][i].doKick = data.robots(i).kick();
			robots[data.team_id()][i].doDribble = data.robots(i).drible();
			robots[data.team_id()][i].id = data.robots(i).id();
			robots[data.team_id()][i].isUpdated = true;

			printf("RECEIVED Robot[%i]: forceVector<%lf,%lf> (%i degrees)\n",data.team_id(),data.robots(i).force_x(),data.robots(i).force_y(),data.robots(i).displacement_theta());
		}
	}
}

void send()
{
	bool verbose = true;
	RoboPET_WrapperPacket packet;

	if(verbose) printf("----------------------------\n");
	if(verbose) printf("Sendindg Sim-To-Tracker\n");

	SimToTracker *simtotrackerPacket = packet.mutable_simtotracker();
	SimToTracker::Ball *b = simtotrackerPacket->mutable_ball();

	for(int team = 0; team < TEAM_TOTAL; team++)
		for(int i = 0; i < playersTotal[team]; i++) {
				SimToTracker::Robot *r =
					(team == TEAM_BLUE ?
						simtotrackerPacket->add_blue_robots() :
						simtotrackerPacket->add_yellow_robots());

				r->set_x( (int)M_TO_MM(robots[team][i].body->GetPosition().x) );
				r->set_y( (int)M_TO_MM(robots[team][i].body->GetPosition().y) );
				r->set_theta( (int)(robots[team][i].body->GetAngle()*180./M_PI) );
				r->set_id( robots[team][i].id );

				if(verbose) printf("SENT Robot[%i]: <%lf,%lf> (%i degrees)\n",robots[team][i].id,robots[team][i].body->GetPosition().x,robots[team][i].body->GetPosition().y,(int)(robots[team][i].body->GetAngle()*180./M_PI));
		}

	 b->set_x( (int)M_TO_MM(ball.body->GetPosition().x) );
	 b->set_y( (int)M_TO_MM(ball.body->GetPosition().y) );

	 simtotracker.send(packet);

	 //printf("packet.robots_blue_size() = %5i\n", simtotrackerPacket->blue_robots_size());
	 //printf("packet.robots_yellow_size() = %5i\n", simtotrackerPacket->yellow_robots_size());

	//printf("Sent Sim-To-Tracker\n");
}

void openRadiotosim() 
{
	radiotosim.open(false);
}

void openSimtotracker() 
{
	simtotracker.open();
}

void parseOptions(int argc, char **argv)
{
	char ch;

	while((ch = getopt(argc, argv, "uh")) != EOF) {
		
		switch(ch) {
			case 'h':
				// Be sure that this print is updated with all options from this 'switch'.
				printf("Command line options:\n");
				printf(" -u\t\t Unreal Simulation Mode.\n");
				break;
			
			case 'u':
				ROBOT_DAMP = 30;
				printf("Unreal Simulation Mode ON.\n");
				break;
		}
	}
}
