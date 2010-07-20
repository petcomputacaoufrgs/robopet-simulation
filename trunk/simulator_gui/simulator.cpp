/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "/usr/include/Box2D.h"
#include <GL/glut.h>
#include <cstdio>
#include <iostream>
using namespace std;

#define WORLD_X 1000
#define WORLD_Y 1000

#define NROBOTS 4
#define ROBOT_R 50
#define ROBOT_DENSITY 0.1

#define BALL_R 10


b2Body* objects[NROBOTS];
b2Body* ball;
b2World* world;


// keyboard keys
void keyboardFunc(int key, int x, int y)
{
		b2Vec2 fv;
		objects[0]->WakeUp();
		
        if( key == GLUT_KEY_LEFT ) {
			fv = b2Vec2(-10000,0);
        }
        
        if( key == GLUT_KEY_RIGHT ) {
	        fv = b2Vec2(10000,0);
        }
        
        if( key == GLUT_KEY_DOWN ) {
            fv = b2Vec2(0,-10000);
        }
        
        if( key == GLUT_KEY_UP ) {
            fv = b2Vec2(0,10000);
        }
        
		// Newtons/100
        //objects[0]->ApplyForce(fv,objects[0]->GetWorldCenter());
		objects[0]->ApplyImpulse(fv,objects[0]->GetWorldCenter());
}

// create a box physics object which will be used for the walls
b2Body* newWall(float x, float y, float sizex, float sizey)
{
	b2BodyDef bodyDef;
	bodyDef.position.Set(x, y);
	
	b2Body* body = world->CreateBody(&bodyDef);
	
	b2PolygonDef bodyShapeDef;
	bodyShapeDef.SetAsBox(sizex, sizey);
	bodyShapeDef.restitution = 0.8;
	body->CreateShape(&bodyShapeDef);
	
	return body;
}
	
// create a physics circular object which will be used for the ball and robots
b2Body* newDynamicCircle(float x, float y, float radius, float density, float friction, float restitution, float damping, b2Color color)
{
    b2CircleDef circsd;
	circsd.radius = radius;
	circsd.localPosition.Set(0,0);
	circsd.density = density;
	circsd.friction = friction;
	circsd.restitution = restitution;

	b2Vec2 pos(x,y);
	b2BodyDef bd;
	bd.position = pos;
    bd.linearDamping = damping;

	b2Body* body = world->CreateBody(&bd);
	body->CreateShape(&circsd);
	
	body->SetMassFromShapes();
	cout<< "mass: " << body->GetMass() <<endl;
		
    return body;
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


	// Prepare for simulation. Typically we use a time step of 1/60 of a
	// second (60Hz) and 10 iterations. This provides a high quality simulation
	// in most game scenarios.
	float32 timeStep = 1.0f / 60.0f;
	int32 iterations = 10;
		
	glClear(GL_COLOR_BUFFER_BIT);
	// Instruct the world to perform a single step of simulation. It is
	// generally best to keep the time step and iterations fixed.
	world->Step(timeStep, iterations);

	// Now print the position and angle of the body.
    for(int i=0; i<NROBOTS; i++)
    {
		b2Vec2 position = objects[i]->GetPosition();
		float angle = objects[i]->GetAngle();

		float arad;
		glBegin(GL_LINE_LOOP);
			for(float ang = 0; ang < 360; ang+=10)
			{
				arad = ang * M_PI / 180.0;
				glVertex2f( position.x + cos(arad)*ROBOT_R,
							position.y + sin(arad)*ROBOT_R);
			}
		glEnd();
	}
	b2Vec2 position = ball->GetPosition();
	float arad;
	glBegin(GL_LINE_LOOP);
		for(float ang = 0; ang < 360; ang+=10)
		{
			arad = ang * M_PI / 180.0;
			glVertex2f( position.x + cos(arad)*BALL_R,
						position.y + sin(arad)*BALL_R);
		}
	glEnd();	
	
	glutSwapBuffers();
}
	
	
void initObjects()
{
	static b2Body* wallTop = newWall(0,WORLD_Y, WORLD_X,1);
	static b2Body* wallBot = newWall(0,-WORLD_Y, WORLD_X,1);
	static b2Body* wallRight = newWall(WORLD_X,0, 1,WORLD_Y);
	static b2Body* wallLeft = newWall(-WORLD_X,0, 1,WORLD_Y);

	//                              x, y, radius,  density,       friction,restitution,damping color)
    objects[0] = newDynamicCircle( 30,30, ROBOT_R, ROBOT_DENSITY, 1, 0.5, 0.05,  b2Color(1,0,0));
    objects[1] = newDynamicCircle(-30,-30,ROBOT_R, ROBOT_DENSITY, 1, 0.5, 0.05,  b2Color(1,0,0));
    objects[2] = newDynamicCircle(-10,-30,ROBOT_R, ROBOT_DENSITY, 1, 0.5, 0.05,  b2Color(1,0,0));
	objects[3] = newDynamicCircle(-20,-400,ROBOT_R, ROBOT_DENSITY, 1, 0.5, 0.05,  b2Color(1,0,0));
    ball = newDynamicCircle(        0,  0,BALL_R,  0.5,           1, 0.9, 0.03,  b2Color(1,0,0));	
}

void initWorld()
{
	// Define the size of the world. Simulation will still work
	// if bodies reach the end of the world, but it will be slower.
	static b2AABB worldAABB;
	worldAABB.lowerBound.Set(-WORLD_X, -WORLD_Y);
	worldAABB.upperBound.Set(WORLD_X, WORLD_Y);

	// Do we want to let bodies sleep?
	bool doSleep = true;

	// worldAABB, b2Vec2 &gravity, bool doSleep
	world = new b2World(worldAABB, b2Vec2(0,0), true);
}

void initGlut(int argc, char** argv)
{
	glutInit (&argc, argv);
    glutInitDisplayMode ( GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA );
    glutInitWindowSize (500,500);
    glutInitWindowPosition (500, 500);
    glutCreateWindow("");
    
	glutIdleFunc (drawScene);
	glutSpecialFunc(keyboardFunc);
	
	glutMainLoop();
}	

// This is a simple example of building and running a simulation
// using Box2D. Here we create a large ground box and a small dynamic
// box.
int main(int argc, char** argv)
{
	initWorld();
	initObjects();
	initGlut(argc, argv);
}
