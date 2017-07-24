/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include "GLDebugFont.h"
#include <stdio.h> //printf debugging

#include "GyroscopicDemo.h"
#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"


#include "GLDebugDrawer.h"

// This library
#include "core/terrain/tgBoxGround.h"
#include "core/terrain/tgEmptyGround.h"
#include "core/terrain/tgHillyGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "tgcreator/tgUtil.h"
// Bullet Physics
#include "LinearMath/btVector3.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h" //cd
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
 
static GLDebugDrawer	gDebugDrawer;





void	GyroscopicDemo::setupEmptyDynamicsWorld()
{
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	m_overlappingPairCache = new btDbvtBroadphase();
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
}

void	GyroscopicDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

//tgSimulation simulation(view);

void	GyroscopicDemo::initPhysics()
{
	m_azi=90;
	m_ele = 20;

	setTexturing(true);
	setShadows(true);
	setCameraUp(btVector3(0,0,1));
	setCameraForwardAxis(1);
	m_sundirection.setValue(0,-1,-1);
	setCameraDistance(7.f);

	setupEmptyDynamicsWorld();
	m_dynamicsWorld->setGravity(btVector3(0,0,-9.8));
	m_dynamicsWorld->setDebugDrawer(&gDebugDrawer);
	


	//btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.),btScalar(50.),btScalar(0.5)));
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0,0,1),0);
	
	m_collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0,0,0));
	btRigidBody* groundBody;
	groundBody= localCreateRigidBody(0, groundTransform, groundShape);
	groundBody->setFriction(btSqrt(2));
	btVector3 positions[2] = {
		btVector3(0.8,-2,2),
		btVector3(0.8,2,2)
	};
	bool gyro[2] = {
		true,
		false
	};

	for (int i=0;i<1;i++)
	{
		//btCylinderShapeZ* top  = new btCylinderShapeZ(btVector3(1,1,0.125));
		//btCapsuleShapeZ* pin  = new btCapsuleShapeZ(0.05,1.5);
		btBoxShape* box= new btBoxShape(btVector3(1 ,1 ,1 ) );
		//top->setMargin(0.01);
		//pin->setMargin(0.01);
		box->setMargin(0.01);
		btCompoundShape* compound = new btCompoundShape();
		//compound->addChildShape(btTransform::getIdentity(),top);
		//compound->addChildShape(btTransform::getIdentity(),pin);
		compound->addChildShape(btTransform::getIdentity(),box);
		btVector3 localInertia;
		//top->calculateLocalInertia(1,localInertia);
		box->calculateLocalInertia(1, localInertia);
		
		btRigidBody* body = new btRigidBody(1,0,compound,localInertia);
		
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(positions[i]);
		body->setCenterOfMassTransform(tr);
		
		//Set Angular velocity of the box
		body->setAngularVelocity(btVector3(0,0,100));
                //Set Linear velocity of the box
		body->setLinearVelocity(btVector3(0,.2,0));
		body->setFriction(btSqrt(1));
		
		m_dynamicsWorld->addRigidBody(body);
		
		if (gyro[i])
		{
			body->setFlags(BT_ENABLE_GYROPSCOPIC_FORCE);
		} else
		{
			body->setFlags(0);
		}
		body->setDamping(0.00001f,0.0001f);

		//simulation.addModel(body);
		//simulation.run();
	}

}

void	GyroscopicDemo::exitPhysics()
{

		int i;

	//removed/delete constraints
	for (i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
	{
		btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
		m_dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}

	//remove the rigidbodies from the dynamics world and delete them
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}




	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	m_collisionShapes.clear();

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_constraintSolver;

	//delete broadphase
	delete m_overlappingPairCache;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

}

GyroscopicDemo::GyroscopicDemo()
{
}
GyroscopicDemo::~GyroscopicDemo()
{
	//cleanup in the reverse order of creation/initialization

	exitPhysics();

}


void GyroscopicDemo::clientMoveAndDisplay()
{
/*	
 glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

 	float dt = float(getDeltaTimeMicroseconds()) * 0.000001f;
	//printf("dt = %f: ",dt);

	{
		static bool once = true;
		if ( m_dynamicsWorld->getDebugDrawer() && once)
		{
			m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
			once=false;
		}
	}

	
	{
	 	//during idle mode, just run 1 simulation step maximum

		//DELETED
	int numSimSteps = m_dynamicsWorld->stepSimulation(dt,100,1./1000.f);

		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	
		
	}
	renderme();


    glFlush();
    swapBuffers();
    */
}




void GyroscopicDemo::displayCallback(void) {

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();


	renderme();

    glFlush();
    swapBuffers();
}
//ADDING HEDGEHOG

/**
 * Anonomous namespace so we don't have to declare the config in
 * the header.
 */
namespace
{
    /**
     * Configuration parameters so they're easily accessable.
     * All parameters must be positive.
     */
    const struct Config
    {
        double density;
        double radius;
        double stiffness;
        double damping;
        double pretension;
        double triangle_length;
        double triangle_height;
        double prism_height;
        bool hist;
        double maxTension;
        double targetVelocity;
    } c =
   {
       0.688,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       50.0,     // damping (mass / sec)
       400.0,     // pretension (mass * length / sec^2)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // prism_height (length)
       0,           // history logging (boolean)
       10000,       // max tension
       1,         // target actuator velocity
  };
} // namespace


void GyroscopicDemo::addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{
    //Large box
    // bottom 1
    s.addNode(-5, 0, 0); // 0
    // bottom 2
    s.addNode( 5, 0, 0); // 1
    // bottom 3
    s.addNode(0, 0, 5); // 2
    // bottom 4
    s.addNode(0, 0, -5); // 3
    // top 1
    s.addNode(-5, 5, 0); // 4
    // top 2
    s.addNode(5, 5, 0); // 5
    // top 3
    s.addNode(0, 5, 5); // 6
    // top 4
    s.addNode(0, 5, -5); // 7
    //Small Box
    // bottom 5
    s.addNode(-2.5, 1.25, 0); // 8
    // bottom 6
    s.addNode( 2.5, 1.25, 0); // 9
    // bottom 7
    s.addNode(0, 1.25, 2.5); // 10
    // bottom 8
    s.addNode(0, 1.25, -2.5); // 11
    // top 9
    s.addNode(-2.5, 3.75, 0); // 12
    // top 10
    s.addNode(2.5, 3.75, 0); // 13
    // top 11
    s.addNode(0, 3.75, 2.5); // 14
    // top 12
    s.addNode(0, 3.75, -2.5); // 15
}

//btRigidBody * threeBarModel = //...
//btTransform tr;
//tr.setIdentity();
//btQuaternion quat;
//quat.setEuler(5,0,0); //or quat.setEulerZYX depending on the ordering you want
//tr.setRotation(quat);
//threeBarModel->setCenterOfMassTransform(tr);

void GyroscopicDemo::addRods(tgStructure& s)
{
/*
    s.addPair( 0,  4, tgString("rod num", 0));
    s.addPair( 1,  5, tgString("rod num", 1));
    s.addPair( 2,  6, tgString("rod num", 2));
    s.addPair( 3,  7, tgString("rod num", 3));
    s.addPair( 0,  2, tgString("rod num", 4));
    s.addPair( 0,  3, tgString("rod num", 5));
    s.addPair( 1,  2, tgString("rod num", 6));
    s.addPair( 1,  3, tgString("rod num", 7));
    s.addPair( 4,  6, tgString("rod num", 8));
    s.addPair( 4,  7, tgString("rod num", 9));
    s.addPair( 5,  6, tgString("rod num", 10));
    s.addPair( 5,  7, tgString("rod num", 11));
*/

    // s.addPair( 0,  4, "rod");
    // s.addPair( 1,  5, "rod");
    // s.addPair( 2,  3, "rod");
}
// GYRO DEMO

/*
void	threeBarModel::initPhysics()
{
	for (int i=0;i<1;i++)
	{
		
		//ISSUES HERE
		//btCylinderShapeZ* top  = new btCylinderShapeZ(btVector3(1,1,0.125));
		//btCapsuleShapeZ* pin  = new btCapsuleShapeZ(0.05,1.5);
		tgBox* box= new tgBox(btVector3(1 ,1 ,1 ) );
		//top->setMargin(0.01);
		//pin->setMargin(0.01);
		box->setMargin(0.01);
		//btCompoundShape* compound = new btCompoundShape();
		//compound->addChildShape(btTransform::getIdentity(),top);
		//compound->addChildShape(btTransform::getIdentity(),pin);
		//compound->addChildShape(btTransform::getIdentity(),box);
		btVector3 localInertia;
		//top->calculateLocalInertia(1,localInertia);
		//box->calculateLocalInertia(1, localInertia);
		
		btRigidBody* body = new btRigidBody(1,0,box,localInertia);
		
		btTransform tr;
		tr.setIdentity();
		//tr.setOrigin(positions[i]);
		body->setCenterOfMassTransform(tr);
		
		//Set Angular velocity of the box
		body->setAngularVelocity(btVector3(0,0,100));
                //Set Linear velocity of the box
		body->setLinearVelocity(btVector3(0,.2,0));
		body->setFriction(btSqrt(1));
		
		//m_dynamicsWorld->addRigidBody(body);
		//tgWorld->addRigidBody(body);
		
		body->setDamping(0.00001f,0.0001f);

		
	}

}

//}
*/
//GYRO DEMO


//void threeBarModel::addActuators(tgStructure& s)
//{
//    // Bottom Triangle
//    s.addPair(0, 1, tgString("actuator num", 0));
//    s.addPair(1, 2, tgString("actuator num", 1));
//    s.addPair(2, 0, tgString("actuator num", 2));
//    
//    // Top
//    s.addPair(3, 4, tgString("actuator num", 3));
//    s.addPair(4, 5, tgString("actuator num", 4));
//    // FILL IN LINE 5.2 HERE
//    s.addPair(5, 3, tgString("actuator num", 5));

    //Edges
//    s.addPair(0, 3, tgString("actuator num", 6));
//    s.addPair(1, 4, tgString("actuator num", 7));
//    s.addPair(2, 5, tgString("actuator num", 8));
//}

void GyroscopicDemo::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density);
    //const tgBasicActuator::Config actuatorConfig(c.stiffness, c.damping, c.pretension,
    //    c.hist, c.maxTension, c.targetVelocity);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s, c.triangle_length, c.triangle_height, c.prism_height);
    
    // Add rods to the structure
    addRods(s);
    
    // Add actuators to the structure
    //addActuators(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
  //s.addRotation(btVector3(0,10,0),btVector3(4,12,3),btVector3(1,45,18));
    btTransform T(btQuaternion(btVector3(0,1,0),btRadians(60)),btVector3(0.0,0.5,0));
    //s.addRotation(btVector3(0,10,0),btQuaternion(1,2,4,3));
//    s.addPair(2, 5, tgString("actuator num", 8));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    //spec.addBuilder("actuator", new tgBasicActuatorInfo(actuatorConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // Get the rod rigid bodies for controller
    std::vector<tgRod*> rods = GyroscopicDemo::find<tgRod>("rod");
    for (int i = 0; i < rods.size(); i++) {
   //     allRods.push_back(GyroscopicDemo::find<tgRod>(tgString("rod num", i))[0]);    
    }
        
    // Get the actuators for controller
    //std::vector<tgBasicActuator*> actuators = threeBarModel::find<tgBasicActuator>("actuator");
    //for (int i = 0; i < rods.size(); i++) {
    //    allActuators.push_back(threeBarModel::find<tgBasicActuator>(tgString("actuator num", i))[0]);    
    //}

    // Notify controllers that setup has finished.
    //notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void GyroscopicDemo::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        //notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void GyroscopicDemo::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

//std::vector<tgBasicActuator*>& threeBarModel::getAllActuators()
//{
//    return allActuators;
//}

std::vector<tgRod*>& GyroscopicDemo::getAllRods()
{
    return allRods;
}
    
void GyroscopicDemo::teardown()
{
    //notifyTeardown();
    tgModel::teardown();
}


