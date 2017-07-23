#include "GyroscopicDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

#include "btBulletDynamicsCommon.h"


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
#include "btBulletDynamicsCommon.h" //cd
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
 
 
 

int main(int argc,char** argv)
{
	//TENSE
    std::cout << "Gyro" << std::endl;
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
	btScalar x = 1.0;
	btScalar y = 1.0;
	btScalar z = 1.0;
	// instantiate our model object
	btVector3 boxModelShape(x, y, z);
	
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    
    double sf = 10;
    double gravity = 9.81*sf;
    const tgWorld::Config config(gravity); // gravity, cm/sec^2
    tgWorld world(config, ground);
	
	
	    // Second create the view
	const double timestep_physics =0.001; // seconds
	const double timestep_graphics = 1.f/60.f; // seconds
	tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulationu t
	//tgSimulation simulation(view);
	tgSimulation* simulation = new tgSimulation(view);
	
    // Fourth create the models with their controllers and add the models to the
    // simulation
    
    btBoxShape* boxModel = new btBoxShape(boxModelShape);
    
    // Create the controller
    // FILL IN 5.4 HERE
	//LengthController* const myController = new LengthController();

    // Attach controller to the model
    // FILL IN 5.6 HERE
	//myModel->attach(myController);

    // Add the model to the world

// TENSE
	GyroscopicDemo* const constraintDemo = new GyroscopicDemo();
	tgModel*  newTgDemo = new tgModel();
	
    	simulation->run();
    constraintDemo->initPhysics();
	constraintDemo->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
	constraintDemo->Create();
	simulation->addModel(  (tgModel*)constraintDemo );
	simulation->addModel(  (tgModel*)boxModel );
	//simulation.addModel(newTgDemo);
	//I am having issues with this fucntionc

	
	return glutmain(argc, argv,640,480,"Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/",constraintDemo);
}

