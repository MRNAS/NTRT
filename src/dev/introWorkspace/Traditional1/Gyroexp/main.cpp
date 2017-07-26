#include "GyroscopicDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"

#include "btBulletDynamicsCommon.h"

//tg dependencies
///////////////////////////////////////////////////
// include "LengthController.h"
// This application

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
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
/////////////////////////////////////////////////////////////
//tg dependencies

int main(int argc,char** argv)
{
//CREATING TG WORLD
/////////////////////////////////
std::cout << "App3Bar" << std::endl;

    // First create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    
    double sf = 10;
    double gravity = 9.81*sf;
    const tgWorld::Config config(gravity); // gravity, cm/sec^2
    tgWorld world(config, ground);

//NORMAL TERRAIN


    // Second create the view
	const double timestep_physics =0.001; // seconds
	const double timestep_graphics = 1.f/60.f; // seconds
	tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
	tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
	//threeBarModel* const myModel = new threeBarModel();
    
    // Create the controller
    // FILL IN 5.4 HERE
	//LengthController* const myController = new LengthController();

    // Attach controller to the model
    // FILL IN 5.6 HERE
	//myModel->attach(myController);


//////////////////////////////////////////////	
//CREATING TG WORLD

    GyroscopicDemo* constraintDemo = new GyroscopicDemo();
	
    // Add the model to the world (THIS IS TG)
	simulation.addModel(constraintDemo);
    	simulation.run();
//

	constraintDemo->initPhysics();
	constraintDemo->setDebugMode(btIDebugDraw::DBG_DrawConstraints+btIDebugDraw::DBG_DrawConstraintLimits);
    //Teardown is handled by delete, so that should be automatic
	return 0; //TG added
    
	//below eliminted for TG work to function
	//return glutmain(argc, argv,640,480,"Constraint Demo. http://www.continuousphysics.com/Bullet/phpBB2/",constraintDemo);
}



