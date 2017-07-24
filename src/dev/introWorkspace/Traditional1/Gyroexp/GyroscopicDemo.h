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
#ifndef BT_GYROSCOPIC_DEMO_H
#define BT_GYROSCOPIC_DEMO_H
/*
#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif
*/
//tg addins

// This library
#include "core/tgModel.h"
#include "core/tgSubject.h"

// Builder libraries
//#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgNode.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "core/tgSimViewGraphics.h" 

// The C++ Standard Library
#include <vector>

// Forward declarations
class tgBasicActuator;
class tgModelVisitor;
class tgStructure;
class tgWorld;
//tg addins

///GyroscopicDemo shows how to create a constraint, like Hinge or btGenericD6constraint
class GyroscopicDemo : public PlatformDemoApplication, public tgModel
{
	//keep track of variables to delete memory at the end
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	void	setupEmptyDynamicsWorld();

	void	clientResetScene();


	public:

	GyroscopicDemo();

	virtual ~GyroscopicDemo();

	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
/*
	static DemoApplication* Create()
	{
		GyroscopicDemo* demo = new GyroscopicDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}
*/
//tg addins
    
    /**
     * The only constructor. Configuration parameters are within the
     * .cpp file in this case, not passed in. 
     */
    
    /**
     * Destructor. Deletes controllers, if any were added during setup.
     * Teardown handles everything else.
     */
    
    /**
     * Create the model. Place the rods and strings into the world
     * that is passed into the simulation. This is triggered
     * automatically when the model is added to the simulation, when
     * tgModel::setup(world) is called (if this model is a child),
     * and when reset is called. Also notifies controllers of setup.
     * @param[in] world - the world we're building into
     */
    virtual void setup(tgWorld& world);
    
    /**
     * Undoes setup. Deletes child models. Called automatically on
     * reset and end of simulation. Notifies controllers of teardown
     */
    virtual void teardown();
    
    /**
     * Step the model, its children. Notifies controllers of step.
     * @param[in] dt, the timestep. Must be positive.
     */
    virtual void step(double dt);
    
    /**
     * Receives a tgModelVisitor and dispatches itself into the
     * visitor's "render" function. This model will go to the default
     * tgModel function, which does nothing.
     * @param[in] r - a tgModelVisitor which will pass this model back
     * to itself 
     */
    virtual void onVisit(tgModelVisitor& r);
    
    /**
     * Return a vector of all actuators for the controllers to work with.
     * @return A vector of all of the actuators
     */
    //std::vector<tgBasicActuator*>& getAllActuators();

    /**
     * Return a vector of all rod bodies for the controllers to work with.
     * @return A vector of all of the rod rigid bodies
     */
    std::vector<tgRod*>& getAllRods();
      
private:
    
    /**
     * A function called during setup that determines the positions of
     * the nodes based on construction parameters. Rewrite this function
     * for your own models
     * @param[in] s: A tgStructure that we're building into
     * @param[in] edge: the X distance of the base points
     * @param[in] width: the Z distance of the base triangle
     * @param[in] height: the Y distance along the axis of the prism
     */
    static void addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height);
    
    /**
     * A function called during setup that creates rods from the
     * relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addRods(tgStructure& s);
    
    /**
     * A function called during setup that creates actuators (Strings) from
     * the relevant nodes. Rewrite this function for your own models.
     * @param[in] s A tgStructure that we're building into
     */
    static void addActuators(tgStructure& s);

private:    
    /**
     * A list of all of the basic actuators. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    //std::vector<tgBasicActuator*> allActuators;

    /**
     * A list of all of the rods. Will be empty until most of the way
     * through setup when it is filled using tgModel's find methods
     */
    std::vector<tgRod*> allRods;
};
//add

#endif //BT_GYROSCOPIC_DEMO_H

