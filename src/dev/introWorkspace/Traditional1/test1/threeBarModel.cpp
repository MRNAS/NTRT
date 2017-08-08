/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
*/

/**
 * @file 3BarModel.cpp
 * @brief Contains the definition of the members of the class threeBarModel.
 * $Id$
 */

// This module
#include "threeBarModel.h"
#include "core/tgString.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
// The C++ Standard Library
#include <stdexcept>

// Dependencies coming from Gyro
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include "GLDebugFont.h"
#include <stdio.h> //printf debugging

#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"


#include "GLDebugDrawer.h"
//Dependencies coming from Gyro


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

threeBarModel::threeBarModel() : tgModel() 
{
}

threeBarModel::~threeBarModel()
{
}

void threeBarModel::addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{
    //Large box

    s.addNode(-5, 0, 0); // 0
    s.addNode( 5, 0, 0); // 1
    s.addNode(-5, 0, 0); // 2
    s.addNode(0, 0, 0); // 3
    s.addNode(-5, 0, 5); // 4
    s.addNode(5, 0, 5); // 5
	
}

//btRigidBody * threeBarModel = //...
//btTransform tr;
//tr.setIdentity();
//btQuaternion quat;
//quat.setEuler(5,0,0); //or quat.setEulerZYX depending on the ordering you want
//tr.setRotation(quat);
//threeBarModel->setCenterOfMassTransform(tr);

void threeBarModel::addRods(tgStructure& s)
{
    s.addPair( 0,  1, tgString("rod num", 0));
    s.addPair( 2,  3, tgString("rod num", 1));
    s.addPair( 4,  5, tgString("rod num", 2));
    s.addPair( 6,  7, tgString("rod num", 3));

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

void threeBarModel::setup(tgWorld& world)
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
    s.move(btVector3(0, 20, 0));
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
    std::vector<tgRod*> rods = threeBarModel::find<tgRod>("rod");
    for (int i = 0; i < rods.size(); i++) {
        allRods.push_back(threeBarModel::find<tgRod>(tgString("rod num", i))[0]);    
    }
        
    // Get the actuators for controller
    //std::vector<tgBasicActuator*> actuators = threeBarModel::find<tgBasicActuator>("actuator");
    //for (int i = 0; i < rods.size(); i++) {
    //    allActuators.push_back(threeBarModel::find<tgBasicActuator>(tgString("actuator num", i))[0]);    
    //}

    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void threeBarModel::step(double dt)
{
    // Precondition
    if (dt <= 0.0)
    {
        throw std::invalid_argument("dt is not positive");
    }
    else
    {
        // Notify observers (controllers) of the step so that they can take action
        notifyStep(dt);
        tgModel::step(dt);  // Step any children
    }
}

void threeBarModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

//std::vector<tgBasicActuator*>& threeBarModel::getAllActuators()
//{
//    return allActuators;
//}

std::vector<tgRod*>& threeBarModel::getAllRods()
{
    return allRods;
}
    
void threeBarModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
