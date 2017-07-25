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
 * @file Tensegrity_HedgehogModel.cpp
 * @brief Contains the definition of the members of the class HedgehogTensegrity
  * @author Manuel Retana
 * $Id$
 */

// This module
#include "TensegrityHedgehogModel.h"
#include "core/tgString.h"
// The Bullet Physics library
//Box add
#include "core/tgBox.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "tgcreator/tgNode.h"
//Box add
#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
// The C++ Standard Library
#include <stdexcept>

// Dependencies coming from Gyro
/*#include "btBulletDynamicsCommon.h"
#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"

#include "GLDebugFont.h"
#include <stdio.h> //printf debugging

#include "GL_ShapeDrawer.h"
#include "GlutStuff.h"


#include "GLDebugDrawer.h"
*/
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
	// Box parameters
	double width;
	double height;
	double friction;
	double rollFriction;
	double restitution;
	//Box parameters
    } c =
   {
       0.688,     // density (mass / length^3)
       0.15,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       50.0,     // damping (mass / sec)
       400.0,     // pretension (mass * length / sec^2)
       10.0,     // triangle_length (length)
       10.0,     // triangle_height (length)
       20.0,     // prism_height (length)
       0,           // history logging (boolean)
       10000,       // max tension
       1,         // target actuator velocity
	//Box
	1.0, // width (dm?)
        1.0, // height (dm?)
        //0.0,  density (kg / length^3)
        1.0,  // friction (unitless)
        0.01, // rollFriction (unitless)
        0.2,  // restitution (?)
	//Box
  };
} // namespace

TensegrityHedgehogModel::TensegrityHedgehogModel() : tgModel() 
{
	origin = btVector3(0,0,0); //Box
}
//box
TensegrityHedgehogModel::TensegrityHedgehogModel(btVector3 center) : tgModel() 
{
	origin = btVector3(center.getX(), center.getY(), center.getZ()); 
}
//box
TensegrityHedgehogModel::~TensegrityHedgehogModel()
{
}
/*void TensegrityHedgehogModel::addNodes(tgStructure& s) {
#if (0)
    const int nBoxes = 4; 
#endif // Suppress compiler warning unused variable
    // Accumulating rotation on boxes
    btVector3 rotationPoint = origin;
    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    double rotationAngle = M_PI/2;

    addBoxNodes();
    addBoxNodes();
    addBoxNodes();
    addBoxNodes();
    
    for(std::size_t i=0;i<nodes.size();i+=2) {
        s.addNode(nodes[i]);
        s.addNode(nodes[i+1]);
        s.addRotation(rotationPoint, rotationAxis, rotationAngle);
        s.addPair(i, i+1, "box");
    }
    s.move(btVector3(0, 50, 0)); // Sink boxes into the ground
*/
void TensegrityHedgehogModel::addNodes(tgStructure& s,
                            double edge,
                            double width,
                            double height)
{
    //Large box
    // bottom 1
    s.addNode(-10, 0, 0); // 0
    // bottom 2
    s.addNode( 10, 0, 0); // 1
    // bottom 3
    s.addNode(0, 0, 10); // 2
    // bottom 4
    s.addNode(0, 0, -10); // 3
    // top 1
    s.addNode(-10, 10, 0); // 4
    // top 2
    s.addNode(10, 10, 0); // 5
    // top 3
    s.addNode(0, 10, 10); // 6
    // top 4
    s.addNode(0, 10, -10); // 7
    //Small Box
    // bottom 5
    s.addNode(-5, 2.5, 0); // 8
    // bottom 6
    s.addNode( 5, 2.5, 0); // 9
    // bottom 7
    s.addNode(0, 2.5, 5); // 10
    // bottom 8
    s.addNode(0, 2.5, -5); // 11
    // top 5
    s.addNode(-5, 7.5, 0); // 12
    // top 7
    s.addNode(5, 7.5, 0); // 13
    // top 8
    s.addNode(0, 7.5, 5); // 14
    // top 9
    s.addNode(0, 7.5, -5); // 15

}


//btRigidBody * threeBarModel = //...
//btTransform tr;
//tr.setIdentity();
//btQuaternion quat;
//quat.setEuler(5,0,0); //or quat.setEulerZYX depending on the ordering you want
//tr.setRotation(quat);
//threeBarModel->setCenterOfMassTransform(tr);

void TensegrityHedgehogModel::addRods(tgStructure& s)
{
    //Tensegrity
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

//Hedgehog Cover
    s.addPair( 8,  12, tgString("rod num", 12));
    s.addPair( 9,  13, tgString("rod num", 13));
    s.addPair( 10,  14, tgString("rod num", 14));
    s.addPair( 11,  15, tgString("rod num", 15));
    s.addPair( 8,  10, tgString("rod num", 16));
    s.addPair( 8,  11, tgString("rod num", 17));
    s.addPair( 9,  10, tgString("rod num", 18));
    s.addPair( 9,  11, tgString("rod num", 19));
    s.addPair( 12,  14, tgString("rod num", 20));
    s.addPair( 12,  15, tgString("rod num", 21));
    s.addPair( 13,  14, tgString("rod num", 22));
    s.addPair( 13,  15, tgString("rod num", 23));
    //xpairs
  //top
    s.addPair( 12,  13, tgString("rod num", 24));
    s.addPair( 14,  15, tgString("rod num", 25));
    //bottom
    s.addPair( 8,  9, tgString("rod num", 26));
    s.addPair(10,  11, tgString("rod num", 27));

    s.addPair( 9,  14, tgString("rod num", 28));
    s.addPair( 10,  13, tgString("rod num", 29));
    
    s.addPair( 9,  15, tgString("rod num", 30));
    s.addPair( 10,  12, tgString("rod num", 31));
          
    s.addPair( 8,  14, tgString("rod num", 32));
    s.addPair( 11,  13, tgString("rod num", 33));
//Still troubleshoot
    s.addPair( 8,  15, tgString("rod num", 34));
    s.addPair( 11,  12, tgString("rod num", 35));
   


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

void TensegrityHedgehogModel::addActuators(tgStructure& s)
{
    // Bottom
    s.addPair(0, 8, tgString("actuator num", 0));
    s.addPair(1, 9, tgString("actuator num", 1));
    s.addPair(2, 10, tgString("actuator num", 2));
    s.addPair(3, 11, tgString("actuator num", 3));
 
    // Top
    s.addPair(4, 12, tgString("actuator num", 4));
    s.addPair(5, 13, tgString("actuator num", 5));
    s.addPair(6, 14, tgString("actuator num", 6));
    s.addPair(7, 15, tgString("actuator num", 7));

}

void TensegrityHedgehogModel::setup(tgWorld& world)
{
    const tgBox::Config boxConfig(c.width, c.height, c.density, c.friction, c.rollFriction, c.restitution); //box
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgBasicActuator::Config actuatorConfig(c.stiffness, c.damping, c.pretension,
       c.hist, c.maxTension, c.targetVelocity);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    tgStructure y;
    addNodes(y); 
    // Add nodes to the structure
    addNodes(s, c.triangle_length, c.triangle_height, c.prism_height);
    
    // Add rods to the structure
    addRods(s);
    
    // Add actuators to the structure
    //addActuators(s);
    
    // Move the structure so it dohgesn't start in the ground
    y.move(btVector3(5,1.5, 5));
    s.move(btVector3(5,0, 5));
  //s.addRotation(btVector3(0,10,0),btVector3(4,12,3),btVector3(1,45,18));
    //btTransform T(btQuaternion(btVector3(0,1,0),btRadians(60)),btVector3(0.0,0.5,0));
    //s.addRotation(btVector3(0,10,0),btQuaternion(1,2,4,3));
//    s.addPair(2, 5, tgString("actuator num", 8));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("box", new tgBoxInfo(boxConfig));//tg Box

    spec.addBuilder("actuator", new tgBasicActuatorInfo(actuatorConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec); //Tensegrity
    tgStructureInfo structureInfos(y, spec); //Hedgehog

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world); //Tensegrity
    structureInfos.buildInto(*this, world); //Hedgehog

    // Get the rod rigid bodies for controller
    std::vector<tgRod*> rods = TensegrityHedgehogModel::find<tgRod>("rod");
    for (int i = 0; i < rods.size(); i++) {
        allRods.push_back(TensegrityHedgehogModel::find<tgRod>(tgString("rod num", i))[0]);    
    }
        
    // Get the actuators for controller
    std::vector<tgBasicActuator*> actuators = TensegrityHedgehogModel::find<tgBasicActuator>("actuator");
    for (int i = 0; i < rods.size(); i++) {
        allActuators.push_back(TensegrityHedgehogModel::find<tgBasicActuator>(tgString("actuator num", i))[0]);    
    }

    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void TensegrityHedgehogModel::step(double dt)
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

void TensegrityHedgehogModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

std::vector<tgBasicActuator*>& TensegrityHedgehogModel::getAllActuators()
{
   return allActuators;
}

std::vector<tgRod*>& TensegrityHedgehogModel::getAllRods()
{
    return allRods;
}
    
void TensegrityHedgehogModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}


//BOX Extra
// Nodes: center points of opposing faces of rectangles

void TensegrityHedgehogModel::addNodes(tgStructure& y) {
#if (0)
    const int nBoxes = 1; 
#endif // Suppress compiler warning unused variable
    // Accumulating rotation on boxes
    //btVector3 rotationPoint = origin; Not necessary Only need one box
    //btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    //double rotationAngle = M_PI/2;

    addBoxNodes();
    //addBoxNodes();
    //addBoxNodes();
    //addBoxNodes();
    
        for(std::size_t i=0;i<nodes.size();i+=2) {
        y.addNode(nodes[i]);
        y.addNode(nodes[i+1]);
        //y.addRotation(rotationPoint, rotationAxis, rotationAngle);
        y.addPair(i, i+1, "box");
    }
	
    //y.move(btVector3(0, 50, 0)); // Sink boxes into the ground
}

//Box Extra
void TensegrityHedgehogModel::addBoxNodes() {
    tgNode node;
  
    double x1 = 2; // Smaller x values leads to a narrower crater
    double x2 = 1;
    double y1 = 2;
    double y2 = 1;
    double z1 = 1;
    double z2 = 1.5;//sqrt(3);
 /* 
    btVector3 rotationPoint = btVector3((x2-x1)/2, (y2-y1)/2, (z2-z1)/2); //Halfway between nodes
    btVector3 rotationAxis = btVector3(0, 1, 0);  // y-axis
    double rotationAngle = 0; // Must != 0 for actual change
*/
    node = tgNode(x1, y1, z1, "node");
    //node.addRotation(rotationPoint, rotationAxis, rotationAngle);
    nodes.push_back(node);

    node = tgNode(x2, y2, z2, "node");
    //node.addRotation(rotationPoint, rotationAxis, rotationAngle);
    nodes.push_back(node);
     
}


//Box extra