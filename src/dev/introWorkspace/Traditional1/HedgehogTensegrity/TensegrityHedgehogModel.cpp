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
// This library
#include "core/tgSpringCableActuator.h"
#include "core/tgRod.h"
#include "core/tgBox.h"
#include "tgcreator/tgBoxInfo.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btQuaternion.h"
#include "LinearMath/btTransform.h"
// The C++ Standard Library
#include <stdexcept>

namespace
{
    // see tgBaseString.h for a descripton of some of these rod parameters
    // (specifically, those related to the motor moving the strings.)
    // NOTE that any parameter that depends on units of length will scale
    // with the current gravity scaling. E.g., with gravity as 98.1,
    // the length units below are in decimeters.

    // Provide specifics about the hedgehog mass etc
    

    // similarly, frictional parameters are for the tgRod objects.
    const struct Config
    {
        double density; //Density of the rods
        double radius;
        double stiffness;
        double stiffness_in;
        double damping;
        double damping_in;
        double rod_length;
        double rod_space; 
        double payload_h; 
        double density_pay;
        double radius_pay;  
        double friction;
        double rollfriction;
        double restitution;
        double pretension;
        bool   history; 
        double maxTens;
        double targetVelocity;
	//Hedgehog Parameters beyond this point
	double width;
	double height;
	double density1; //Density of the box
	double friction1;
	double rollfriction1;
	double restitution1;
	double triangle_length;
        double triangle_height;
        double prism_height;
    } c =
   {
     0.5366,    // density (kg / length^3)
     0.3175/2.0,     // radius (length)
     250000.0,   // stiffness of outer muscles (kg / sec^2)
     500.0,    // stiffness of inner muscles (kg/sec^2)
     25000.0,    // damping of outer muscles (kg / sec)
     50.0,     //damping of inner muscles (kg/sec)
     15.0,     // rod_length (length)
     3.75,      // rod_space (length)
     0.5,         // half payload height (length)
     20,        //payload density (kg/length^3)
     0.5,        //payload radius (length)
     0.99,      // friction (unitless)
     0.1,     // rollFriction (unitless)
     0.0,      // restitution (?)
     10.0,        // pretension (force)
     false,     // history
     1000000,   // maxTens
     10000,    // targetVelocity
     1.75, // width (dm)
     1.75, // length (dm)
     4.0,  //Density of box (kg / length^3) 
     0.5,  // friction (unitless)
     0.01, // rollFriction (unitless) Double check what roll friction of a box means
     0,  // restitution (?)
     10.0,     // triangle_length (length) Value not relevant
     10.0,     // triangle_height (length) Value not relevant
     20.0,     // prism_height (length) Value not relevant
  };
} // namespace

TensegrityHedgehogModel::TensegrityHedgehogModel() : tgModel() 
{
	origin = btVector3(0,0,0); //Double check if needed
}

TensegrityHedgehogModel::TensegrityHedgehogModel(btVector3 center) : tgModel() 
{
	origin = btVector3(center.getX(), center.getY(), center.getZ()); //Double check if needed
}

TensegrityHedgehogModel::~TensegrityHedgehogModel()
{
}

void TensegrityHedgehogModel::addNodes(tgStructure& s,
			    double edge,
                            double width,
                            double height)
{
    const double half_length = c.rod_length / 2;

    // Nodes for struts
    s.addNode(-c.rod_space,  -half_length, 0);            // 0
    s.addNode(-c.rod_space,   half_length, 0);            // 1
    s.addNode( c.rod_space,  -half_length, 0);            // 2
    s.addNode( c.rod_space,   half_length, 0);            // 3
    s.addNode(0,           -c.rod_space,   -half_length); // 4
    s.addNode(0,           -c.rod_space,    half_length); // 5
    s.addNode(0,            c.rod_space,   -half_length); // 6
    s.addNode(0,            c.rod_space,    half_length); // 7
    s.addNode(-half_length, 0,            c.rod_space);   // 8
    s.addNode( half_length, 0,            c.rod_space);   // 9
    s.addNode(-half_length, 0,           -c.rod_space);   // 10
    s.addNode( half_length, 0,           -c.rod_space);   // 11

    //Nodes for payload
    s.addNode(0,c.payload_h,0); //12 
    s.addNode(0,-c.payload_h,0); //13
	
    //Hedgehogcage
    /*s.addNode(-3, 4, 0); // 8 14
    s.addNode( 3, 4, 0); // 9 15
    s.addNode(0, 4, 3); // 10 16 
    s.addNode(0, 4, -3); // 11 17
    s.addNode(-3, 8, 0); // 12 18
    s.addNode(3, 8, 0); // 13 19 
    s.addNode(0, 8, 3); // 14 20 
    s.addNode(0, 8, -3); // 15 21
    */
    s.addNode(-1, 2, 0); // 8 14
    s.addNode( 1, 2, 0); // 9 15
    s.addNode(0, 2, 1); // 10 16 
    s.addNode(0, 2, -1); // 11 17
    s.addNode(-1, 6, 0); // 12 18
    s.addNode(1, 6, 0); // 13 19 
    s.addNode(0, 6, 1); // 14 20 
    s.addNode(0, 6, -1); // 15 21

}

void TensegrityHedgehogModel::addRods(tgStructure& s)
{
    // Struts
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair(10, 11, "rod");

    // Payload
    //s.addPair(12, 13, "payload_rod");
	
    //Hedgehogcage
    s.addPair( 14,  18, "rod");
    s.addPair( 15,  19, "rod");
    s.addPair( 16,  20, "rod");
    s.addPair( 17,  21, "rod");
    s.addPair( 14,  16, "rod");
    s.addPair( 14,  17, "rod");
    s.addPair( 15,  16, "rod");
    s.addPair( 15,  17, "rod");
    s.addPair( 18,  20, "rod");
    s.addPair( 18,  21, "rod");
    s.addPair( 19,  20, "rod");
    s.addPair( 19,  21, "rod");
    //XPairs Top
    s.addPair( 18,  19, "rod");
    s.addPair( 20,  21, "rod");
    //bottom
    s.addPair( 14,  21, "rod");
    s.addPair( 16,  17, "rod");
    s.addPair( 15,  20, "rod");
    s.addPair( 16,  19, "rod");
    s.addPair( 15,  21, "rod");
    s.addPair( 16,  18, "rod");
    s.addPair( 14,  20, "rod");
    s.addPair( 17,  19, "rod");
    s.addPair( 14,  21, "rod");
    s.addPair( 17,  18, "rod");
}


void TensegrityHedgehogModel::addMuscles(tgStructure& s)
{
    // Outer Cables
    s.addPair(0, 4,  "muscle");
    s.addPair(0, 5,  "muscle");
    s.addPair(0, 8,  "muscle");
    s.addPair(0, 10, "muscle");

    s.addPair(1, 6,  "muscle");
    s.addPair(1, 7,  "muscle");
    s.addPair(1, 8,  "muscle");
    s.addPair(1, 10, "muscle");

    s.addPair(2, 4,  "muscle");
    s.addPair(2, 5,  "muscle");
    s.addPair(2, 9,  "muscle");
    s.addPair(2, 11, "muscle");

    s.addPair(3, 7,  "muscle");
    s.addPair(3, 6,  "muscle");
    s.addPair(3, 9,  "muscle");
    s.addPair(3, 11, "muscle");

    s.addPair(4, 2,  "muscle");
    s.addPair(4, 10, "muscle");
    s.addPair(4, 11, "muscle");

    s.addPair(5, 8,  "muscle");
    s.addPair(5, 9,  "muscle");

    s.addPair(6, 10, "muscle");
    s.addPair(6, 11, "muscle");

    s.addPair(7, 8,  "muscle");
    s.addPair(7, 9,  "muscle");

    // Payload Muscles 
    /*
    s.addPair(0, 13, "muscle_in"); 
    s.addPair(1, 12, "muscle_in");
    s.addPair(2, 13, "muscle_in");
    s.addPair(3, 12, "muscle_in");
    s.addPair(4, 13, "muscle_in");
    s.addPair(5, 13, "muscle_in");
    s.addPair(6, 12, "muscle_in");
    s.addPair(7, 12, "muscle_in");
    s.addPair(8, 13, "muscle_in");
    s.addPair(9, 12, "muscle_in");
    s.addPair(10, 13, "muscle_in");
    s.addPair(11, 12, "muscle_in");*/
    
    // Hedgehog Cage
    s.addPair(0, 14, "muscle_in");
    s.addPair(1, 15, "muscle_in");
    s.addPair(2, 16, "muscle_in");
    s.addPair(3, 17, "muscle_in");
    s.addPair(4, 18, "muscle_in");
    s.addPair(5, 19, "muscle_in");
    s.addPair(6, 20, "muscle_in");
    s.addPair(7, 21, "muscle_in");
    
    //s.addPair(8, 13, "muscle_in");
    //s.addPair(9, 12, "muscle_in");
    //s.addPair(10, 13, "muscle_in");
    //s.addPair(11, 12, "muscle_in");



}

void TensegrityHedgehogModel::addNodes(tgStructure& y) {
    addBoxNodes();
 
        for(std::size_t i=0;i<nodes.size();i+=2) {
        y.addNode(nodes[i]);
        y.addNode(nodes[i+1]);
        y.addPair(i, i+1, "box");
    }
    //y.move(btVector3(0, 50, 0)); //  Ability to move box from this function
}

//Hedgehog Box Nodes
void TensegrityHedgehogModel::addBoxNodes() {
    tgNode node; //Creates the height of Hedgehog based on Nodes

    double x1 = 0; // use the distance between vector 1 and vector 2 to control the height
    double x2 = 1.75;
    double y1 = 0;
    double y2 = 1.75;
    double z1 = 0;
    double z2 = 1.75*sqrt(2); // The length of the diagonal is the input and the output is the height
	//For instance if you want a 1 meter height cube you need to provide sqrt 2 for the height

    node = tgNode(x1, y1, z1, "node");
    nodes.push_back(node);

    node = tgNode(x2, y2, z2, "node");
    nodes.push_back(node);
}

void TensegrityHedgehogModel::setup(tgWorld& world)
{
	//Define configuration for hedgehog
    const tgBox::Config boxConfig(c.width, c.height, c.density1, c.friction1, c.rollfriction1, c.restitution1);
	
	// Define the configurations of the rods and strings
    const tgRod::Config rodConfig(c.radius, c.density, c.friction, 
				c.rollfriction, c.restitution);

    const tgRod::Config payConfig(c.radius_pay, c.density_pay, c.friction, 
                c.rollfriction, c.restitution);
    
    // Define configuration for actuators
    tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension * c.stiffness / c.stiffness_in, c.history,
					    c.maxTens, c.targetVelocity);

    tgSpringCableActuator::Config muscleInConfig(c.stiffness_in, c.damping_in, c.pretension, c.history,
                        c.maxTens, c.targetVelocity);

    // Start creating the structures
    tgStructure s;
    tgStructure y; // Hedgehog
    // Add nodes to the structure
    addNodes(y);  //Nodes of Hedgehog
    addNodes(s, c.triangle_length, c.triangle_height, c.prism_height);
    addRods(s);
    //addNodes(s); //Fun Glitch: if activated rods become boxes

    addMuscles(s);

    
    //Initial location of tensegrity
    s.move(btVector3(0, 15, 0));
    
    //Initial Location and orientation of Hedgehog
    y.move(btVector3(-10,5, -10));
    y.addRotation(btVector3(-10,5,-10),btVector3(0,0,1), 180); // Z blue Axis
    y.addRotation(btVector3(-10,5,-10),btVector3(1,0,0), 260); // X red axis

    // Add a rotation to land the struture on a V.
    btVector3 rotationPoint1 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis1 = btVector3(1, 0, 0);  // x-axis
    double rotationAngle1 = 0.4636; //M_PI/2;
    s.addRotation(rotationPoint1, rotationAxis1, rotationAngle1);
    // Add a rotation to move structure towards triangle.
    btVector3 rotationPoint2 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis2 = btVector3(0, 1, 0);  // z-axis
    double rotationAngle2 = 1.991; 
    s.addRotation(rotationPoint2, rotationAxis2, rotationAngle2);
    // Add a rotation to land the struture on a triangle.
    btVector3 rotationPoint3 = btVector3(0, 0, 0); // origin
    btVector3 rotationAxis3 = btVector3(-1, 0, 0);  // x-axis
    double rotationAngle3 = 0.58895; 
    s.addRotation(rotationPoint3, rotationAxis3, rotationAngle3);

    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("payload_rod", new tgRodInfo(payConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    spec.addBuilder("muscle_in", new tgBasicActuatorInfo(muscleInConfig));
    spec.addBuilder("box", new tgBoxInfo(boxConfig));//Hedgehog
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec); //Tensegrity
    tgStructureInfo structureInfos(y, spec); //Hedgehog

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world); //Tensegrity
    structureInfos.buildInto(*this, world); //Hedgehog

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allMuscles = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());

    // call the onSetup methods of all observed things e.g. controllers
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
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& TensegrityHedgehogModel::getAllMuscles() const
{
    return allMuscles;
}

std::vector<tgRod*>& TensegrityHedgehogModel::getAllRods()
{
    return allRods;
}

void TensegrityHedgehogModel::teardown()
{
    tgModel::teardown();
}

