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
 * @file AppTensegrityHedgehog.cpp
 * @brief Contains the definition function main() for the Tensegrity Hedgdehog Application
  * @author Manuel Retana
 * $Id$
 */

// This application
#include "TensegrityHedgehogModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
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
 
 /*tgBoxGround *createGround();
tgWorld *createWorld();
tgSimViewGraphics *createGraphicsView(tgWorld *world);
tgSimView *createView(tgWorld *world);
void simulate(tgSimulation *simulation);*/

int main(int argc, char** argv)
{
    std::cout << "AppTensegrityHedgehog" << std::endl;

    // First create the ground and world
    
    // Determine the angle of the ground in radians. All 0 is flat
    const double yaw = 0.0;
    const double pitch = 0.0; 
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    
    const tgWorld::Config config(9.81); // gravity, cm/sec^2  Use this to adjust length scale of world.
        // Note, by changing the setting below from 981 to 98.1, we've
        // scaled the world length scale to decimeters not cm.
    tgWorld world(config, ground);

   // Second create the view
    const double timestep_physics = 0.0001; // Seconds
    const double timestep_graphics = 1.f/60.f; // Seconds // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

    // Fourth create the models with their controllers and add the models to the
    // simulation
    TensegrityHedgehogModel* const myModel = new TensegrityHedgehogModel();

    simulation.addModel(myModel);
    
    // Run until the user stops
    simulation.run();

    //Teardown is handled by delete, so that should be automatic
    return 0;
}
/*
 //Use for displaying tensegrities in simulation 
tgSimViewGraphics *createGraphicsView(tgWorld *world) {
    const double timestep_physics = 1.0 / 60.0 / 10.0; // Seconds
    const double timestep_graphics = 1.f /60.f; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    return new tgSimViewGraphics(*world, timestep_physics, timestep_graphics); 
}

//Use for trial episodes of many tensegrities in an experiment 
tgSimView *createView(tgWorld *world) {
    const double timestep_physics = 1.0 / 60.0 / 10.0; // Seconds
    const double timestep_graphics = 1.f /60.f; // Seconds, AKA render rate. Leave at 1/60 for real-time viewing
    return new tgSimView(*world, timestep_physics, timestep_graphics); 
}

void simulate(tgSimulation *simulation) {
    int nEpisodes = 5; // Number of episodes ("trial runs")
    int nSteps = 3000; // Number of steps in each episode, 60k is 100 seconds (timestep_physics*nSteps)
    for (int i=0; i<nEpisodes; i++) {
        simulation->run(nSteps);
        simulation->reset();
    }
}
*/


//TODO: Create a switch scenario for a hilly world with the following code
// IGNORE FOR THE MOMENT

//HILLY TERRAIN
/**int main(int argc, char** argv)
*{
*	std::cout << "TensegrityHedgehogModel" << std::endl;
*#if (0)
*    // First create the ground and world. Specify ground rotation in radians
*	const double yaw = 0.0;
*	const double pitch = 0.0;
*	const double roll = 0.0;
*	const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
*
*    // the world will delete this
*	tgBoxGround* ground = new tgBoxGround(groundConfig);
*#else //terra
*    btVector3 eulerAngles=btVector3(0.0, 0.0, 0.0); //te
*   btScalar friction = 0.5; //te
*   btScalar restitution = 0.0; //te
*   // Size doesn't affect hilly terrain
*   btVector3 size = btVector3(0.0, 0.1, 0.0);
*   btVector3 origin = btVector3(0.0, 0.0, 0.0);
*   size_t nx = 100;
*   size_t ny = 100;
*   double margin = 0.5;
*   double triangleSize = 5.0;
*   double waveHeight = 3.0;
*   double offset = 0.0;
*    tgHillyGround::Config groundConfig(eulerAngles, friction, restitution,
*                                    size, origin, nx, ny, margin, triangleSize,
*                                    waveHeight, offset);
*
*    tgHillyGround* ground = new tgHillyGround(groundConfig);
*
*
*#endif
*    //double sf = 10; terra change
*    //double gravity = 9.81*sf; terra change //Do not forget to change this numbering 7/28 to the same scale above
*	const tgWorld::Config config(98.1); // gravity, cm/sec^2 98.1
*	tgWorld world(config, ground);
*
*/

//HILLY TERRAIN