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

#include "simpleMuscleNP.h"
#include "core/tgModelVisitor.h"
#include "core/tgBulletUtil.h"
#include "core/tgWorld.h"
#include "core/tgLinearString.h"
#include "core/tgBaseString.h"
#include "core/tgRod.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgLinearStringInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
#include "dev/muscleNP/tgMultiPointStringInfo.h"
simpleMuscleNP::simpleMuscleNP()
{
}

simpleMuscleNP::~simpleMuscleNP()
{
}

void simpleMuscleNP::setup(tgWorld& world)
{


	const double rodDensity = 1; // Note: This needs to be high enough or things fly apart...
	const double rodRadius = 0.25;
	const tgRod::Config rodConfig(rodRadius, rodDensity);
	const tgRod::Config rodConfig2(rodRadius, 0.0);
	
	tgStructure s;
	
#if (0)
	s.addNode(0,0,0);
	s.addNode(0,3,0);
	s.addNode(2,5,0);
	s.addNode(-2,5,0);
	s.addNode(0, 10, 0);
	s.addNode(0, 7, 0);
	s.addNode(0, 5, 2);
	s.addNode(0, 5,-2);
	s.addPair(0, 1, "rod");
	s.addPair(1, 2, "rod");
	s.addPair(1, 3, "rod");
	s.addPair(4, 5, "rod");
	s.addPair(5, 6, "rod");
	s.addPair(5, 7, "rod");
	s.addPair(2, 6, "muscle");
	s.addPair(3, 7, "muscle");
	s.addPair(2, 7, "muscle");
	s.addPair(3, 6, "muscle");
	s.move(btVector3(0, 10, 0));
#else
	s.addNode(-2, 2, 0);
	s.addNode(0, 2, 0);
	s.addNode(10, 2, 0);
	s.addNode(12, 2, 0);
	s.addNode(5, 3, -2);
	s.addNode(5, 3, 2);
	s.addPair(0, 1, "rod2");
	s.addPair(2, 3, "rod2");
	//s.addPair(4, 5, "rod");
	s.addPair(1, 2, "muscle");
	s.move(btVector3(0, 0, 0));
#endif

	// Move the structure so it doesn't start in the ground
	s.move(btVector3(0, 0, 0));
	tgBaseString::Config muscleConfig(1000, 10, false, 0, 600000000);
	
	// Create the build spec that uses tags to turn the structure into a real model
	tgBuildSpec spec;
	spec.addBuilder("rod", new tgRodInfo(rodConfig));
	spec.addBuilder("rod2", new tgRodInfo(rodConfig2));
#if (1)
	spec.addBuilder("muscle", new tgMultiPointStringInfo(muscleConfig));
#else
	spec.addBuilder("muscle", new tgLinearStringInfo(muscleConfig));
#endif
	// Create your structureInfo
	tgStructureInfo structureInfo(s, spec);
	// Use the structureInfo to build ourselves
	structureInfo.buildInto(*this, world);
	// We could now use tgCast::filter or similar to pull out the
	// models (e.g. muscles) that we want to control.
	allMuscles = tgCast::filter<tgModel, tgLinearString> (getDescendants());
	allRods = tgCast::filter<tgModel, tgRod> (getDescendants());
	
	notifySetup();
	totalTime = 0.0;
	tgModel::setup(world);
}

void simpleMuscleNP::teardown()
{
	tgModel::teardown();
}

void simpleMuscleNP::step(double dt)
{
	totalTime += dt;
	
	allMuscles[0]->setRestLength(3.0, dt);
	
	btVector3 com(0, 0, 0);
	btScalar mass = 0;
	btScalar energy = 0;
	for (std::size_t i = 0; i < allRods.size(); i++)
	{
		tgRod& ri = *(allRods[i]);
		com += ri.centerOfMass() * ri.mass();
		mass += ri.mass();
	}

	//std::cout << allMuscles[0]->getTension() << std::endl;
	tgModel::step(dt);
}

/**
* Call tgModelVisitor::render() on self and all descendants.
* @param[in,out] r a reference to a tgModelVisitor
*/
void simpleMuscleNP::onVisit(const tgModelVisitor& r) const
{
	r.render(*this);
	tgModel::onVisit(r);
}
