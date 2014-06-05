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
 * @file tgRigidInfo.cpp
 * @brief Contains the definitions of members of class tgRigidInfo
 * $Id$
 */

// This module
#include "tgRigidInfo.h"
// This application
#include "tgNode.h"
#include "tgNodes.h"
#include "tgPair.h"
#include "tgPairs.h"
// The NTRT Core Libary
#include "core/tgBulletUtil.h"
#include "core/tgTagSearch.h"
#include "tgUtil.h"
#include "core/tgWorld.h"

tgRigidInfo* tgRigidInfo::createRigidInfo(const tgNode& node, const tgTagSearch& tagSearch)
{
    // Our subclasses may not be able to create rigidInfos based on nodes. Also, the
    // tags may not match our search. Make sure both work, or return 0. 
    tgRigidInfo* rigidInfo = 0;
    if(tagSearch.matches(node.getTags())) {
        rigidInfo = createRigidInfo(node);
    }
    return rigidInfo;
}

std::vector<tgRigidInfo*> tgRigidInfo::createRigidInfos(const tgNodes& nodes, const tgTagSearch& tagSearch)
{
    std::vector<tgRigidInfo*> result;
    for(int i = 0; i < nodes.size(); i++) {
        tgRigidInfo* r = createRigidInfo(nodes[i], tagSearch);
        if(r != 0) {
            r->addTags(nodes[i].getTags());
            result.push_back(r);
        }
    }
    return result;
}

tgRigidInfo* tgRigidInfo::createRigidInfo(const tgPair& pair, const tgTagSearch& tagSearch)
{
    // Our subclasses may not be able to create rigidInfos based on nodes. Also, the
    // tags may not match our search. Make sure both work, or return 0. 
    tgRigidInfo* rigidInfo = 0;
    if(tagSearch.matches(pair.getTags())) {
        rigidInfo = createRigidInfo(pair);
    }
    return rigidInfo;
}

std::vector<tgRigidInfo*> tgRigidInfo::createRigidInfos(const tgPairs& pairs, const tgTagSearch& tagSearch)
{
    std::vector<tgRigidInfo*> result;
    for(int i = 0; i < pairs.size(); i++) {
        tgRigidInfo* r = createRigidInfo(pairs[i], tagSearch);
        if(r != 0) {
            result.push_back(r);
        }
    }
    return result;
}



void tgRigidInfo::initRigidBody(tgWorld& world)
    {
        if(!getRigidBody()) {

            // we want to do this based on group instead the rigid itself; otherwise we throw away autocompounding.
            tgRigidInfo* rigid = getRigidInfoGroup();

            // If we're not using autocompounding, use the rigid body itself.
            // NOTE: This means that auto-compounding can be silently skipped, which means that your parts may not be joined correctly. Do we want that?
            if(rigid == 0) { 
                rigid = this;
            }

            if (rigid->getRigidBody() == NULL) { // Init only if it doesn't have a btRigidBody (has already been initialized)

                double mass = rigid->getMass();
                btTransform transform = rigid->getTransform();
                btCollisionShape* shape = rigid->getCollisionShape(world);
                
                btRigidBody* body = 
          tgBulletUtil::createRigidBody(&tgBulletUtil::worldToDynamicsWorld(world),
                        mass,
                        transform,
                        shape);

                rigid->setRigidBody(body);
            }
        }
    }