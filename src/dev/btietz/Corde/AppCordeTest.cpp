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
 * @file AppCordeTest.cpp
 * @brief Contains the definition function main() for testing the Corde
 * string model
 * @author Brian Mirletz
 * $Id$
 */

// This application
#include "dev/btietz/tgCordeModel.h"
// This library
#include "core/terrain/tgBoxGround.h"
#include "core/tgModel.h"
#include "core/tgSimViewGraphics.h"
#include "core/tgSimulation.h"
#include "core/tgWorld.h"
#include "tgcreator/tgUtil.h"
// The Bullet Physics Library
#include "LinearMath/btVector3.h"
#include "LinearMath/btQuaternion.h"
// The C++ Standard Library
#include <iostream>

/**
 * The entry point.
 * @param[in] argc the number of command-line arguments
 * @param[in] argv argv[0] is the executable name
 * @return 0
 */
int main(int argc, char** argv)
{
	#ifdef BT_USE_DOUBLE_PRECISION
		std::cout << "Double precision" << std::endl;
	#else
		std::cout << "Single Precision" << std::endl;
	#endif
    std::cout << "AppCordeModelTest" << std::endl;

    // First create the ground and world. Specify ground rotation in radians
    const double yaw = 0.0;
    const double pitch = 0.0;
    const double roll = 0.0;
    const tgBoxGround::Config groundConfig(btVector3(yaw, pitch, roll));
    // the world will delete this
    tgBoxGround* ground = new tgBoxGround(groundConfig);
    
    const tgWorld::Config config(981); // gravity, cm/sec^2
    
    tgWorld world(config, ground);

    // Second create the view
    const double timestep_physics = 1.0/100000.0; // seconds
    const double timestep_graphics = 1.f/60.f; // seconds
    tgSimViewGraphics view(world, timestep_physics, timestep_graphics);

    // Third create the simulation
    tgSimulation simulation(view);

	    // Fourth create the models with their controllers and add the models to the
    // simulation
    tgCordeModel* const myModel = new tgCordeModel();
    
    // Add the model to the world
    simulation.addModel(myModel);
    
    simulation.run();
	
    return 0;
}
