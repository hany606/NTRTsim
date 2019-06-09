/**
 * @file sixBarsModel.cpp
 * @brief Contains the definition of the members of the class sixBarsModel.
 * This file is built over one of the examples from NTRTsim from NASA
 */

// This module
#include "sixBarsModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgRod.h"
#include "tgcreator/tgBuildSpec.h"
#include "tgcreator/tgBasicActuatorInfo.h"
#include "tgcreator/tgRodInfo.h"
#include "tgcreator/tgStructure.h"
#include "tgcreator/tgStructureInfo.h"
// The Bullet Physics library
#include "LinearMath/btVector3.h"
// The C++ Standard Library
#include <stdexcept>
#include<stdio.h>

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
        double length;
    } c =
   {
       0.2,     // density (mass / length^3)
       0.31,     // radius (length)
       1000.0,   // stiffness (mass / sec^2)
       10.0,     // damping (mass / sec)
       5000.0,     // pretension (mass * length / sec^2)
       10.0
  };
} // namespace

sixBarsModel::sixBarsModel() :
tgModel() 
{
}

sixBarsModel::~sixBarsModel()
{
}

void sixBarsModel::addNodes(tgStructure& s, double length)
{   

    //Bar 1 Vertical
    s.addNode(-0.5*length,length,length);
    s.addNode(-0.5*length,length,-length);
    //Bar 2 Vertical
    s.addNode(0.5*length,length,length);
    s.addNode(0.5*length,length,-length);

    // Bar 3 Horizontal
    s.addNode(-length,1.5*length,0);
    s.addNode(length,1.5*length,0);
    //Bar 4 Horizontal
    s.addNode(-length,0.5*length,0);
    s.addNode(length,0.5*length,0);


    //Bar 3D
    s.addNode(0,2*length,-0.5*length);
    s.addNode(0,0,-0.5*length);
    //Bar 3D
    s.addNode(0,2*length,0.5*length);
    s.addNode(0,0,0.5*length);
    
    //Best framework configuration
    // //Bar 1 Vertical
    // s.addNode(-length,length,-length);
    // s.addNode(-length,length,length);
    // //Bar 2 Vertical
    // s.addNode(length,length,-length);
    // s.addNode(length,length,length);

    // // Bar 3 Horizontal
    // s.addNode(-length,1.5*length,0);
    // s.addNode(length,1.5*length,0);
    // //Bar 3 Horizontal
    // s.addNode(-length,0.5*length,0);
    // s.addNode(length,0.5*length,0);


    // //Bar 3D
    // s.addNode(0,2*length,-0.5*length);
    // s.addNode(0,0,-0.5*length);
    // //Bar 3D
    // s.addNode(0,2*length,0.5*length);
    // s.addNode(0,0,0.5*length);

}

void sixBarsModel::addRods(tgStructure& s)
{
    s.addPair( 0,  1, "rod");
    s.addPair( 2,  3, "rod");
    s.addPair( 4,  5, "rod");
    s.addPair( 6,  7, "rod");
    s.addPair( 8,  9, "rod");
    s.addPair( 10,  11, "rod");

}

void sixBarsModel::addMuscles(tgStructure& s)
{
    //Each node is connected with 4 Strings/Cables/Muscles
    //Which means it should be repeated only four times
    //-1 means redundent and not use it it already has been configured
    //as a,b = b,a (unodred pairs)
    //rows means the starting node and the columns is the index of the cell
    //cell value is the end node
    int pairs[12][4] = {{4,6,10,11},{4,6,8,9},         //0,1
                        {5,7,10,11},{5,7,8,9},         //2,3
                        {-1,-1,8,10},{-1,-1,8,10},     //4,5
                        {-1,-1,9,11},{-1,-1,9,11},     //6,7
                        {-1,-1,-1,-1},{-1,-1,-1,-1},   //8,9
                        {-1,-1,-1,-1},{-1,-1,-1,-1}    //10,11
    };
    for(int i = 0; i < 12; i++){
        for(int j = 0; j < 4; j++){
            if(pairs[i][j] == -1)
                continue;
            printf("%d - %d\n",i,pairs[i][j]);
            s.addPair(i,pairs[i][j], "muscle");
        }
    }
}

void sixBarsModel::setup(tgWorld& world)
{
    // Define the configurations of the rods and strings
    // Note that pretension is defined for this string
    const tgRod::Config rodConfig(c.radius, c.density);
    const tgSpringCableActuator::Config muscleConfig(c.stiffness, c.damping, c.pretension);
    
    // Create a structure that will hold the details of this model
    tgStructure s;
    
    // Add nodes to the structure
    addNodes(s,c.length);
    
    // Add rods to the structure
    addRods(s);
    
    // Add muscles to the structure
    addMuscles(s);
    
    // Move the structure so it doesn't start in the ground
    s.move(btVector3(0, 10, 0));
    
    // Create the build spec that uses tags to turn the structure into a real model
    tgBuildSpec spec;
    spec.addBuilder("rod", new tgRodInfo(rodConfig));
    spec.addBuilder("muscle", new tgBasicActuatorInfo(muscleConfig));
    
    // Create your structureInfo
    tgStructureInfo structureInfo(s, spec);

    // Use the structureInfo to build ourselves
    structureInfo.buildInto(*this, world);

    // We could now use tgCast::filter or similar to pull out the
    // models (e.g. muscles) that we want to control. 
    allActuators = tgCast::filter<tgModel, tgSpringCableActuator> (getDescendants());
    
    // Notify controllers that setup has finished.
    notifySetup();
    
    // Actually setup the children
    tgModel::setup(world);
}

void sixBarsModel::step(double dt)
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

void sixBarsModel::onVisit(tgModelVisitor& r)
{
    // Example: m_rod->getRigidBody()->dosomething()...
    tgModel::onVisit(r);
}

const std::vector<tgSpringCableActuator*>& sixBarsModel::getAllActuators() const
{
    return allActuators;
}
    
void sixBarsModel::teardown()
{
    notifyTeardown();
    tgModel::teardown();
}
