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
l */

#ifndef T12CONTROLLER
#define T12CONTROLLER

/**
 * @file T12Controller.h
 * @brief Contains the definition of class T12Controller.
 * @author Hannah Petersson based on code from Steven Lessard
 * @version 1.0.0
 * $Id$
 */

#include <vector>

#include "core/tgObserver.h"
#include "learning/Adapters/AnnealAdapter.h"

// Forward declarations
class T12Model;
class tgBasicActuator;

//namespace std for vectors
using namespace std;

/** Escape Controller for T6 */
class T12Controller : public tgObserver<T12Model>
{
    public:
        // Note that currently this is calibrated for decimeters.
        T12Controller(T12Model* subject, const double prefLength=5.0, double startTime=3);

        /** Nothing to delete, destructor must be virtual */
        virtual ~T12Controller() { }

        virtual void onSetup(T12Model& subject);

        virtual void onStep(T12Model& subject, double dt);

        virtual void onTeardown(T12Model& subject);

    protected:
        virtual vector< vector <double> > transformActions(vector< vector <double> > act);

        virtual void applyActions(T12Model& subject, vector< vector <double> > act);

    private:
        vector<double> initPosition; // Initial position of model
        const double m_initialLengths;
   	double m_startTime;
        double m_totalTime;
        double const maxStringLengthFactor; // Proportion of string's initial length by which a given actuator can increase/decrease

        // Evolution and Adapter
        AnnealAdapter evolutionAdapter;
        vector< vector<double> > actions; // For modifications between episodes

        // Muscle Clusters
        int nClusters;
        int musclesPerCluster;
        /** A vector clusters, each of which contains a vector of muscles */
        vector<vector<tgBasicActuator*> > clusters; 

        // Sine Wave Data
        double* amplitude;
        double* angularFrequency;
        double* phaseChange;
        double* dcOffset;

        /** Initialize the evolution adapter as well as its own parameters */
        void setupAdapter();

        /** Returns amount of energy spent by each muscle in subject */
        double totalEnergySpent(T12Model& subject);

        /** Sets target lengths for each muscle */
        void setPreferredMuscleLengths(T12Model& subject, double dt);

        /** Divides the 24 muscles of an Escape_T6Model 
         * into 8 clusters of 3 muscles */
        void populateClusters(T12Model& subject);

        /** Sets the amplitude, angularFrequency, phase change, and dcOffset 
         * for each sine wave used in muscle actuation */
        void initializeSineWaves();

        /** Difference in position between initPosition and finalPosition
         * of subject */
        double displacement(T12Model& subject);

        /** Select action paramters from a comma-separated line in a file */
        std::vector<double> readManualParams(int lineNumber, const char* filename);

        void printSineParams();
};

#endif // T12CONTROLLER