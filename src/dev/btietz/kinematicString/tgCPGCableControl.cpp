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

#include "tgCPGCableControl.h"

#include "core/tgSpringCableActuator.h"
#include "core/tgBulletSpringCableAnchor.h"
#include "controllers/tgImpedanceController.h"
#include "controllers/tgPIDController.h"
#include "core/tgCast.h"

// The C++ Standard Library
#include <iostream>
#include <stdexcept>
#include <vector>

tgCPGCableControl::tgCPGCableControl(tgPIDController::Config pid_config, const double controlStep) :
m_config(pid_config),
tgCPGStringControl(controlStep),
m_PID(NULL)
{
    if (m_controlStep < 0.0)
    {
        throw std::invalid_argument("Negative control step");
    }
}

tgCPGCableControl::~tgCPGCableControl()
{
    delete m_PID;
}

void tgCPGCableControl::onSetup(tgSpringCableActuator& subject)
{
    m_PID = new tgPIDController(&subject, m_config);
}

void tgCPGCableControl::onStep(tgSpringCableActuator& subject, double dt)
{
    assert(&subject == m_PID->getControllable());
    
    m_controlTime += dt;
	m_totalTime += dt;

    if (m_controlTime >= m_controlStep)
    {
        
		m_commandedTension = motorControl().control(*m_PID, dt, controlLength(), getCPGValue());

        m_controlTime = 0;
    }
    else
    {
		const double currentTension = subject.getTension();
		m_PID->control(dt, m_commandedTension, currentTension);
	}
}
