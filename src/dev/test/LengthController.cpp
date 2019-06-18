/**
 * @file LengthController.h
 * @brief Implementation of class LengthController
 * @author Hany Hamed
 * This was built over one of the source codes of NTRTsim codes
 */

// This module
#include "LengthController.h"

// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <time.h>
#include <iostream>
#include <fstream>
#include <string>

// ROS libraries

// #include "ros/ros.h"
// #include "jsoncpp/json/json.h"



// using namespace std;

LengthController::LengthController(const double length) :
  m_length(length)
{ 
  LengthController::rosBridge = new ROS_Bridge();
  if (length < 0.0)
    {
      throw std::invalid_argument("Negative length");
    }
}

LengthController::~LengthController()
{
}	

void LengthController::onSetup(sixBarsModel& subject)
{

  m_controllers.clear(); //clear vector of controllers
  start_lengths.clear(); //vector of randomized restlengths
  
  //get all of the tensegrity structure's cables
  actuators = subject.getAllActuators();

  //Attach a tgBasicController to each actuator
  for (size_t i = 0; i < actuators.size(); ++i)
    {
      tgBasicActuator * const pActuator = actuators[i];
      assert(pActuator != NULL);
      //instantiate controllers for each cable
      tgBasicController* m_lenController = new tgBasicController(pActuator, m_length);
      //add controller to vector
      m_controllers.push_back(m_lenController);
      //generate random end restlength
      double start_length = actuators[i]->getStartLength();
      printf("Start Lenght %d: %lf\n", (int) i,  start_length);
      start_lengths.push_back(start_length);
      rosBridge->setController(i,start_length);
    }
}

//This function is being activated each step
void LengthController::onStep(sixBarsModel& subject, double dt)
{

  if (dt <= 0.0) {
    throw std::invalid_argument("dt is not positive");
  }
  else {
    globalTime += dt;
    if(globalTime > 2){ //delay start of cable actuation
      if(toggle==0){    //print once when motors start moving
        std::cout << std::endl << "Activating Cable Motors (Randomized Lengths) -------------------------------------" << std::endl;
	      toggle = 1;   //is used like a state flag
        // Json::FastWriter fastWriter;
        // Json::Value obj;
        // obj["ACtion"] = "12";
        // std::cout << fastWriter.write(obj) << "\n";
        // wr.write(obj,);
        // ros::init(argc,argv,"Name");
        // ros::NodeHandle node_obj;
        // // ros::Publisher number_publisher = node_obj.advertise<std_msgs::Int32>("/numbers",10);
        // ros::Publisher number_publisher = node_obj.advertise<std_msgs::String>("chatter", 1000);
        // ros::Rate loop_rate(10);
        int number_count = 0;
      }
      if(toggle==1){
        toggle = 2;
        m_controllers[0]->control(dt,12);
        actuators[0]->moveMotors(dt);

        //This to confirm that it had been changed
        if(actuators[0]->getRestLength()!=12)
          toggle = 1;
        /*
        for(int i = 0; i<actuators.size(); i++){
          m_controllers[i]->control(dt,rand_lengths[i]);
          actuators[i]->moveMotors(dt);

          //This to confirm that it had been changed
          if(actuators[i]->getRestLength()!=rand_lengths[i])
            toggle = 1;
        }	
        */
      }
      if(toggle==2){
        // char** argv = 0;
        // int argc = 0;
        // if (ros::ok())
        // {
            // std_msgs::Int32 msg;
            // std_msgs::String msg;
            // msg.data = "HEYYYY";
            // ROS_INFO("%s",msg.data.c_str());
            // number_publisher.publish(msg);
            // ros::spinOnce();
            // loop_rate.sleep();
        // }
        // toggle = 1;
        // m_controllers[0]->control(dt,10);
        // actuators[0]->moveMotors(dt);

        //This to confirm that it had been changed
        if(actuators[0]->getRestLength()!=10)
          toggle = 2;
        /*
        for(int i = 0; i<actuators.size(); i++){
          m_controllers[i]->control(dt,start_lengths[i]);
          actuators[i]->moveMotors(dt);
          //This to confirm that it had been changed
          if(actuators[i]->getRestLength()!=start_lengths[i])
            toggle = 2;
        }
        */
      }
    }
  }

}

