// File:          cargo2.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <stdio.h>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Lidar.hpp>
#include "DetectedMarker.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  Camera *bottom_camera = robot->getCamera("bottom camera");
  bottom_camera->enable(timeStep);
  bottom_camera->recognitionEnable(timeStep);
  Camera *front_camera = robot->getCamera("front camera");
  front_camera->enable(timeStep);
  front_camera->recognitionEnable(timeStep);
  Camera *rear_camera = robot->getCamera("rear camera");
  rear_camera->enable(timeStep);
  rear_camera->recognitionEnable(timeStep);
  Camera *lift_camera = robot->getCamera("lift camera");
  lift_camera->enable(timeStep);
  lift_camera->recognitionEnable(timeStep);

  Motor *right_motor = robot->getMotor("rightMotor");
  Motor *left_motor = robot->getMotor("leftMotor");
  right_motor->setPosition(INFINITY);
  left_motor->setPosition(INFINITY);
  right_motor->setVelocity(0.5);
  left_motor->setVelocity(0.5);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    auto front_detectedMarkers = get_visible_markers(front_camera);
    auto bottom_detectedMarkers = get_visible_markers(bottom_camera);
    auto rear_detectedMarkers = get_visible_markers(rear_camera);
    auto lift_detectedMarkers = get_visible_markers(lift_camera);
    // Process sensor data here.
    if (!front_detectedMarkers.empty()) {
      printf("Front camera detected marker ID: %d at (%d, %d)\n", front_detectedMarkers[0].id, front_detectedMarkers[0].x, front_detectedMarkers[0].y);
    }
    if (!bottom_detectedMarkers.empty()) {
      printf("Bottom camera detected marker ID: %d at (%d, %d)\n", bottom_detectedMarkers[0].id, bottom_detectedMarkers[0].x, bottom_detectedMarkers[0].y);
    }
    if (!rear_detectedMarkers.empty()) {
      printf("Rear camera detected marker ID: %d at (%d, %d)\n", rear_detectedMarkers[0].id, rear_detectedMarkers[0].x, rear_detectedMarkers[0].y);
    }
    if (!lift_detectedMarkers.empty()) {
      printf("Lift camera detected marker ID: %d at (%d, %d)\n", lift_detectedMarkers[0].id, lift_detectedMarkers[0].x, lift_detectedMarkers[0].y);
    } else {
      printf("No marker detected in front, bottom, rear, and lift cameras.\n");
    }


    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
