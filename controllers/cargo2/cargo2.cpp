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
#include <webots/Connector.hpp>
#include "DetectedMarker.hpp"

// All the webots classes are defined in the "webots" namespace
using namespace webots;

/**
 * @brief Get the crate position object
 * 
 * @param camera 
 * @return std::vector<DetectedMarker> 
 */
std::vector<DetectedMarker> get_crate_position(Camera *camera)
{
  std::vector<DetectedMarker> crate_markers;
  auto markers = get_visible_markers(camera);
  DetectedMarker right, left;
  for (const auto &marker : markers) {
    if (marker.id == 100) { // Assuming marker ID 1 corresponds to the crate
      right = marker;
    } else if (marker.id == 101) { // Assuming marker ID 2 corresponds to the crate
      left = marker;
    }
  }
  if (right.id != 0 && left.id != 0) {
    crate_markers.push_back(right);
    crate_markers.push_back(left);
  }
  return crate_markers;
}

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
  auto lift_motor = robot->getMotor("lift motor");
  lift_motor->setPosition(0.0);
  lift_motor->setVelocity(0.0);

  Connector *connector = robot->getConnector("connector");

  DistanceSensor *front_ds = robot->getDistanceSensor("distance sensor"); // Replace with the name of your distance sensor");
  front_ds->enable(timeStep);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    static int state = 0;
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    auto front_detectedMarkers = get_visible_markers(front_camera);
    auto bottom_detectedMarkers = get_visible_markers(bottom_camera);
    auto rear_detectedMarkers = get_visible_markers(rear_camera);
    auto crate_detectedMarkers = get_crate_position(lift_camera);
    auto distance = front_ds->getValue();

    switch (state) {
      case 0:
        printf("Searching for the crate using the lift camera...\n");
        // Move forward until the crate is detected by the lift camera.
        if (!crate_detectedMarkers.empty()) {
          printf("detect!\n");
          state = 1;
        }
        break;
      case 1:
        printf("Crate detected. Adjusting the lift to prepare for grasping...\n");
        // Adjust the lift to prepare for grasping the crate.
        if (crate_detectedMarkers[0].y > lift_camera->getHeight() / 2) {
          // If the crate is below the center of the camera view, move the lift down.
          lift_motor->setVelocity(-0.5);
          lift_motor->setPosition(lift_motor->getTargetPosition() - 0.01); // Move down by a small increment
        } else if (crate_detectedMarkers[0].y < lift_camera->getHeight() / 2) {
          // If the crate is above the center of the camera view, move the lift up.
          lift_motor->setVelocity(0.5);
          lift_motor->setPosition(lift_motor->getTargetPosition() + 0.01); // Move up by a small increment
        } else {
          // If the crate is centered, stop the lift and transition to the next state.
          lift_motor->setVelocity(0.0);
          state = 2;
        }
        break;
      case 2:
        printf("Lift adjusted. Aproaching the crate...\n");
        printf("Distance to the crate: %f\n", distance);
        if (distance > 800) {
          right_motor->setVelocity(0.05);
          left_motor->setVelocity(0.05);
          connector->enablePresence(timeStep);
          state = 3;
        } else if (distance > 500) {
          right_motor->setVelocity(0.1);
          left_motor->setVelocity(0.1);
        } else {
          right_motor->setVelocity(0.5);
          left_motor->setVelocity(0.5);
        }
        break;
      case 3:
        if (connector->getPresence() == 1) {
          printf("Crate grasped. Crate lifting...\n");
          right_motor->setVelocity(0);
          left_motor->setVelocity(0);
          for (auto i = 0; i < 5; i++) {
            robot->step(timeStep);
          }
          connector->disablePresence();
          connector->lock();
          // wait for a few steps to ensure the crate is securely grasped before lifting
          while (connector->isLocked() == false) {
            robot->step(timeStep);
          }
          for (auto i = 0; i < 100; i++) {
            robot->step(timeStep);
          }
          lift_motor->setVelocity(0.05);
          lift_motor->setPosition(lift_motor->getTargetPosition() + 0.5); // Lift the crate up
          // wait for a few steps to ensure the crate is lifted before moving
          for (auto i = 0; i < 50; i++) {
            robot->step(timeStep);
          }
          state = 4;
        } else {
          printf("Grasping the crate...\n");
        }
        break;
        case 4:
          printf("Crate lifted. Moving to the drop-off location...\n");
          lift_motor->setVelocity(0.0);
          right_motor->setVelocity(-0.5);
          left_motor->setVelocity(-0.5);
          // You can add logic here to navigate to the drop-off location using the cameras and distance sensor.
          break;
    }
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
