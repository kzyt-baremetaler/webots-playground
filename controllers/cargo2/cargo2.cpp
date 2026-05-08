// File:          cargo2.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <stdio.h>
#include <cmath>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Lidar.hpp>
#include <webots/Connector.hpp>
#include <webots/PositionSensor.hpp>
#include "CargoRobot.hpp"
#include "DetectedMarker.hpp"


// All the webots classes are defined in the "webots" namespace
using namespace webots;

/**
 * @brief 浮動小数点の比較
 * 
 * 浮動小数点数は誤差があるので、deltaの誤差を許容する
 * 
 * @param a 
 * @param b 
 * @param delta 
 * @return true 
 * @return false 
 */
inline bool neary_equal(double a, double b, double delta)
{
  return std::abs(a - b) < delta;
}

inline double angle_compare(double a1, double a2)
{
  return a1 - a2;
}

inline bool angle_isqual(double a1, double a2, double delta = 0.0001)
{
  return std::abs(angle_compare(a1, a2)) < delta;
}

/**
 * @brief 2点間の距離を算出する
 */
inline double distance(double x1, double y1, double x2, double y2)
{
  return std::sqrt((x1 - x2)*(x1-x2) + (y1-y2)*(y1-y2));
}

/**
 * @brief ターゲットまでの距離を算出
 * 
 * @param x 
 * @param y 
 * @return double 
 */
inline double distance_to_target(double x, double y)
{
  return distance(x, y, 0.0, 0.0);
}

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

static int ApproachState(Robot *robot) {
  static const double SEARCH_RANGE_INIT = 0.2;
  static int state = 0;
  static double search_range = SEARCH_RANGE_INIT;
  static auto cargo_robot = CargoRobot(robot);
  static auto lift_camera = robot->getCamera("lift camera");
  auto crate_detectedMarkers = get_crate_position(lift_camera);
  double crate_x, crate_y;

  if (!crate_detectedMarkers.empty()) {
    crate_x = (crate_detectedMarkers[0].pos_x + crate_detectedMarkers[1].pos_x) / 2;
    crate_y = (crate_detectedMarkers[0].pos_y + crate_detectedMarkers[1].pos_y) / 2 - 0.05;
  }
  switch (state) {
  case 0: // 左を探す
    printf("Crate not detected. Searching left...\n");
    if (!crate_detectedMarkers.empty()) {
      state = 2;
      cargo_robot.StopMove();
      search_range = SEARCH_RANGE_INIT;
    } else {
      // クレートが見つからないのでその場で旋回して探す。
      cargo_robot.StopMove();
      // 左を捜索
      if (cargo_robot.TurnBody(search_range) < 0.0) {
        state = 1; // 端に到達
      }
    }
    break;
  case 1: // 右を探す
    printf("Crate not detected. Searching right...\n");
    if (!crate_detectedMarkers.empty()) {
      state = 2;
      search_range = SEARCH_RANGE_INIT;
    } else {
      // クレートが見つからないのでその場で旋回して探す。
      cargo_robot.StopMove();
      // 右を捜索
      if (cargo_robot.TurnBody(-search_range) > 0.0) {
        state = 0;// 見つからない！
        search_range *= 2;  // 範囲拡大
      }
    }
    break;
  case 2: // カメラの正面にとらえる
    printf("Crate detected. Lock on...\n");
    if (crate_detectedMarkers.empty()) {
      state = 0;
    } else {
      double angle_to_crate = atan2(crate_y, crate_x);
      double angle_body = cargo_robot.BodyAngle();
      printf("Angle to crate %f\n", angle_to_crate);
      cargo_robot.TurnBody(angle_body + angle_to_crate, 2.0);
      if (abs(angle_to_crate) < 0.0001) {
        state = 3;
        return 0;
      }
    }
    break;
  case 3:
    // 車体を目標方向へ向ける
    if (crate_detectedMarkers.empty()) {
      state = 0;
      search_range = SEARCH_RANGE_INIT;
    } else {
      printf("車両の方向決め\n");
      auto vec_x = crate_detectedMarkers[1].pos_x - crate_detectedMarkers[0].pos_x;
      auto vec_y = crate_detectedMarkers[1].pos_y - crate_detectedMarkers[0].pos_y;
      printf("vec:%f %f\n",vec_x / std::sqrt(vec_x*vec_x + vec_y*vec_y), vec_y / std::sqrt(vec_x*vec_x + vec_y*vec_y) );
      auto target_x = crate_x - 2 * vec_y / std::sqrt(vec_x*vec_x + vec_y*vec_y);
      auto target_y = crate_y + 2 * vec_x / std::sqrt(vec_x*vec_x + vec_y*vec_y);
      double angle_to_target = atan2(target_y, target_x);
      double angle_to_crate = atan2(crate_y, crate_x);
      double angle_body = -cargo_robot.BodyAngle();
      auto turn_angle = angle_to_target - angle_body;
      printf("Angle body: %f\n", angle_body);
      printf("Crate position: (%f, %f), Angle to crate: %f\n", crate_x, crate_y, angle_to_crate);
      printf("Target position: (%f, %f), Angle to target: %f\n", target_x, target_y, angle_to_target);
      printf("Turn angle:%f\n", turn_angle);
      printf("distance %f m\n", distance_to_target(target_x, target_y));
      if (angle_isqual(angle_to_target, angle_body)) {
        state = 4;
      } else {
        cargo_robot.TurnMove(turn_angle, 0);
        cargo_robot.TurnBody(angle_to_crate - angle_body, 2.0);
      }
    }
    break;
  case 4: // 
    if (crate_detectedMarkers.empty()) {
      state = 0;
      search_range = SEARCH_RANGE_INIT;
    } else {
      printf("まっすぐ進んで、マーカーを補足する\n");      
      // クレートの位置から目標座標を取得する。クレートの前方1mの位置を目標とする。
      auto vec_x = crate_detectedMarkers[1].pos_x - crate_detectedMarkers[0].pos_x;
      auto vec_y = crate_detectedMarkers[1].pos_y - crate_detectedMarkers[0].pos_y;
      printf("vec:%f %f\n",vec_x / std::sqrt(vec_x*vec_x + vec_y*vec_y), vec_y / std::sqrt(vec_x*vec_x + vec_y*vec_y) );
      auto target_x = crate_x - 2 * vec_y / std::sqrt(vec_x*vec_x + vec_y*vec_y);
      auto target_y = crate_y + 2 * vec_x / std::sqrt(vec_x*vec_x + vec_y*vec_y);
      double angle_to_target = atan2(target_y, target_x);
      double angle_to_crate = atan2(crate_y, crate_x);
      double angle_body = -cargo_robot.BodyAngle();
      auto turn_angle = angle_to_target - angle_body;
      static auto pre_angle_to_target = angle_to_target;
      static double dir = 1;
      if (pre_angle_to_target * angle_to_target < 0) {
        dir *= -1;
      }
      printf("Angle body: %f\n", angle_body);
      printf("Crate position: (%f, %f), Angle to crate: %f\n", crate_x, crate_y, angle_to_crate);
      printf("Target position: (%f, %f), Angle to target: %f\n", target_x, target_y, angle_to_target);
      printf("Turn angle:%f\n", turn_angle);
      printf("distance %f m\n", distance_to_target(target_x, target_y));

      // 目標位置にいる
      if (angle_isqual(angle_to_target, 0.0, 0.0005)) {
        state = 5;
      }
      // 目標位置ではない
      else {
        double v = 0.5;
        if (distance_to_target(target_x, target_y) < 1) {
          v = 0.05;
        }

        // 目標の方向へ旋回しつつ前進
        cargo_robot.TurnMove(turn_angle/10.0, v);
        //cargo_robot.TurnMove(0.0, dir * v);

        // カメラを荷物へ向ける
        cargo_robot.TurnBody(cargo_robot.BodyAngle()+angle_to_crate, 2.0);
      }
      pre_angle_to_target = angle_to_target;
    }
    break;
  case 5:
    if (crate_detectedMarkers.empty()) {
      state = 0;
      search_range = SEARCH_RANGE_INIT;
    } else {
      printf("車体を荷物に正対させる\n");      
      // クレートの位置から目標座標を取得する。クレートの前方1mの位置を目標とする。
      auto vec_x = crate_detectedMarkers[1].pos_x - crate_detectedMarkers[0].pos_x;
      auto vec_y = crate_detectedMarkers[1].pos_y - crate_detectedMarkers[0].pos_y;
      printf("vec:%f %f\n",vec_x / std::sqrt(vec_x*vec_x + vec_y*vec_y), vec_y / std::sqrt(vec_x*vec_x + vec_y*vec_y) );
      auto target_x = crate_x - 1 * vec_y / std::sqrt(vec_x*vec_x + vec_y*vec_y);
      auto target_y = crate_y + 1 * vec_x / std::sqrt(vec_x*vec_x + vec_y*vec_y);
      double angle_to_target = atan2(target_y, target_x);
      double angle_to_crate = atan2(crate_y, crate_x);
      double angle_body = -cargo_robot.BodyAngle();
      auto turn_angle = angle_to_target - angle_body;
      static auto pre_angle_to_target = angle_to_target;
      static double dir = 1;
      if (pre_angle_to_target * angle_to_target < 0) {
        dir *= -1;
      }
      printf("Angle body: %f\n", angle_body);
      printf("Crate position: (%f, %f), Angle to crate: %f\n", crate_x, crate_y, angle_to_crate);
      printf("Target position: (%f, %f), Angle to target: %f\n", target_x, target_y, angle_to_target);
      printf("Turn angle:%f\n", turn_angle);
      printf("distance %f m\n", distance_to_target(target_x, target_y));

      // 目標位置にいる
      if (angle_isqual(turn_angle, 0.0, 0.05)) {
        // カメラと、ボディ角が一致している。つまり目標が正面にある
        if (angle_isqual(turn_angle, 0.0)) {
          // このステージを終了する
          cargo_robot.StopMove();
          return 1;
        }
        cargo_robot.TurnBody(0);

        // 目標の方法へ旋回
        cargo_robot.TurnMove(turn_angle/10, dir * 0.05);
      }
      // 目標位置ではない
      else {
        // 目標の方法へ旋回
        cargo_robot.TurnMove(turn_angle/20, dir * 0.05);
        // カメラを荷物へ向ける
        cargo_robot.TurnBody(cargo_robot.BodyAngle()+angle_to_crate, 3.0);
      }
      pre_angle_to_target = angle_to_target;
    }  }
  return 0; // Still in progress
}

static int PickupState(Robot *robot) {
  static int state = 0;
  static auto cargo_robot = CargoRobot(robot);
  static auto lift_camera = robot->getCamera("lift camera");
  static auto front_ds = robot->getDistanceSensor("distance sensor");
  static auto lift_motor = robot->getMotor("lift motor");
  static auto connector = robot->getConnector("connector");
  static int timeStep = (int)robot->getBasicTimeStep();

  auto crate_detectedMarkers = get_crate_position(lift_camera);
  auto distance = front_ds->getValue();
  double crate_x = (crate_detectedMarkers[0].pos_x + crate_detectedMarkers[1].pos_x) / 2;
  double crate_y = (crate_detectedMarkers[0].pos_y + crate_detectedMarkers[1].pos_y) / 2;

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
        cargo_robot.TurnMove(0, 0.01);
        connector->enablePresence(timeStep);
        lift_camera->disable();
        state = 3;
      } else if (distance > 500) {
        if (crate_y < 0) {
          cargo_robot.TurnMove(0, 0.05);
        } else {
          cargo_robot.TurnMove(0, 0.05);
        }
      } else {
        if (crate_y < 0) {
          cargo_robot.TurnMove(0.0, 0.5);
        } else {
          cargo_robot.TurnMove(0.0, 0.5);
        }
      }
      break;
    case 3:
      if (connector->getPresence() == 1) {
        printf("Crate grasped. Crate lifting...\n");
        cargo_robot.StopMove();
        for (auto i = 0; i < 5; i++) {
          robot->step(timeStep);
        }
        connector->disablePresence();
        connector->lock();
        // wait for a few steps to ensure the crate is securely grasped before lifting
        while (connector->isLocked() == false) {
          robot->step(timeStep);
        }
        lift_motor->setVelocity(0.05);
        lift_motor->setPosition(lift_motor->getTargetPosition() + 0.5); // Lift the crate up
        // wait for a few steps to ensure the crate is lifted before moving
        for (auto i = 0; i < 5; i++) {
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
      cargo_robot.TurnMove(0.0, -0.1);
      for (auto i = 0; i < 100; i++) {
        robot->step(timeStep);
      }
      state = 5;
      break;
    case 5:
      cargo_robot.StopMove();
      return 1;
      break;
  }
  return 0; // Still in progress
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
  auto body_motor = robot->getMotor("bodyMotor");
  body_motor->setPosition(0.0);

  auto position_sensor = robot->getPositionSensor("body position");
  position_sensor->enable(timeStep);

  DistanceSensor *front_ds = robot->getDistanceSensor("distance sensor"); // Replace with the name of your distance sensor");
  front_ds->enable(timeStep);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    static int state = 0;
    if (state == 0) {
      if (ApproachState(robot) == 1) {
        printf("Approach complete!\n");
        state = 1;
      }
    } else if (state == 1) {
      if (PickupState(robot) == 1) {
        printf("Pickup complete!\n");
        state = 2;
        break;
      }
    } else {
      body_motor->setPosition(body_motor->getTargetPosition() + 0.1);
      right_motor->setVelocity(0);
      left_motor->setVelocity(0);
      auto pos = position_sensor->getValue();
      auto tgt = body_motor->getTargetPosition();
      while (!angle_isqual(pos, tgt)) {
        pos = position_sensor->getValue();
        tgt = body_motor->getTargetPosition();
        robot->step(timeStep);
        printf("Target:%f Position:%f\n",body_motor->getTargetPosition(), position_sensor->getValue());
      }
    }
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  }

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
