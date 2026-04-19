/*
 * File:          track_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include "config.h"
#include "LightSensor.h"
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/camera.h>
#include <webots/gps.h>
#include <string.h>
#include <stdio.h>
 
/*
* You may want to add macros here.
*/
#define MAX_SPEED 6.28

bool line_proc(unsigned char *data, double *course) {
  double a = 0;
  double b = 0;

  for (int i = 0; i < 8; i++) {
    a += data[i]*i;
    b += data[i];
  }
  if (b == 0) {
    // 多分クランク
    // 直前のコースのまま最大旋回率を要求
    *course = *course < 0 ? -1.0 : 1.0;
    return true;
  }
  *course = -(a / b - 4.0) / 4.0;
  return true;
}
 
 /**
 * @brief Set the velocities of the left and right motors based on course information.
 * @details The function adjusts the speed of the motors for turning and straight movement.
 * @param left Left motor device tag
 * @param right Right motor device tag
 * @param course Course information
 */
void set_drive(WbDeviceTag left, WbDeviceTag right, double course) {
  double base_speed = 0.095 * MAX_SPEED;
  double turn_speed = -0.02 * MAX_SPEED;
  double left_speed = base_speed;
  double right_speed = base_speed;

  if (course > 0.15) {
    left_speed = turn_speed;
  } else if (course < -0.15) {
    right_speed = turn_speed;
  }

  wb_motor_set_velocity(left, left_speed);
  wb_motor_set_velocity(right, right_speed);
}
 
 
/*
* This is the main program.
* The arguments of the main function can be specified by the
* "controllerArgs" field of the Robot node
*/
int main(int argc, char **argv) {
  unsigned char data[8];
  double course = 0.0;
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
  * You should declare here WbDeviceTag variables for storing
  * robot devices like this:
  *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
  *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
  */
  LightSensor_init();   // ライン読み取り用光センサ初期化
  WbDeviceTag lMotor = wb_robot_get_device("LeftMotor");
  WbDeviceTag rMotor = wb_robot_get_device("RightMotor");
  wb_motor_set_position(lMotor, INFINITY);  // モータ回転上限を無効化
  wb_motor_set_position(rMotor, INFINITY);  // モータ回転上限を無効化

  /* main loop
  * Perform simulation steps of TIME_STEP milliseconds
  * and leave the loop when the simulation is over
  */
  while (wb_robot_step(TIME_STEP) != -1) {
    // 光センサからライン情報を取得
    // 光強度は256段階の数値で 白は255 黒は0
    for (int i = 0; i < 8; i++) {
      // ライン情報を反転 黒を255 白を0に
      data[i] = 255 - LightSensor_read(i);
    }
    // ライン情報からコースを取得
    bool ret = line_proc(data, &course);
    // コースからモータ回転速度を入力
    if (ret) {
      set_drive(lMotor, rMotor, course);
    } else {
    //wb_motor_set_velocity(lMotor, 0);
    //wb_motor_set_velocity(rMotor, 0);      
    }
  }

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}
 