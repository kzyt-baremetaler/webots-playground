/*
 * File:          container.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/accelerometer.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32
#define THRESHOLD_G 4.0  // 4G以上の衝撃で壊れる設定

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  double max_g = 0.0;
  wb_robot_init();

  // 加速度センサのセットアップ
  WbDeviceTag accel = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accel, TIME_STEP);

  bool is_broken = false;
  for (int i = 0; i < 60; i++) wb_robot_step(TIME_STEP);

  while (wb_robot_step(TIME_STEP) != -1) {
    if (!is_broken) {
      // センサ値を取得 (m/s^2)
      const double *values = wb_accelerometer_get_values(accel);
      
      // 合成加速度を計算
      double accel_mag = sqrt(values[0]*values[0] + values[1]*values[1] + values[2]*values[2]);
      
      // 重力加速度(9.81)で割り、G単位に変換
      double g_force = accel_mag / 9.81;
      if (max_g < g_force) max_g = g_force;
      printf("G %f\n",max_g);
      if (max_g > THRESHOLD_G) {
        printf("【警告】荷物が破損しました！ 衝撃: %f G\n", max_g);
        is_broken = true;
        
        // ここに破損時の挙動を追加
        // 例: 色を変える、音を出す、シミュレーションを止めるなど
        wb_robot_set_custom_data("broken");
      }
    }
  }

  wb_robot_cleanup();
  return 0;
}
