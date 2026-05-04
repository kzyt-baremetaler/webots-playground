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
#define THRESHOLD_G 5.0  // 5G以上の衝撃で壊れる設定

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  wb_robot_init();

  // 加速度センサのセットアップ
  WbDeviceTag accel = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accel, TIME_STEP);

  bool is_broken = false;

  while (wb_robot_step(TIME_STEP) != -1) {
    if (!is_broken) {
      // センサ値を取得 (m/s^2)
      const double *values = wb_accelerometer_get_values(accel);
      
      // 合成加速度を計算
      double accel_mag = sqrt(values[0]*values[0] + values[1]*values[1] + values[2]*values[2]);
      
      // 重力加速度(9.81)で割り、G単位に変換
      double g_force = accel_mag / 9.81;

      if (g_force > THRESHOLD_G) {
        printf("【警告】荷物が破損しました！ 衝撃: %f G\n", g_force);
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
