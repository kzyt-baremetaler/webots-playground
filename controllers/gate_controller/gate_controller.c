/*
 * File:          gate_controller.c
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
#include <webots/display.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>
#include <stdio.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 32

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  // ディスプレイデバイスの取得
  WbDeviceTag display = wb_robot_get_device("display");
  WbDeviceTag sensor = wb_robot_get_device("distance sensor");
  
  wb_distance_sensor_enable(sensor, 32);

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    double distance = wb_distance_sensor_get_value(sensor);
    if (distance < 800) {
      break;
    }
  }
  double start_t = wb_robot_get_time();
  while (wb_robot_step(TIME_STEP) != -1) {
    char buf[80];
    double t = wb_robot_get_time();
    sprintf(buf, "%2.3f",t - start_t);
    // ディスプレイを初期化 (背景を白に設定)
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_fill_rectangle(display, 0, 0, 800, 600);  // 白色 (R:255, G:255, B:255)
    // 文字を描画
    wb_display_set_color(display, 0);
    // フォントを設定 (Arial、サイズ20、アンチエイリアス有効)
    wb_display_set_font(display, "Arial", 20, true);
    wb_display_draw_text(display, buf, 10, 10);  // 黒色 (R:0, G:0, B:0)
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    double distance = wb_distance_sensor_get_value(sensor);
    /* Process sensor data here */
    if (distance < 800 && t - start_t > 5.0) {
      // 5秒間待ち
      while (wb_robot_step(TIME_STEP) != -1) {
        if (wb_robot_get_time() > t -start_t + 3.0)
          break;
      }
      break;
    }
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
     
  };
  wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();
  return 0;
}
