/*
 * File:          tutorial1.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

WbDeviceTag rightMotor;
WbDeviceTag leftMotor;
WbDeviceTag rightPosition;
WbDeviceTag leftPosition;
WbDeviceTag bottomCamera;

extern void move_straight(double distance, double velocity);
extern int move_straight_isdone(double delta);
extern void turn(double direction);
extern int turn_isdone(double delta);

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
	/* necessary to initialize webots stuff */
	wb_robot_init();
	const int time_step = wb_robot_get_basic_time_step();
	/* 今回使わないけど */
	WbDeviceTag bodyMotor = wb_robot_get_device("bodyMotor");
	wb_motor_set_position(bodyMotor, 0);
	wb_motor_set_velocity(bodyMotor, 1);
	/* 駆動系のデバイスを取得 */
	rightMotor = wb_robot_get_device("rightMotor");
	leftMotor = wb_robot_get_device("leftMotor");
	rightPosition = wb_robot_get_device("rightMotor position");
	leftPosition = wb_robot_get_device("leftMotor position");
	wb_position_sensor_enable(rightPosition, time_step);
	wb_position_sensor_enable(leftPosition, time_step);
	/* カメラデバイスを取得 */
	bottomCamera = wb_robot_get_device("bottom camera");
	wb_camera_enable(bottomCamera, wb_robot_get_basic_time_step());


	// ここで一度ステップを踏む。これがないと、position_sensorが計測してない。
	if (wb_robot_step(time_step) == -1)	return 0;

	/* 座標 (3,3) から 座標 (6,3) へ移動開始
	* 1grid 2mなので、6m移動。
	*/
	move_straight(6.0,  // 移動距離
				0.5); // 移動速度 = 0.05m/sec

	/* 目標まで移動を続ける */
	while (!move_straight_isdone(0.1)) {
		if (wb_robot_step(time_step) == -1)	return 0;
	}

	/* 方向転換 北へ
	 * つまり、反時計回りに 0.5π radian
	 */
	turn(M_PI/2);
	while (!turn_isdone(0.01)) {
		if (wb_robot_step(time_step) == -1)	return 0;
	}
	/* 座標 (6,3) から 座標 (6,6) へ移動開始 */
	move_straight(6.0, 0.5);
	while (!move_straight_isdone(0.1)) {
		if (wb_robot_step(time_step) == -1)	return 0;
	}

	/*
	 * 方向転換 西へ
	 * つまり、反時計回りに 0.5π radian
	 */	
	turn(M_PI/2);
	while (!turn_isdone(0.01)) {
		if (wb_robot_step(time_step) == -1)	return 0;
	}
	/* 座標 (6,6) から 座標 (3,6) へ移動開始 */
	move_straight(6.0, 0.5);
	while (!move_straight_isdone(0.1)) {
		if (wb_robot_step(time_step) == -1)	return 0;
	}

	/*
	 * 方向転換 南へ
	 * つまり、反時計回りに 0.5π radian
	 */	
	turn(M_PI/2);
	while (!turn_isdone(0.01)) {
		if (wb_robot_step(time_step) == -1)	return 0;
	}
	/* 座標 (3,6) から 座標 (3,3) へ移動開始 */
	move_straight(6.0, 0.5);
	while (!move_straight_isdone(0.1)) {
		if (wb_robot_step(time_step) == -1)	return 0;
	}
	/* Enter your cleanup code here */

	/* This is necessary to cleanup webots resources */
	wb_robot_cleanup();

	return 0;
}
