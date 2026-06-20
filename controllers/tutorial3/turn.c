#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>
#include <webots/gyro.h>

#define TRACK_TURN_EFFICIENCY 0.5 // スキッドロスによる補正係数（実測値）

#define WHEEL_RADIUS    0.1
#define TREAD    0.8

#define FRONT_OFFSET	0
#define REAR_OFFSET		1


extern WbDeviceTag rightMotor;
extern WbDeviceTag leftMotor;
extern WbDeviceTag rightPosition;
extern WbDeviceTag leftPosition;
extern WbDeviceTag bottomCamera;
extern WbDeviceTag gyro;

static double angle = 0.0;
static double target_angle = 0.0;

/**
 * @brief 旋回
 * 
 * @param direction 車体の向きを0として、反時計回りを正とする角度(radian)
 */
void turn(double direction)
{
	angle = 0.0;
	target_angle = direction;
	printf("angle %f direction %f\n", angle, direction);
    double distance = TREAD/2 * direction / TRACK_TURN_EFFICIENCY;
    // 速度はそのまま指定すれば3秒で旋回完了するはず。
    double velocity = distance / 3;
	wb_motor_set_velocity(rightMotor, 0);
	wb_motor_set_velocity(leftMotor, 0);
	wb_motor_set_position(rightMotor, INFINITY);
	wb_motor_set_position(leftMotor, INFINITY);
	wb_motor_set_velocity(rightMotor, velocity);
	wb_motor_set_velocity(leftMotor, -velocity);
}

int turn_isdone(double delta)
{
	const int time_step = wb_robot_get_basic_time_step();
	const double *gyro_value = wb_gyro_get_values(gyro);
	angle += gyro_value[2] * time_step / 1000;
	if (fabs(angle - target_angle) < delta) {
		wb_motor_set_velocity(rightMotor, 0.0);
		wb_motor_set_velocity(leftMotor, 0.0);				
		return 1;
	}
	return 0;
}