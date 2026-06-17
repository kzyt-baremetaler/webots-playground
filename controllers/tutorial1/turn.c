#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#define WHEEL_RADIUS    0.1
#define TREAD    0.8

extern WbDeviceTag rightMotor;
extern WbDeviceTag leftMotor;
extern WbDeviceTag rightPosition;
extern WbDeviceTag leftPosition;

static double rightTarget = 0.0;
static double leftTarget = 0.0;

/**
 * @brief 旋回
 * 
 * @param direction 車体の向きを0として、反時計回りを正とする角度(radian)
 */
void turn(double direction)
{
    double distance = TREAD/2 * direction;
	rightTarget = wb_position_sensor_get_value(rightPosition) + distance;
	leftTarget = wb_position_sensor_get_value(leftPosition) - distance;
    // 速度はそのまま指定すれば1秒で旋回完了するはず。
    double velocity = fabs(distance);
	wb_motor_set_position(rightMotor, rightTarget);
	wb_motor_set_position(leftMotor, leftTarget);
	wb_motor_set_velocity(rightMotor, velocity);
	wb_motor_set_velocity(leftMotor, velocity);
}

int turn_isdone(double delta)
{
	if (fabs(rightTarget - wb_position_sensor_get_value(rightPosition)) < delta &&
		fabs(leftTarget - wb_position_sensor_get_value(leftPosition)) < delta) {
		return 1;
	}
	return 0;
}