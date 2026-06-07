#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

extern WbDeviceTag rightMotor;
extern WbDeviceTag leftMotor;
extern WbDeviceTag rightPosition;
extern WbDeviceTag leftPosition;

static double rightTarget = 0.0;
static double leftTarget = 0.0;
/**
 * @brief 直進移動
 * 
 * @param distance      移動距離(m)
 * @param velocity		移動速度(rad/sec)
 * 
 * 移動速度は ホイール半径 * 移動速度 で (m/sec)へ変換できる。
 */
void move_straight(double distance, double velocity)
{
	// ホイールの半径から指定距離を算出 
	rightTarget = wb_position_sensor_get_value(rightPosition) + distance;
	leftTarget = wb_position_sensor_get_value(leftPosition) + distance;
	wb_motor_set_position(rightMotor, rightTarget);
	wb_motor_set_position(leftMotor, leftTarget);
	wb_motor_set_velocity(rightMotor, velocity);
	wb_motor_set_velocity(leftMotor, velocity);
}

/**
 * @brief 移動完了判定
 * 
 * @param delta 誤差
 * @return true 移動完了
 * @return false 
 */
int move_straight_isdone(double delta)
{
	if (fabs(rightTarget - wb_position_sensor_get_value(rightPosition)) < delta &&
		fabs(leftTarget - wb_position_sensor_get_value(leftPosition)) < delta) {
		return 1;
	}
	return 0;
}

