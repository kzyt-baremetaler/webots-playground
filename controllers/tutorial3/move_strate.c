#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>

extern WbDeviceTag rightMotor;
extern WbDeviceTag leftMotor;
extern WbDeviceTag rightPosition;
extern WbDeviceTag leftPosition;
extern WbDeviceTag bottomCamera;

static double rightTarget = 0.0;
static double leftTarget = 0.0;

static int direction(double *d) {
	double m = 0;
    double s1 = 0.0, s2 = 0.0;
	const int width = wb_camera_get_width(bottomCamera);
	const unsigned char *image = wb_camera_get_image(bottomCamera);

    for (int x = 0; x < width; x++) {
        double a = (double)(255 - wb_camera_image_get_gray(image, width, x, 0));
        if (a < 256.0/3.0)
            a = 0;
        else if (a > 256.0 / 3.0 * 2.0)
            a = 255.0;
        s1 += a;
        s2 += a * x;
    }
    if (s2 < 0.1)
        return 0;
    m = s2 / s1;	// 座標
	*d = m / (double)width - 0.5;
    return 1;
}

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
	wb_motor_set_position(rightMotor, INFINITY);
	wb_motor_set_position(leftMotor, INFINITY);
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
	double dir;
	if (fabs(rightTarget - wb_position_sensor_get_value(rightPosition) +
			 leftTarget - wb_position_sensor_get_value(leftPosition))/2 < delta) {
		wb_motor_set_velocity(rightMotor, 0);
		wb_motor_set_velocity(leftMotor, 0);
		return 1;
	}

	double right = wb_motor_get_velocity(rightMotor);
	double left = wb_motor_get_velocity(leftMotor);
	double velocity = (right + left)/2.0;
	// ここで方向を補正する
	if (direction(&dir)) {
		if (dir > 0.0) { // 左にそれてる
			wb_motor_set_velocity(rightMotor, velocity - 0.02);
			wb_motor_set_velocity(leftMotor, velocity + 0.02);
		} else {
			wb_motor_set_velocity(rightMotor, velocity + 0.02);
			wb_motor_set_velocity(leftMotor, velocity - 0.02);
		}
	} else {
		wb_motor_set_velocity(rightMotor, velocity);
		wb_motor_set_velocity(leftMotor, velocity);
	}
	return 0;
}

