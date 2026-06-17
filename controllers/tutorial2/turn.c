#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>

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

static double rightTarget = 0.0;
static double leftTarget = 0.0;

/**
 * @brief Get the offset object
 * 
 * @param offset 	オフセットを -0.5 ～ 0.5 で返す
 * @param pos 		オフセットの取得位置 (FRONT_OFFSET ore REAR_OFFSET)
 * @return true 
 * @return false 
 */
static int get_offset(double *offset, int pos) {
	double m = 0;
    double s1 = 0.0, s2 = 0.0;
	const int width = wb_camera_get_width(bottomCamera);
	const int height = wb_camera_get_height(bottomCamera);
	const unsigned char *image = wb_camera_get_image(bottomCamera);

    for (int x = 0; x < width; x++) {
        double a = (double)(255 - wb_camera_image_get_gray(image, width, x, pos * (height-1)));
        if (a < 256.0/3.0)
            a = 0;
        else if (a > 256.0 / 3.0 * 2.0)
            a = 255.0;
        s1 += a;
        s2 += a * x;
    }
    if (s2 < 0.1 || s1 < 0.1)
        return 0;
    m = s2 / s1;	// 座標
	*offset = m / (double)width - 0.5;
    return 1;
}

/**
 * @brief 旋回
 * 
 * @param direction 車体の向きを0として、反時計回りを正とする角度(radian)
 */
void turn(double direction)
{
    double distance = TREAD/2 * direction / TRACK_TURN_EFFICIENCY;
	rightTarget = wb_position_sensor_get_value(rightPosition) + distance;
	leftTarget = wb_position_sensor_get_value(leftPosition) - distance;
    // 速度はそのまま指定すれば3秒で旋回完了するはず。
    double velocity = fabs(distance) / 3;
	wb_motor_set_position(rightMotor, rightTarget);
	wb_motor_set_position(leftMotor, leftTarget);
	wb_motor_set_velocity(rightMotor, velocity);
	wb_motor_set_velocity(leftMotor, velocity);
}

int turn_isdone(double delta)
{
	if (fabs(rightTarget - wb_position_sensor_get_value(rightPosition)) < delta &&
		fabs(leftTarget - wb_position_sensor_get_value(leftPosition)) < delta) {
		double f_os, r_os;
		wb_motor_set_position(rightMotor, INFINITY);
		wb_motor_set_position(leftMotor, INFINITY);
		if (get_offset(&f_os, FRONT_OFFSET) && get_offset(&r_os, REAR_OFFSET)) {
			if (f_os - r_os < -0.01) {	// ちょっと左向いてる
				wb_motor_set_velocity(rightMotor, -0.05);
				wb_motor_set_velocity(leftMotor, 0.05);
				return 0;
			} else if (f_os - r_os > 0.01) {	// ちょっと右向いてる
				wb_motor_set_velocity(rightMotor, 0.05);
				wb_motor_set_velocity(leftMotor, -0.05);
				return 0;
			}
		}
		wb_motor_set_velocity(rightMotor, 0.0);
		wb_motor_set_velocity(leftMotor, 0.0);				
		return 1;
	}
	return 0;
}