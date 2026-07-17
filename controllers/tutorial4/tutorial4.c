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
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/camera.h>
#include <webots/gyro.h>
#include <webots/compass.h>
#include <webots/distance_sensor.h>
#include <webots/connector.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

WbDeviceTag rightMotor;
WbDeviceTag leftMotor;
WbDeviceTag rightPosition;
WbDeviceTag leftPosition;
WbDeviceTag bottomCamera;
WbDeviceTag gyro;
WbDeviceTag compass;
WbDeviceTag distanceSensor;
WbDeviceTag liftMotor;
WbDeviceTag liftPosition;
WbDeviceTag liftCamera;
WbDeviceTag connector;

double platform_level;

// Crate Marker Info
typedef struct {
	const WbCameraRecognitionObject *left;
	const WbCameraRecognitionObject *right;
} CreateMarkerInfo;

const CreateMarkerInfo *find_crate(const WbCameraRecognitionObject *obj, const int num)
{
	static CreateMarkerInfo info = {NULL, NULL};
	info.left = NULL;
	info.right = NULL;
	printf("num = %d\n", num);
	for (int i = 0; i < num; i++) {
		if (strcmp(obj[i].model, "#101") == 0) {
			info.left = &obj[i];
			printf("left\n");
		} else if (strcmp(obj[i].model, "#100") == 0) {
			info.right = &obj[i];
			printf("right\n");
		}
	}
	if (info.left != NULL && info.right != NULL)
		return &info;
	else
		return NULL;
}

int state_machine()
{
	const double time_step = wb_robot_get_basic_time_step();
	static int state = 0;
	switch (state) {
	case 0:	// start
		// 駆動設定
		wb_motor_set_position(rightMotor, INFINITY);
		wb_motor_set_position(leftMotor, INFINITY);
		wb_motor_set_velocity(rightMotor, 0.5);
		wb_motor_set_velocity(leftMotor, 0.5);
		// カメラ設定
		wb_camera_enable(liftCamera, time_step);
		wb_camera_recognition_enable(liftCamera, time_step);
		state = 1;
		break;
	case 1:
	{
		const int num = wb_camera_recognition_get_number_of_objects(liftCamera);
		const WbCameraRecognitionObject *obj = wb_camera_recognition_get_objects(liftCamera);
		// 荷物のマーカー２つを捉える。
		const CreateMarkerInfo *crates = find_crate(obj, num);
		if (crates) {
			// 荷物発見
			wb_motor_set_velocity(rightMotor, 0);
			wb_motor_set_velocity(leftMotor, 0);
			state = 2;
		} else {
			// 荷物が遠いので、直進
			wb_motor_set_velocity(rightMotor, 0.5);
			wb_motor_set_velocity(leftMotor, 0.5);
		}
		break;
	}
	case 2:	// Lift Up
	{
		wb_motor_set_position(liftMotor, INFINITY);
		wb_motor_set_velocity(liftMotor, 0.05);
		state = 3;
		break;
	}
	case 3:
	{
		const int num = wb_camera_recognition_get_number_of_objects(liftCamera);
		const WbCameraRecognitionObject *obj = wb_camera_recognition_get_objects(liftCamera);
		// 荷物のマーカー２つを捉える。
		const CreateMarkerInfo *crates = find_crate(obj, num);
		if (crates) {
			// マーカーのY座標がカメラの中央に来るまで
			int y1 = crates->left->position_on_image[1];
			int y2 = crates->right->position_on_image[1];
			int height = wb_camera_get_height(liftCamera);
			printf("height %d y1 %d\n", height, y1);
			if (height/2 - (y1+y2)/2 > 0) {
				wb_motor_set_velocity(liftMotor, 0.05);
			} else if (height/2 - (y1+y2)/2 < 0) {
				wb_motor_set_velocity(liftMotor, -0.05);
			} else {
				// リフト止めて
				wb_motor_set_velocity(liftMotor, 0);
				// 前進開始
				wb_motor_set_velocity(rightMotor, 0.5);
				wb_motor_set_velocity(leftMotor, 0.5);
				state = 4;
			}
		}
		break;
	}
	case 4:	// 荷物に近づく
	{
		const int num = wb_camera_recognition_get_number_of_objects(liftCamera);
		const WbCameraRecognitionObject *obj = wb_camera_recognition_get_objects(liftCamera);
		// 荷物のマーカー２つを捉える。
		const CreateMarkerInfo *crates = find_crate(obj, num);
		if (crates) {
			// マーカーのX座標がカメラの中央に来るまで
			int x1 = crates->left->position_on_image[0];
			int x2 = crates->right->position_on_image[0];
			int width = wb_camera_get_width(liftCamera);
			if (width/2 - (x1+x2)/2 == 0) {
				wb_motor_set_velocity(rightMotor, 0.5);
				wb_motor_set_velocity(leftMotor, 0.5);
			} else if (width/2 - (x1+x2)/2 > 0) {
				wb_motor_set_velocity(rightMotor, 0.55);
				wb_motor_set_velocity(leftMotor, 0.5);
			} else if (width/2 - (x1+x2)/2 < 0) {
				wb_motor_set_velocity(rightMotor, 0.5);
				wb_motor_set_velocity(leftMotor, 0.55);
			}
		} else {
			// 荷物が近くて、マーカーがカメラに入らないので直進
			wb_motor_set_velocity(rightMotor, 0.5);
			wb_motor_set_velocity(leftMotor, 0.5);
		}
		// 距離チェック
		double dist = wb_distance_sensor_get_value(distanceSensor);
		if (dist > 300) {
			// 距離が近いので、減速
			wb_motor_set_velocity(rightMotor, 0.1);
			wb_motor_set_velocity(leftMotor, 0.1);
			// コネクタを有効にする
			wb_connector_enable_presence(connector, time_step);
			wb_camera_disable(liftCamera);
			state = 5;
		}
		break;
	}	
	case 5:
		// 捕まえるまで前進
		if (wb_connector_get_presence(connector) == 1) {
			// 捕まえたので、ロックする
			wb_connector_lock(connector);
			wb_connector_disable_presence(connector);
			// 停止
			wb_motor_set_velocity(leftMotor, 0.0);
			wb_motor_set_velocity(rightMotor, 0.0);
			// 持ち上げる。
			platform_level = wb_position_sensor_get_value(liftPosition);
			wb_motor_set_position(liftMotor, platform_level + 0.5);
			wb_motor_set_velocity(liftMotor, 0.1);
			printf("max_pos:%f, l:%f\n", wb_motor_get_max_position(liftMotor), platform_level + 0.5);
			state = 6;
		}
		break;
	case 6:
		// 終了
		wb_motor_set_velocity(rightMotor, -0.1);
		wb_motor_set_velocity(leftMotor, -0.1);
		return 1;
	}
	return 0;
}




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
	wb_camera_enable(bottomCamera, time_step);

	// 距離センサ
	distanceSensor = wb_robot_get_device("distance sensor");
	wb_distance_sensor_enable(distanceSensor, time_step);
	// リフトモーター
	liftMotor = wb_robot_get_device("lift motor");
	liftPosition = wb_robot_get_device("liftMotor position");
	wb_position_sensor_enable(liftPosition, time_step);
	// リフトカメラ
	liftCamera = wb_robot_get_device("lift camera");
	// コネクタ
	connector = wb_robot_get_device("connector");

	// ここで一度ステップを踏む。これがないと、position_sensorが計測してない。
	if (wb_robot_step(time_step) == -1)	return 0;

	while (wb_robot_step(time_step) != -1) {
		if (state_machine()) {
			break;
		}
	}

	/* This is necessary to cleanup webots resources */
	wb_robot_cleanup();

	return 0;
}
