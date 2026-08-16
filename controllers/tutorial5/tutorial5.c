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
#include <webots/compass.h>
#include <webots/lidar.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define ANGLE_EPSILON 0.01

WbDeviceTag rightMotor;
WbDeviceTag leftMotor;
WbDeviceTag rightPosition;
WbDeviceTag leftPosition;
WbDeviceTag bottomCamera;
WbDeviceTag frontCamera;
WbDeviceTag gyro;
WbDeviceTag compass;
WbDeviceTag distanceSensor;
WbDeviceTag liftMotor;
WbDeviceTag liftPosition;
WbDeviceTag liftCamera;
WbDeviceTag connector;
WbDeviceTag compass;
WbDeviceTag lidar;

typedef enum TURN {
	TRN_LEFT=-1,
	TRN_RIGHT=1,
} TURN;

typedef enum DIRECTION {
	DIR_NORTH,
	DIR_EAST,
	DIR_SOUTH,
	DIR_WEST,
} DIRECTION;

typedef enum COMMAND {
	CMD_POSITION,
	CMD_MOVETO,
	CMD_TURN,
	CMD_PICKUP,
	CMD_PUTOFF,
} COMMAND;

typedef struct Command {
	COMMAND cmd;
	int args[3];
} Command;

typedef struct CargoStatus {
	DIRECTION direction;
	int pos_x;
	int pos_y;
	Command *command;	
} CargoStatus;

CargoStatus cargo_status;

Command command_list[] = {
	{CMD_POSITION,	{3,	3,	DIR_EAST}},
	{CMD_MOVETO,	{4,	3,	0}},
    {CMD_PICKUP,	{0,	0,	0}},
	{CMD_TURN,		{TRN_LEFT, 0, 0}},
	{CMD_TURN,		{TRN_LEFT, 0, 0}},
	{CMD_MOVETO,	{3,	3,	0}},
	{CMD_TURN, 		{TRN_RIGHT,	0, 0}},
	{CMD_PUTOFF,	{0,	0,	0}},
};

double platform_level;

// Crate Marker Info
typedef struct {
	const WbCameraRecognitionObject *left;
	const WbCameraRecognitionObject *right;
} CreateMarkerInfo;

double get_bearing_in_radian(WbDeviceTag tag)
{
  const double *north = wb_compass_get_values(tag);
  double bearing = atan2(north[1], north[0]);
  if (bearing < 0.0)
    bearing = bearing + 2.0 * M_PI;
  return bearing;
}

double direction_to_radian(TURN dir)
{
	double d = (double)dir;
	d *= (M_PI / 2.0);
	return d;
}

int turn_state()
{
	static int state = 0;
	static double target_dir;
	double current_dir = get_bearing_in_radian(compass);
	printf("current_dir %lf\n", current_dir);
	switch (state) {
	case 0:
		target_dir = current_dir + direction_to_radian(cargo_status.command->args[0]);
		printf("target_dir %lf\n", target_dir);
		wb_motor_set_velocity(leftMotor, 0.0);
		wb_motor_set_velocity(rightMotor, 0.0);
		state = 1;
		break;
	case 1:
	{
		double diff_dir = target_dir - current_dir;
		if (diff_dir > M_PI) {
			diff_dir -= 2.0 * M_PI;
		} else if (diff_dir < -1.0 * M_PI) {
			diff_dir += 2.0 * M_PI;
		}
		printf("diff_dir %lf\n", diff_dir);
		if (fabs(diff_dir) <= ANGLE_EPSILON) {
			wb_motor_set_velocity(leftMotor, 0.0);
			wb_motor_set_velocity(rightMotor, 0.0);
			state = 2;
		} else if (diff_dir < 0) {
			wb_motor_set_velocity(leftMotor, -0.1);
			wb_motor_set_velocity(rightMotor, 0.1);
		} else {
			wb_motor_set_velocity(leftMotor, 0.1);
			wb_motor_set_velocity(rightMotor, -0.1);
		}
		break;
	}
	case 2:
		state = 0;
		cargo_status.direction += cargo_status.command->args[0];
		return 1;
	}
	return 0;
}

const WbCameraRecognitionObject *find_front_marker()
{
	int num = wb_camera_recognition_get_number_of_objects(frontCamera);
	const WbCameraRecognitionObject *objs = wb_camera_recognition_get_objects(frontCamera);
	for (int i = 0; i < num; i++) {
		int x, y;
		printf("front: objs[%d].model:%s\n", i, objs[i].model);
		if (sscanf(objs[i].model, "#%1d%1d", &y, &x) == 2) {

			printf("front: pos%d,%d:tgt%d,%d\n", cargo_status.pos_x, cargo_status.pos_y, x, y);

			switch (cargo_status.direction) {
			case DIR_NORTH:
			printf("front: NORTH\n");
				if (cargo_status.pos_y < y && cargo_status.pos_x == x) {
					return &objs[i];
				}
				break;
			case DIR_EAST:
			printf("front: EAST\n");
				if (cargo_status.pos_x < x && cargo_status.pos_y == y) {
					return &objs[i];
				}
				break;
			case DIR_SOUTH:
			printf("front: SOUTH\n");
				if (cargo_status.pos_y > y && cargo_status.pos_x == x) {
					return &objs[i];
				}
				break;
			case DIR_WEST:
			printf("front: WEST\n");
				if (cargo_status.pos_x > x && cargo_status.pos_y == y) {
					return &objs[i];
				}
				break;
			}
		}
	}
	return NULL;
}

const WbCameraRecognitionObject *find_bottom_marker(int tgt_x, int tgt_y)
{
	int num = wb_camera_recognition_get_number_of_objects(bottomCamera);
	const WbCameraRecognitionObject *objs = wb_camera_recognition_get_objects(bottomCamera);
	for (int i = 0; i < num; i++) {
		int x, y;
		printf("bottom: objs[%d].model:%s\n", i, objs[i].model);
		if (sscanf(objs[i].model, "#%1d%1d", &y, &x) == 2) {
			if (tgt_x == x && tgt_y == y) {
				return &objs[i];
			}
		}
	}
	return NULL;
}

int moveto_state()
{
	static int state = 0;
	static int target_x = 10;
	static int target_y = 10;

	const WbCameraRecognitionObject *tgt = find_front_marker();
	const WbCameraRecognitionObject *btm = find_bottom_marker(target_x, target_y);
	if (tgt) {
		printf("tgt:%s\n", tgt->model);
	}
	if (btm) {
		printf("btm:%s\n", btm->model);
	}

	switch (state) {
	case 0:
		target_x = cargo_status.command->args[0];
		target_y = cargo_status.command->args[1];
		state = 1;
		printf("moveto_state:state = 1: %d %d\n", target_x, target_y);
		break;
	case 1:
	{
		if (tgt) {
			state = 2;
		} else {
			state = 3;
		}
	}
	case 2:	// 前方のマーカーへまっすぐ前進
	{
		printf("moveto_state:state = 2\n");
		if (btm) {
			if (btm->position_on_image[1] >= wb_camera_get_height(bottomCamera) / 2) {
				wb_motor_set_velocity(leftMotor, 0.0);
				wb_motor_set_velocity(rightMotor, 0.0);
				state = 4;
			} else {
				wb_motor_set_velocity(leftMotor, 0.5);
				wb_motor_set_velocity(rightMotor, 0.5);
			}
			break;
		}
		if (!tgt) {	// 見失った
			state = 1;
			wb_motor_set_velocity(leftMotor, 0.50);
			wb_motor_set_velocity(rightMotor, 0.50);
			break;
		}
		// 捕捉中
		if (tgt->position_on_image[0] > wb_camera_get_width(frontCamera) / 2) {
			wb_motor_set_velocity(leftMotor, 0.6);
			wb_motor_set_velocity(rightMotor, 0.40);
		} else if (tgt->position_on_image[0] < wb_camera_get_width(frontCamera) / 2) {
			wb_motor_set_velocity(leftMotor, 0.40);
			wb_motor_set_velocity(rightMotor, 0.6);
		} else {
			wb_motor_set_velocity(leftMotor, 0.50);
			wb_motor_set_velocity(rightMotor, 0.50);
		}
		break;
	}
	case 3:	// 前方へ移動しながらポジションチェック
	{
		printf("moveto_state:state = 3\n");
		if (tgt) {
			state = 2;
			break;
		}
		if (!btm) {
			wb_motor_set_velocity(leftMotor, 0.50);
			wb_motor_set_velocity(rightMotor, 0.50);
		} else {
			if (btm->position_on_image[1] >= wb_camera_get_height(bottomCamera) / 2) {
				wb_motor_set_velocity(leftMotor, 0.0);
				wb_motor_set_velocity(rightMotor, 0.0);
				state = 4;
			} else {
				wb_motor_set_velocity(leftMotor, 0.5);
				wb_motor_set_velocity(rightMotor, 0.5);
			}
		}
		break;
	}
	case 4:
		printf("moveto_state:state = 4\n");
		state = 0;
		cargo_status.pos_x = cargo_status.command->args[0];
		cargo_status.pos_y = cargo_status.command->args[1];
		return 1;
	}
	return 0;
}

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

int pickup_state()
{
	const WbCameraRecognitionObject *btm = find_bottom_marker(cargo_status.pos_x, cargo_status.pos_y);
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
				wb_motor_set_velocity(liftMotor, 0.1);
			} else if (height/2 - (y1+y2)/2 < 0) {
				wb_motor_set_velocity(liftMotor, -0.1);
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
				wb_motor_set_velocity(leftMotor, 0.45);
			} else if (width/2 - (x1+x2)/2 < 0) {
				wb_motor_set_velocity(rightMotor, 0.45);
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
			wb_motor_set_velocity(rightMotor, -0.5);
			wb_motor_set_velocity(leftMotor, -0.5);
			state = 6;
		}
		break;
	case 6:
		// 後退
		if (btm) {
			if (btm->position_on_image[1] <= wb_camera_get_height(bottomCamera) / 2) {
				wb_motor_set_velocity(leftMotor, 0.0);
				wb_motor_set_velocity(rightMotor, 0.0);
				state = 0;
				return 1;
			}
		}
	}
	return 0;
}

int putoff_state()
{
	const WbCameraRecognitionObject *btm = find_bottom_marker(cargo_status.pos_x, cargo_status.pos_y);
	static int state = 0;
	int h_res = wb_lidar_get_horizontal_resolution(lidar);
	int v_res = wb_lidar_get_number_of_layers(lidar);
	switch (state) {
	case 0:
		// 駆動設定
		wb_motor_set_position(rightMotor, INFINITY);
		wb_motor_set_position(leftMotor, INFINITY);
		state = 1;
		break;
	case 1:
		// 距離計測
		wb_motor_set_velocity(rightMotor, 0.5);
		wb_motor_set_velocity(leftMotor, 0.5);
		const float *image = wb_lidar_get_layer_range_image(lidar, v_res/2);
		if (image[h_res/2] < 0.25) {
			wb_motor_set_velocity(rightMotor, 0.0);
			wb_motor_set_velocity(leftMotor, 0.0);
			state = 2;
		}
		break;
	case 2:
		// rift down
		wb_motor_set_position(liftMotor, 0.3);
		wb_motor_set_velocity(liftMotor, -0.2);
		state = 3;
		break;
	case 3:
		{
			double pos = wb_position_sensor_get_value(liftPosition);
			if (pos < 0.31) {
				wb_motor_set_position(liftMotor, 0);
				wb_motor_set_velocity(rightMotor, -0.5);
				wb_motor_set_velocity(leftMotor, -0.5);
				wb_connector_unlock(connector);
				state = 4;
			}
			break;
		}
	case 4:
		// 後退
		if (btm) {
			if (btm->position_on_image[1] <= wb_camera_get_height(bottomCamera) / 2) {
				wb_motor_set_velocity(leftMotor, 0.0);
				wb_motor_set_velocity(rightMotor, 0.0);
				state = 0;
				return 1;
			}
		}
	}
	return 0;
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) 
{
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
	wb_motor_set_position(rightMotor, INFINITY);
	wb_motor_set_position(leftMotor, INFINITY);
	wb_motor_set_velocity(leftMotor, 0.0);
	wb_motor_set_velocity(rightMotor, 0.0);
	/* カメラデバイスを取得 */
	bottomCamera = wb_robot_get_device("bottom camera");
	wb_camera_enable(bottomCamera, time_step);
	wb_camera_recognition_enable(bottomCamera, time_step);

	// 前方カメラ
	frontCamera = wb_robot_get_device("front camera");
	wb_camera_enable(frontCamera, time_step);
	wb_camera_recognition_enable(frontCamera, time_step);

	compass = wb_robot_get_device("compass");
	wb_compass_enable(compass, time_step);

	// 距離センサ
	distanceSensor = wb_robot_get_device("distance sensor");
	wb_distance_sensor_enable(distanceSensor, time_step);
	// リフトモーター
	liftMotor = wb_robot_get_device("lift motor");
	liftPosition = wb_robot_get_device("liftMotor position");
	wb_position_sensor_enable(liftPosition, time_step);
	// リフトカメラ
	liftCamera = wb_robot_get_device("lift camera");
	wb_camera_enable(liftCamera, time_step);
	wb_camera_recognition_enable(liftCamera, time_step);
	// コネクタ
	connector = wb_robot_get_device("connector");

	lidar = wb_robot_get_device("front lidar");
	wb_lidar_enable(lidar, time_step);

	// ここで一度ステップを踏む。これがないと、position_sensorが計測してない。
	if (wb_robot_step(time_step) == -1)	return 0;

	for (int command_idx = 0; command_idx < sizeof command_list / sizeof command_list[0]; command_idx++) {
		cargo_status.command = &command_list[command_idx];
		switch (cargo_status.command->cmd) {
		case CMD_POSITION:
			printf("CMD_POSITION\n");
			break;
		case CMD_MOVETO:
			printf("CMD_MOVETO\n");
			break;
		case CMD_TURN:
			printf("CMD_TURN\n");
			break;
		case CMD_PICKUP:
			printf("CMD_PICKUP\n");
			break;
		case CMD_PUTOFF:
			printf("CMD_PUTOFF\n");
			break;					
		}
		while (wb_robot_step(time_step) != -1) {
			switch (cargo_status.command->cmd) {
			case CMD_POSITION:
				cargo_status.pos_x = cargo_status.command->args[0];
				cargo_status.pos_y = cargo_status.command->args[1];
				cargo_status.direction = cargo_status.command->args[2];
				goto NEXT;
			case CMD_MOVETO:
				if (moveto_state()) {
					goto NEXT;
				}
				break;
			case CMD_TURN:
				if (turn_state()) {
					goto NEXT;
				}
				break;
			case CMD_PICKUP:
				if (pickup_state()) {
					goto NEXT;
				}
				break;
			case CMD_PUTOFF:
				if (putoff_state()) {
					goto NEXT;
				}
				break;
			}
		}
	NEXT:
	}

	/* This is necessary to cleanup webots resources */
	wb_robot_cleanup();

	return 0;
}
