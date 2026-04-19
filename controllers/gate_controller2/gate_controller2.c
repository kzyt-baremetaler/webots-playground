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
#include <string.h>
/*
 * You may want to add macros here.
 */
#define TIME_STEP 16


void display_time(const char *name1, double time1, const char *name2, double time2)
{
	WbDeviceTag display = wb_robot_get_device("display");

	char buf[80];
    sprintf(buf, "%s %2.3f %s %2.3f",name1, time1, name2, time2);
	// ディスプレイを初期化 (背景を白に設定)
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_fill_rectangle(display, 0, 0, 200, 20);  // 白色 (R:255, G:255, B:255)
    // 文字を描画
    wb_display_set_color(display, 0);
    // フォントを設定 (Arial、サイズ20、アンチエイリアス有効)
    wb_display_set_font(display, "Arial", 10, true);
	wb_display_draw_text(display, buf, 1, 1);  // 黒色 (R:0, G:0, B:0)
}

WbFieldRef get_field_by_name(WbNodeRef parent, const char *name) {
	int num = wb_supervisor_node_get_number_of_fields(parent);
	for (int i = 0; i < num; ++i) {
		WbFieldRef field = wb_supervisor_node_get_field_by_index(parent, i);
		const char *field_name = wb_supervisor_field_get_name(field);
		if (strcmp(field_name, name) == 0) {
			return field;
		}
	}
	return NULL;
}

	/* ワールド座標へ変換 
	  Wp = Lp * R + T
	  Wp : ワールド座標
	  Lp : ローカル座標
	  R : 回転行列
	  T : 移動行列
	*/
const double *local_to_world(const double *Lp, const double *R, const double *T, double *Wp) {
	double A[3];
	A[0] = Lp[0] * R[0] + Lp[1] * R[1] + Lp[2] * R[2] + T[0];
	A[1] = Lp[0] * R[3] + Lp[1] * R[4] + Lp[2] * R[5] + T[1];
	A[2] = Lp[0] * R[6] + Lp[1] * R[7] + Lp[2] * R[8] + T[2];
	memcpy(Wp, A, sizeof A);
	return Wp;
}

/*
 * 二次元座標での内積
 */
double cross(const double *a, const double *b, const double *c) {
	return (a[0] - c[0])*(b[1] - c[1]) - (a[1] - c[1])*(b[0] - c[0]);
}
double distance(const double *a, const double *b) {
	return (a[0] - b[0])*(a[0] - b[0]) + (a[1] - b[1])*(a[1] - b[1]);
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
	int max_count = 2;
	/* necessary to initialize webots stuff */
	wb_robot_init();


	WbDeviceTag display = wb_robot_get_device("display");
    // ディスプレイを初期化 (背景を白に設定)
    wb_display_set_color(display, 0xFFFFFF);
    wb_display_fill_rectangle(display, 0, 0, 200, 20);  // 白色 (R:255, G:255, B:255)
    // 文字を描画
    wb_display_set_color(display, 0);
    // フォントを設定 (Arial、サイズ20、アンチエイリアス有効)
    wb_display_set_font(display, "Arial", 20, true);

	/* トラック名を取得 */
 	WbNodeRef track[2];
	track[0] = wb_supervisor_node_get_from_def("BLUE");
	track[1] = wb_supervisor_node_get_from_def("RED");
	

	/* ゲートの位置を取得 */
	WbNodeRef gate = wb_supervisor_node_get_from_def("GATE");
	const double *gate_pos = wb_supervisor_node_get_position(gate);
	const double *gate_R =wb_supervisor_node_get_orientation(gate);

	/* ゲートのピラー作業を取得 */
	WbFieldRef pillar_r = get_field_by_name(gate, "pillar_r_pos");
	double pillar_r_pos[3] = {0, 0, 0};
	memcpy(pillar_r_pos, wb_supervisor_field_get_sf_vec3f(pillar_r), sizeof pillar_r_pos); //double *pillar_r_pos = wb_supervisor_field_get_sf_vec3f(pillar_r);
	WbFieldRef pillar_l = get_field_by_name(gate, "pillar_l_pos");
	double pillar_l_pos[3]	= {0, 0, 0};
	memcpy(pillar_l_pos, wb_supervisor_field_get_sf_vec3f(pillar_l), sizeof pillar_l_pos); //double *pillar_l_pos = wb_supervisor_field_get_sf_vec3f(pillar_l);
	printf("pillar_r %f %f %f\n", pillar_r_pos[0], pillar_r_pos[1], pillar_r_pos[2]);
	printf("pillar_l %f %f %f\n", pillar_l_pos[0], pillar_l_pos[1], pillar_l_pos[2]);

	/* ワールド座標へ変換 */
	local_to_world(pillar_r_pos, gate_R, gate_pos, pillar_r_pos);
	local_to_world(pillar_l_pos, gate_R, gate_pos, pillar_l_pos);

	printf("pillar_r %f %f %f\n", pillar_r_pos[0], pillar_r_pos[1], pillar_r_pos[2]);
	printf("pillar_l %f %f %f\n", pillar_l_pos[0], pillar_l_pos[1], pillar_l_pos[2]);


	const double *pos_b = wb_supervisor_node_get_position(track[0]);
	const double *pos_r = wb_supervisor_node_get_position(track[1]);
	double pre_b_cross = cross(pillar_l_pos, pos_b, pillar_r_pos);
	double pre_r_cross = cross(pillar_l_pos, pos_r, pillar_r_pos);

	int count[2] = {0, 0};
	double start_t[2];
	/* main loop
	* Perform simulation steps of TIME_STEP milliseconds
	* and leave the loop when the simulation is over
	*/
	while (wb_robot_step(TIME_STEP) != -1) {
		const double *pos_b = wb_supervisor_node_get_position(track[0]);
		const double *pos_r = wb_supervisor_node_get_position(track[1]);
		double b_cross = cross(pillar_l_pos, pos_b, pillar_r_pos);
		double r_cross = cross(pillar_l_pos, pos_r, pillar_r_pos);
		if (b_cross * pre_b_cross < 0 && distance(pos_b, pillar_l_pos) < distance(pillar_l_pos, pillar_r_pos)) {
			// 超えた
			count[0]++;
			if (count[0] == 1) {
				start_t[0] = wb_robot_get_time();
			} else if (count[0] > max_count) {
				break;
			}
		}
		if (r_cross * pre_r_cross < 0 && distance(pos_r, pillar_l_pos) < distance(pillar_l_pos, pillar_r_pos)) {
			count[1]++;
			if (count[1] == 1) {
				start_t[1] = wb_robot_get_time();
			} else if (count[1] > max_count) {
				break;
			}
		}
		pre_b_cross = b_cross;
		pre_r_cross = r_cross;
		if (count[0] > 0 && count[1] > 0) {
			display_time("BLUE", wb_robot_get_time() - start_t[0], "RED", wb_robot_get_time() - start_t[1]);			
		}
	}
	display_time("BLUE", wb_robot_get_time() - start_t[0], "RED", wb_robot_get_time() - start_t[1]);			

	wb_supervisor_simulation_set_mode(WB_SUPERVISOR_SIMULATION_MODE_PAUSE);

	/* Enter your cleanup code here */

	/* This is necessary to cleanup webots resources */
	wb_robot_cleanup();
	return 0;
}
