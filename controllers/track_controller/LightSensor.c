/*
 * File:          LightSensor.c
 * Date:
 * Description:     Webotsには光の反射が実装されていないので、通常の光センサでライントレースはできない。
 *                  光センサを模擬的にカメラデバイスで行う。カメラの解像度が 1x1dotになっており、入力情報が光センサと変わらない。
 *                  ライントレース用の8連光センサのように、カメラが左から 0..7の8台並べられている。
 * Author:          tani@tfusion.co.jp
 * Modifications:
 */
#include "config.h"
#include <webots/camera.h>
#include <webots/robot.h>
#include <stdio.h>

static WbDeviceTag light_sensors[8];
static int width;
static int height;

void LightSensor_init()
{
    light_sensors[0] = wb_robot_get_device("LightSensor_0");
    light_sensors[1] = wb_robot_get_device("LightSensor_1");
    light_sensors[2] = wb_robot_get_device("LightSensor_2");
    light_sensors[3] = wb_robot_get_device("LightSensor_3");
    light_sensors[4] = wb_robot_get_device("LightSensor_4");
    light_sensors[5] = wb_robot_get_device("LightSensor_5");
    light_sensors[6] = wb_robot_get_device("LightSensor_6");
    light_sensors[7] = wb_robot_get_device("LightSensor_7");
   
    for (int i = 0; i < 8; i++)
        wb_camera_enable(light_sensors[i], TIME_STEP);
    width = wb_camera_get_width(light_sensors[0]);
    height = wb_camera_get_height(light_sensors[0]);
}

/**
 * Reads the average intensity value from the specified light sensor.
 * 
 * This function retrieves the RGB values from the camera device configured
 * as a light sensor at the given index and calculates the average intensity
 * by averaging the red, green, and blue components.
 *
 * @param index The index of the light sensor to read.
 * @return The average intensity value as a character.
 */
unsigned char LightSensor_read(int index)
{
    const unsigned char *image = wb_camera_get_image(light_sensors[index]);
    unsigned int r = wb_camera_image_get_red(image, width, width/2, height/2);
    unsigned int g = wb_camera_image_get_green(image, width, width/2, height/2);
    unsigned int b = wb_camera_image_get_blue(image, width, width/2, height/2);
    return (r+g+b)/3;
}