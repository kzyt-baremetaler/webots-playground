#include <math.h>
#include "CargoRobot.hpp"

CargoRobot::CargoRobot(Robot *r)
: robot(r)
{
    rightMotor = robot->getMotor("rightMotor");
    leftMotor = robot->getMotor("leftMotor");
    liftMotor = robot->getMotor("lift motor");
    bodyMotor = robot->getMotor("bodyMotor");
    bodyPosition = robot->getPositionSensor("body position");
    frontCamera = robot->getCamera("front camera");
    bottomCamera = robot->getCamera("bottom camera");
    rearCamera = robot->getCamera("rear camera");
}

void CargoRobot::TurnMove(double turn_rate, double verocity)
{
    rightMotor->setVelocity(turn_rate + verocity);
    leftMotor->setVelocity(-turn_rate + verocity);
    printf("TrunMove(%f, %f)\n",turn_rate, verocity);
    printf("move r:%f l:%f\n", rightMotor->getVelocity(), leftMotor->getVelocity());
}

void CargoRobot::StopMove()
{
    rightMotor->setVelocity(0.0);
    leftMotor->setVelocity(0.0);
}

/**
 * @brief UpperBodyを旋回する
 * 
 * 旋回速度は目標値との差で決定するので、speedで倍率を調整する
 * 
 * @param angle radianで指定。反時計回りが正
 * @param speed 旋回速度
 * 
 * @return double 目標値と現在位置の差
 */
double CargoRobot::TurnBody(double angle, double speed)
{
    auto pos = bodyPosition->getValue();
    auto rate = abs(angle - pos);
    if (rate < 0.1*speed) rate = 0.1*speed;
    printf("TurnBody(%f, %f) v:%f t:%f p:%f d:%f\n", angle, speed, rate, angle, pos, angle-pos);
    bodyMotor->setVelocity(rate);
    bodyMotor->setPosition(angle);
    return angle - pos;
}
