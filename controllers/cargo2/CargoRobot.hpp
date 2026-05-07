#ifndef CARGO_ROBOT_HPP
#define CARGO_ROBOT_HPP
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

using namespace webots;

class CargoRobot {
public:
    CargoRobot(Robot *r);
    void TurnMove(double turn_rate, double verocity);
    void StopMove();
    double TurnBody(double angle, double speed = 1.0);
    inline double BodyAngle() const { return bodyPosition->getValue(); }
private:
    Robot *robot;
    Motor *rightMotor;
    Motor *leftMotor;
    Motor *liftMotor;
    Motor *bodyMotor;
    PositionSensor *bodyPosition;
    Camera *frontCamera;
    Camera *bottomCamera;
    Camera *rearCamera;
};

#endif