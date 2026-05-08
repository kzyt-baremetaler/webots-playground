#ifndef CARGO_ROBOT_HPP
#define CARGO_ROBOT_HPP
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Camera.hpp>

using namespace webots;

struct HorizontalLine {
    double offset;  // 画面中央より上が負、下が正 -1.0 ～ +1.0
    double angle;   // 右回転が正
};
struct VerticalLine {
    double offset;
    double angle;
};

class CargoRobot {
public:
    CargoRobot(Robot *r);
    void TurnMove(double turn_rate, double verocity);
    void PositionMove(double distance, double verocity);
    void StopMove();
    double TurnBody(double angle, double speed = 1.0);
    inline double BodyAngle() const { return bodyPosition->getValue(); }
    bool DetectHorizontalLineFromBottom(HorizontalLine *out);
    bool DetectVerticalLineFromBottom(VerticalLine *out);
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
    Camera *liftCamera;
    bool DetectHorizontalLine(Camera *camera, HorizontalLine *out);
    bool DetectVerticalLine(Camera *camera, VerticalLine *out);
};

#endif