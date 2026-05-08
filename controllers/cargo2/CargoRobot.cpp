#include <math.h>
#include <assert.h>
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
    liftCamera = robot->getCamera("lift camera");
}

void CargoRobot::TurnMove(double turn_rate, double verocity)
{
    rightMotor->setPosition(INFINITY);
    leftMotor->setPosition(INFINITY);
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

static bool moment_x(double *m, Camera *c, int x) {
    assert(m != NULL);
    assert(c != NULL);
    double s1 = 0.0, s2 = 0.0;
    auto image = c->getImage();
    auto width = c->getWidth();
    auto height = c->getHeight();
    for (double y = 0.0; y < height; y++) {
        auto a = (double)(255 - c->imageGetGray(image, width, x, y));
        if (a < 256.0/3.0)
            a = 0;
        else if (a > 256.0 / 3.0 * 2.0)
            a = 255.0;
        s1 += a;
        s2 += a * y;
    }
    if (s2 < 0.1)
        return 0;
    *m = s2 / s1;
    return true;
}

/**
 * @brief 水平線を検出する
 *
 * ボトムカメラを使用して水平線を検出し、その情報をHorizontalLine構造体に格納する。
 * 平均輝度と傾斜角を計算し、結果をoutパラメータに設定する。
 *
 * @param camera 使用するカメラ
 * @param out    検出された水平線の情報（offset, angle）が格納される構造体
 * @return true  水平線を検出した場合
 * @return false 水平線を検出できなかった場合
 */
bool CargoRobot::DetectHorizontalLine(Camera *camera, HorizontalLine *out)
{
    double m_R, m_L;
    auto L = moment_x(&m_L, camera, 0);
    auto R = moment_x(&m_R, camera, camera->getHeight() - 1);
    if (L && R) {
        printf("L:%f R:%f\n", m_L, m_R);
        out->offset = (m_L + m_R) / camera->getHeight() - 1;
        out->angle = atan2(m_R - m_L, camera->getWidth());
        return true;
    } else if (L) {
        double m_M;
        if (moment_x(&m_M, camera, camera->getHeight()/2)) {
            printf("L:%f M:%f\n", m_L, m_M);
            out->offset = 2.0 * m_M / camera->getHeight() - 1.0;
            out->angle = atan2(m_M - m_L, camera->getWidth()/2);
            return true;
        }
    } else if (R) {
        double m_M;
        if (moment_x(&m_M, camera, camera->getHeight()/2)) {
            printf("R:%f M:%f\n", m_R, m_M);
            out->offset = 2.0 * m_M / camera->getHeight() - 1.0;
            out->angle = atan2(m_R - m_M, camera->getWidth()/2);
            return true;
        }
    }
    return false;
}

bool CargoRobot::DetectHorizontalLineFromBottom(HorizontalLine *out)
{
    return DetectHorizontalLine(bottomCamera, out);
}

/**
 * @brief 輝度のy方向のモーメントを計算する
 *
 * カメラ画像の指定したx列における輝度の平均と重心位置（y方向）を計算する。
 *
 * @param m     重心位置（y方向）
 * @param c     使用するカメラ
 * @param y     輝度を計算する行位置
 * @return true 成功の場合
 * @return false 失敗の場合
 */
static bool moment_y(double *m, Camera *c, int y)
{
    assert(m != NULL);
    assert(c != NULL);
    double s1 = 0.0, s2 = 0.0;
    auto image = c->getImage();
    auto width = c->getWidth();

    for (auto x = 0; x < width; x++) {
        auto a = (double)(255 - c->imageGetGray(image, width, x, y));
        if (a < 256.0/3.0)
            a = 0;
        else if (a > 256 / 3 * 2)
            a = 255;
        s1 += a;
        s2 += a * x;
    }
    if (s2 < 0.1)
        return false;
    *m = s2 / s1;
    return true;
}

/**
 * @brief 垂直線を検出する
 *
 * ボトムカメラを使用して垂直線を検出し、その情報をVerticalLine構造体に格納する。
 * 平均輝度と傾斜角を計算し、結果をoutパラメータに設定する。
 *
 * @param camera 使用するカメラ
 * @param out    検出された垂直線の情報（offset, angle）が格納される構造体
 * @return true  垂直線を検出した場合
 * @return false 垂直線を検出できなかった場合
 */
bool CargoRobot::DetectVerticalLine(Camera *camera, VerticalLine *out)
{
    double m_T, m_B;
    auto T = moment_y(&m_T, camera, 0);
    auto B = moment_y(&m_B, camera, camera->getHeight() - 1);
    if (T && B) {
        printf("T:%f B:%f\n", m_T, m_B);
        out->offset = (m_T + m_B) / camera->getWidth() - 1;
        out->angle = atan2(m_B - m_T, camera->getHeight());
        return true;
    } else if (T) {
        double m_M;
        if (moment_y(&m_M, camera, camera->getHeight()/2)) {
            printf("T:%f M:%f\n", m_T, m_M);
            out->offset = 2.0 * m_M / camera->getWidth() - 1.0;
            out->angle = atan2(m_M - m_T, camera->getHeight()/2);
            return true;
        }
    } else if (B) {
        double m_M;
        if (moment_y(&m_M, camera, camera->getHeight()/2)) {
            printf("B:%f M:%f\n", m_B, m_M);
            out->offset = 2.0 * m_M / camera->getWidth() - 1.0;
            out->angle = atan2(m_B - m_M, camera->getHeight()/2);
            return true;
        }
    }
    return false;
}

bool CargoRobot::DetectVerticalLineFromBottom(VerticalLine *out)
{
    return DetectVerticalLine(bottomCamera, out);
}

/**
 * @brief 指定距離を移動する
 * 
 * スリップによる誤差に注意
 * 
 * @param distance 
 */
void CargoRobot::PositionMove(double distance, double speed)
{
    leftMotor->setVelocity(speed);
    rightMotor->setVelocity(speed);
    leftMotor->setPosition(distance/0.1);
    rightMotor->setPosition(distance/0.1);
}
