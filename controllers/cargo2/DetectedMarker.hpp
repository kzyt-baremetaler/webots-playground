#ifndef DETECTED_MARKER_HPP
#define DETECTED_MARKER_HPP

#include <vector>
#include <webots/Camera.hpp>
using namespace webots;

struct DetectedMarker {
    int id;
    int x; // 画像上の中心座標X
    int y; // 画像上の中心座標Y
    double pos_x; // 3D空間での位置X
    double pos_y; // 3D空間での位置Y
    double pos_z; // 3D空間での位置Z
    std::vector<std::vector<double>> rotation; // 3D空間での回転（ロドリゲス回転ベクトルなどで表現）
};

std::vector<DetectedMarker> get_visible_markers(Camera *camera);
#endif