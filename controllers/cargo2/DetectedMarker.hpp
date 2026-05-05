#ifndef DETECTED_MARKER_HPP
#define DETECTED_MARKER_HPP

#include <vector>
#include <webots/Camera.hpp>
using namespace webots;

struct DetectedMarker {
    int id;
    int x; // 画像上の中心座標X
    int y; // 画像上の中心座標Y
};

std::vector<DetectedMarker> get_visible_markers(Camera *camera);
#endif