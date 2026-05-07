#include <math.h>
#include "DetectedMarker.hpp"

std::vector<std::vector<double>> Rodrigues(const std::vector<double> &rotation) {
    auto x = rotation[0];
    auto y = rotation[1];
    auto z = rotation[2];
    auto w = rotation[3];
    double size = sqrt(x*x + y*y + z*z);
    double c = cos(w);
    double s = sin(w);
    if (size < 0.001) { // If s is close to zero, then the direction of the axis is not important
        return {{0, 0, 0}, {0, 0, 0},{0, 0, 0}}   ; // Return zero rotation
    } else {
        return {
            {x*x*(1-c)+c, x*y*(1-c)+z*s, z*x*(1-c)+y*s}, 
            {x*y*(1-c)-z*s, y*y*(1-c)+c, y*z*(1-c)-x*s},
            {z*x*(1-c)-y*s, z*y*(1-c)+x*s, z*z*(1-c)+c}
        }; // Return the rotation vector
    }
}
std::vector<DetectedMarker> get_visible_markers(Camera *camera) 
{
    std::vector<DetectedMarker> results;

    const CameraRecognitionObject *objects = camera->getRecognitionObjects();
    int count = camera->getRecognitionNumberOfObjects();
    
    for (int i = 0; i < count; i++) {
        DetectedMarker m;
        // descriptionフィールドに "id:101" とか入れておけば、それをパース
        m.id = std::stoi(std::string(objects[i].model).substr(1)); // "#"の3文字をスキップして数値部分だけを取得
        m.x = objects[i].position_on_image[0];
        m.y = objects[i].position_on_image[1];
        m.pos_x = objects[i].position[0] - objects[i].size[0]/2; // 3D空間での位置も必要ならここから取得可能
        m.pos_y = objects[i].position[1];
        m.pos_z = objects[i].position[2];
        m.rotation = Rodrigues({objects[i].orientation[0], objects[i].orientation[1], objects[i].orientation[2], objects[i].orientation[3]}); // 4Dクォータニオンからロドリゲス回転ベクトルへの変換（必要に応じて実装）
        results.push_back(m);
    }
    return results;
}