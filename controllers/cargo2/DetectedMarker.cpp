#include "DetectedMarker.hpp"
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
        results.push_back(m);
    }
    return results;
}