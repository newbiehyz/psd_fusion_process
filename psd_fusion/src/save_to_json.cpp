#include "save_to_json.h"

void SaveFileToJson::SaveapaSlotListInfoToJson(apaSlotListInfo &info, const std::string &filename, json& j){
    const int cornernum = 4;
    
    j["AbsoluteDeltaX"] = info.AbsoluteDeltaX;
    j["AbsoluteDeltaY"] = info.AbsoluteDeltaY;
    j["AbsoluteDeltaAngle"] = info.AbsoluteDeltaAngle;
    j["UartCount"] = info.UartCount;
    j["ullFrameId"] = info.ullFrameId;
    j["ulAlgStartTime"] = info.ulAlgStartTime;
    j["ulAlgEndTime"] = info.ulAlgEndTime;

    // 提取 WorldoutRect 中的必要信息
    json worldOutRectArray = json::array();

    for (const auto& slot : info.WorldoutRect) {
        json slotJson;
        slotJson["is_reliable"] = slot.is_reliable;
        slotJson["detect_frame_count"] = slot.detect_frame_count;

        // 提取 rectInfo 中的信息
        slotJson["rectInfo"]["label"] = slot.rectInfo.label;
        slotJson["rectInfo"]["PStype"] = slot.rectInfo.PStype;

        // 提取坐标点信息
        json pointsArray = json::array();
        for (int i = 0; i < cornernum; ++i) {
            json pointJson;
            pointJson["x"] = slot.rectInfo.pt[i].x;
            pointJson["y"] = slot.rectInfo.pt[i].y;
            pointsArray.push_back(pointJson);
        }
        slotJson["rectInfo"]["points"] = pointsArray;

        // 将 slotJson 添加到数组中
        worldOutRectArray.push_back(slotJson);
    }

    // 将数组添加到主 JSON 对象中
    j["WorldoutRect"] = worldOutRectArray;

    std::ofstream file(filename);
    if (file.is_open()){
        file << j.dump(4); //缩进4个空格，格式化输出
    }else{
        std::cerr<<"无法打开文件进行写入："<<filename<<std::endl;
    }
    
}