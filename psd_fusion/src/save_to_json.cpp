#include "save_to_json.h"

void SaveFileToJson::SaveapaSlotListInfoToJson(apaSlotListInfo &info, const std::string &filename){
    const int cornernum = 4;
    json j;
    
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

    // 读取已有的JSON文件
    json j_existing;
    std::ifstream inFile(filename);
    if (inFile.is_open()){
        try{
            inFile >> j_existing;
        }catch (json::parse_error& e){
            std::cerr<<"JSON 解析错误:" << e.what()<<std::endl;
            j_existing = json::array();
        }
        inFile.close();
    }else{
        j_existing = json::array();
    }

    //如果不是数组，可能是第一次保存，需要将j_existing 转换为数组
    if(!j_existing.is_array()){
        json temp = j_existing;
        j_existing = json::array();
        j_existing.push_back(temp);
    }

    //将新的数据对象追加到数组中
    j_existing.push_back(j);

    //将更新后的数据写回文件
    std::ofstream outfile(filename);
    if (outfile.is_open()){
        outfile << j_existing.dump(4);
        outfile.close();
    }else{
        std::cerr<<"无法打开文件进行写入："<<filename<<std::endl;
    }
}

void SaveFileToJson::SaveQuadParkingSlotsInfoToJson(rd::QuadParkingSlots &info,const std::string &filename){
    // 构建新的数据对象 j_new
    json j_new;

    // 添加 QuadParkingSlots 的顶层字段
    j_new["frameTimeStampNs"] = info.frameTimeStampNs;
    j_new["sensorId"] = info.sensorId;

    // 提取 header 信息（假设 rd::Header 有需要的字段）
    j_new["header"]["seq"] = info.header.seq;
    j_new["header"]["frameId"] = info.header.frameId;
    // 如果有其他需要的字段，继续添加

    // 提取 quadParkingSlotList 中的每个 QuadParkingSlot
    json slotListArray = json::array();

    for (const auto& slot : info.quadParkingSlotList) {
        json slotJson;

        // 提取四个顶点坐标（tl、tr、bl、br）
        slotJson["tl"] = { {"x", slot.tl.x}, {"y", slot.tl.y} };
        slotJson["tr"] = { {"x", slot.tr.x}, {"y", slot.tr.y} };
        slotJson["bl"] = { {"x", slot.bl.x}, {"y", slot.bl.y} };
        slotJson["br"] = { {"x", slot.br.x}, {"y", slot.br.y} };

        // 提取其他字段
        slotJson["confidence"] = slot.confidence;
        slotJson["label"] = slot.label;
        slotJson["filtered"] = slot.filtered;
        slotJson["slotType"] = slot.slotType;
        slotJson["sTl"] = slot.sTl;
        slotJson["sTr"] = slot.sTr;
        slotJson["sBl"] = slot.sBl;
        slotJson["sBr"] = slot.sBr;

        slotJson["dirIn"] = { {"x", slot.dirIn.x}, {"y", slot.dirIn.y} };
        slotJson["dirWidth"] = { {"x", slot.dirWidth.x}, {"y", slot.dirWidth.y} };
        slotJson["dirLength"] = { {"x", slot.dirLength.x}, {"y", slot.dirLength.y} };
        slotJson["center"] = { {"x", slot.center.x}, {"y", slot.center.y} };

        slotJson["oppModify"] = slot.oppModify;
        slotJson["isComplete"] = slot.isComplete;
        slotJson["width"] = slot.width;
        slotJson["length"] = slot.length;
        slotJson["isVisited"] = slot.isVisited;

        // 如果需要提取 pTl、pTr、pBl、pBr，可以继续添加

        // 将 slotJson 添加到数组中
        slotListArray.push_back(slotJson);
    }

    // 将 slotListArray 添加到主 JSON 对象中
    j_new["quadParkingSlotList"] = slotListArray;

    // 读取已有的 JSON 文件
    json j_existing;
    std::ifstream inFile(filename);
    if (inFile.is_open()) {
        try {
            inFile >> j_existing;
        } catch (json::parse_error& e) {
            std::cerr << "JSON 解析错误：" << e.what() << std::endl;
            j_existing = json::array(); // 如果解析失败，初始化为空数组
        }
        inFile.close();
    } else {
        // 文件不存在，初始化为空数组
        j_existing = json::array();
    }

    // 如果不是数组，可能是第一次保存，需要将 j_existing 转换为数组
    if (!j_existing.is_array()) {
        json temp = j_existing;
        j_existing = json::array();
        j_existing.push_back(temp);
    }

    // 将新的数据对象追加到数组中
    j_existing.push_back(j_new);

    // 将更新后的数据写回文件
    std::ofstream outFile(filename);
    if (outFile.is_open()){
        outFile << j_existing.dump(4); // 缩进4个空格，格式化输出
        outFile.close();
    } else {
        std::cerr << "无法打开文件进行写入：" << filename << std::endl;
    }
}