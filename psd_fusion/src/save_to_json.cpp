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

void SaveFileToJson::SaveDRInfoToJson(Loc::App2emap_DR &info,const std::string &filename){
    // 构建新的数据对象 j_new
    json j_dr;

    j_dr["x"] = info.x;
    j_dr["y"] = info.y;
    j_dr["canAng"] = info.canAng;
    j_dr["DRStatus"] = info.DRStatus;
    j_dr["timeStamp"] = info.timeStamp;

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
    j_existing.push_back(j_dr);

    // 将更新后的数据写回文件
    std::ofstream outFile(filename);
    if (outFile.is_open()){
        outFile << j_existing.dump(4); // 缩进4个空格，格式化输出
        outFile.close();
    } else {
        std::cerr << "无法打开文件进行写入：" << filename << std::endl;
    }
}


void SaveFileToJson::SaveObsToJson(Fus::PkEmapObs &info,const std::string &filename){
    json j_obs;

    json pkEmapObsArray = json::array();
    for (const auto & obs_one : info.pkEmapObs){
        json single_obs_json;

        single_obs_json["FrameIndex"] = obs_one.FrameIndex;
        single_obs_json["FailSafe"] = obs_one.FailSafe;
        single_obs_json["obsID"] = obs_one.obsID;
        single_obs_json["obsTyp"] = obs_one.obsTyp;
        single_obs_json["age"] = obs_one.age;
        single_obs_json["obsBdTyp"] = obs_one.obsBdTyp;
        single_obs_json["obsConf"] = obs_one.obsConf;

        single_obs_json["obsCenter"]["x"] = obs_one.obsCenter.x;
        single_obs_json["obsCenter"]["y"] = obs_one.obsCenter.y;
        single_obs_json["obsCenter"]["z"] = obs_one.obsCenter.z;

        single_obs_json["obsDirection"]["x"] = obs_one.obsDirection.x;
        single_obs_json["obsDirection"]["y"] = obs_one.obsDirection.y;
        single_obs_json["obsDirection"]["z"] = obs_one.obsDirection.z;

        single_obs_json["obsMotionInfo"]["acceleration"]["x"] = obs_one.obsMotionInfo.acceleration.x;
        single_obs_json["obsMotionInfo"]["acceleration"]["y"] = obs_one.obsMotionInfo.acceleration.y;
        single_obs_json["obsMotionInfo"]["acceleration"]["z"] = obs_one.obsMotionInfo.acceleration.z;
        single_obs_json["obsMotionInfo"]["accelerationUncertainty"]["x"] = obs_one.obsMotionInfo.accelerationUncertainty.x;
        single_obs_json["obsMotionInfo"]["accelerationUncertainty"]["y"] = obs_one.obsMotionInfo.accelerationUncertainty.y;
        single_obs_json["obsMotionInfo"]["accelerationUncertainty"]["z"] = obs_one.obsMotionInfo.accelerationUncertainty.z;
        single_obs_json["obsMotionInfo"]["center"]["x"] = obs_one.obsMotionInfo.center.x;
        single_obs_json["obsMotionInfo"]["center"]["y"] = obs_one.obsMotionInfo.center.y;
        single_obs_json["obsMotionInfo"]["center"]["z"] = obs_one.obsMotionInfo.center.z;
        single_obs_json["obsMotionInfo"]["centerUncertainty"]["x"] = obs_one.obsMotionInfo.centerUncertainty.x;
        single_obs_json["obsMotionInfo"]["centerUncertainty"]["y"] = obs_one.obsMotionInfo.centerUncertainty.y;
        single_obs_json["obsMotionInfo"]["centerUncertainty"]["z"] = obs_one.obsMotionInfo.centerUncertainty.z;
        single_obs_json["obsMotionInfo"]["isValid"] = obs_one.obsMotionInfo.isValid;
        single_obs_json["obsMotionInfo"]["jerk"]["x"] = obs_one.obsMotionInfo.jerk.x;
        single_obs_json["obsMotionInfo"]["jerk"]["y"] = obs_one.obsMotionInfo.jerk.y;
        single_obs_json["obsMotionInfo"]["jerk"]["z"] = obs_one.obsMotionInfo.jerk.z;
        single_obs_json["obsMotionInfo"]["jerkUncertainty"]["x"] = obs_one.obsMotionInfo.jerkUncertainty.x;
        single_obs_json["obsMotionInfo"]["jerkUncertainty"]["y"] = obs_one.obsMotionInfo.jerkUncertainty.y;
        single_obs_json["obsMotionInfo"]["jerkUncertainty"]["z"] = obs_one.obsMotionInfo.jerkUncertainty.z;
        single_obs_json["obsMotionInfo"]["velocity"]["x"] = obs_one.obsMotionInfo.velocity.x;
        single_obs_json["obsMotionInfo"]["velocity"]["y"] = obs_one.obsMotionInfo.velocity.y;
        single_obs_json["obsMotionInfo"]["velocity"]["z"] = obs_one.obsMotionInfo.velocity.z;
        single_obs_json["obsMotionInfo"]["velocityHeading"] = obs_one.obsMotionInfo.velocityHeading;
        single_obs_json["obsMotionInfo"]["velocityHeadingRate"] = obs_one.obsMotionInfo.velocityHeadingRate;
        single_obs_json["obsMotionInfo"]["velocityHeadingRateUncertainty"] = obs_one.obsMotionInfo.velocityHeadingRateUncertainty;
        single_obs_json["obsMotionInfo"]["velocityHeadingUncertainty"] = obs_one.obsMotionInfo.velocityHeadingUncertainty;
        single_obs_json["obsMotionInfo"]["velocityUncertainty"]["y"] = obs_one.obsMotionInfo.velocityUncertainty.x;
        single_obs_json["obsMotionInfo"]["velocityUncertainty"]["y"] = obs_one.obsMotionInfo.velocityUncertainty.y;
        single_obs_json["obsMotionInfo"]["velocityUncertainty"]["z"] = obs_one.obsMotionInfo.velocityUncertainty.z;

        single_obs_json["obsTrajectory"]["confidence"] = obs_one.obsTrajectory.confidence;
        single_obs_json["obsTrajectory"]["motionStatus"] = obs_one.obsTrajectory.motionStatus;
        json obsTrajectoryArray = json::array();
        for (const auto & obsTrajectorypoint : obs_one.obsTrajectory.points){
            json single_trajectory_point;
            single_trajectory_point["deltaTNs"] = obsTrajectorypoint.deltaTNs;

            single_trajectory_point["center"]["x"] = obsTrajectorypoint.center.x;
            single_trajectory_point["center"]["y"] = obsTrajectorypoint.center.y;
            single_trajectory_point["center"]["z"] = obsTrajectorypoint.center.z;

            single_trajectory_point["direction"]["x"] = obsTrajectorypoint.direction.x;
            single_trajectory_point["direction"]["y"] = obsTrajectorypoint.direction.y;
            single_trajectory_point["direction"]["z"] = obsTrajectorypoint.direction.z;

            obsTrajectoryArray.push_back(single_trajectory_point);
        }
        single_obs_json["obsTrajectory"]["points"] = obsTrajectoryArray;


        json obsBdBoxArray = json::array();
        for (const auto & box : obs_one.obsBdBox){
            json single_box;
            single_box["x"] = box.x;
            single_box["y"] = box.y;
            single_box["z"] = box.z;
            obsBdBoxArray.push_back(single_box);
        }
        single_obs_json["obsBdBox"] = obsBdBoxArray;

        pkEmapObsArray.push_back(single_obs_json);
    }
    j_obs["PkEmapObs"] = pkEmapObsArray;

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
    j_existing.push_back(j_obs);

    // 将更新后的数据写回文件
    std::ofstream outFile(filename);
    if (outFile.is_open()){
        outFile << j_existing.dump(4); // 缩进4个空格，格式化输出
        outFile.close();
    } else {
        std::cerr << "无法打开文件进行写入：" << filename << std::endl;
    }
}

void SaveFileToJson::SaveUssInfoToJson(UssIf_stPLVOutputInfo_t info,const std::string &filename){
    // 构建新的数据对象 j_uss
    json j_uss;

    // 序列化 UssIf_stPLVOutputInfo_t
    j_uss["enmPLVActiveSts"] = (int)(info.enmPLVActiveSts);

    // 序列化 UssIf_stSlotInfo
    j_uss["UssIf_stSlotInfo"] = json::array();
    for(const auto& slot_info : info.UssIf_stSlotInfo){
        json j_slot_info;
        j_slot_info["u8SlotNum"] = slot_info.u8SlotNum;

        // 序列化 UssIf_stSlotProperty
        j_slot_info["UssIf_stSlotProperty"] = json::array();
        for(const auto& slot_property : slot_info.UssIf_stSlotProperty){
            json j_slot_property;
            j_slot_property["u16SlotID"] = slot_property.u16SlotID;
            j_slot_property["enmSlotType"] = (int)(slot_property.enmSlotType);
            j_slot_property["enmSlotBottomType"] = (int)(slot_property.enmSlotBottomType);
            j_slot_property["u16SlotLength"] = slot_property.u16SlotLength;
            j_slot_property["u16SlotDepth"] = slot_property.u16SlotDepth;

            // 序列化 stSlotPt
            j_slot_property["stSlotPt"] = json::array();
            for(const auto& point : slot_property.stSlotPt){
                json j_point;
                j_point["x"] = point.x;
                j_point["y"] = point.y;
                j_slot_property["stSlotPt"].push_back(j_point);
            }

            j_slot_property["enmInSlotObstacleStatus"] = (int)(slot_property.enmInSlotObstacleStatus);

            // 序列化 stInSlotObstaclePt
            j_slot_property["stInSlotObstaclePt"] = json::array();
            for(const auto& point : slot_property.stInSlotObstaclePt){
                json j_point;
                j_point["x"] = point.x;
                j_point["y"] = point.y;
                j_slot_property["stInSlotObstaclePt"].push_back(j_point);
            }

            j_slot_property["u16UssOppositeSpace"] = slot_property.u16UssOppositeSpace;
            j_slot_property["u16UssTransverseSpace"] = slot_property.u16UssTransverseSpace;
            j_slot_property["u16ObjDistanceBetweenLineABToSlotBottom"] = slot_property.u16ObjDistanceBetweenLineABToSlotBottom;
            j_slot_property["enmDownSlotSODType"] = (int)(slot_property.enmDownSlotSODType);
            j_slot_property["u64Timestamp"] = slot_property.u64Timestamp;

            j_slot_info["UssIf_stSlotProperty"].push_back(j_slot_property);
        }

        j_uss["UssIf_stSlotInfo"].push_back(j_slot_info);
    }

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
    j_existing.push_back(j_uss);

    // 将更新后的数据写回文件
    std::ofstream outFile(filename);
    if (outFile.is_open()){
        outFile << j_existing.dump(4); // 缩进4个空格，格式化输出
        outFile.close();
    } else {
        std::cerr << "无法打开文件进行写入：" << filename << std::endl;
    }
}