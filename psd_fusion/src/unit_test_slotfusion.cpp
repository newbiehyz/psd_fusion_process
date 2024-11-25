#include <vector>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>
#include "json.hpp"
#include "psd_fusion_process_header.h"
#include "fusion.h" // 替换为你的实际头文件

#define TESTCASE 0
slotfusion slotfusion;
using json = nlohmann::json;

void loadAllData(const std::string& filename, std::vector<json>& dataArray) {
    std::ifstream inFile(filename);
    if (inFile.is_open()) {
        json j;
        try {
            inFile >> j;
            inFile.close();
            if (j.is_array()) {
                dataArray = j.get<std::vector<json>>();
            } else {
                std::cerr << "JSON 数据不是数组格式。" << std::endl;
            }
        } catch (json::parse_error& e) {
            std::cerr << "JSON 解析错误：" << e.what() << std::endl;
        }
    } else {
        std::cerr << "无法打开文件进行读取：" << filename << std::endl;
    }
}

// 定义一个辅助函数来创建矩形
apaSlotInfo createRect(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4) {
    apaSlotInfo rect;
    rect.rectInfo.pt[0] = {x1, y1}; // 左上角
    rect.rectInfo.pt[1] = {x2, y2}; // 右上角
    rect.rectInfo.pt[2] = {x3, y3}; // 右下角
    rect.rectInfo.pt[3] = {x4, y4}; // 左下角
    std::cout<<"左上角("<<x1<<","<<y1<<")"<<std::endl;
    std::cout<<"右上角("<<x2<<","<<y2<<")"<<std::endl;
    std::cout<<"右下角("<<x3<<","<<y3<<")"<<std::endl;
    std::cout<<"左下角("<<x4<<","<<y4<<")"<<std::endl;
    return rect;
}


// 定义函数：将矩形画到 JPG 文件
void drawRectanglesToJPG(const std::string &filename, std::vector<apaSlotInfo> &rectanglesA, 
                         std::vector<apaSlotInfo> &rectanglesB) {
    // 创建空白图像
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    // 计算输入的全局坐标范围
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::lowest();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::lowest();

    auto updateBounds = [&](const std::vector<apaSlotInfo> &rectangles) {
        for (const auto &rect : rectangles) {
            for (const auto &pt : rect.rectInfo.pt) {
                minX = std::min(minX, pt.x);
                maxX = std::max(maxX, pt.x);
                minY = std::min(minY, pt.y);
                maxY = std::max(maxY, pt.y);
            }
        }
    };

    // 更新范围
    updateBounds(rectanglesA);
    updateBounds(rectanglesB);

    // 图像中心点
    double center_img_x = image.cols / 2.0;
    double center_img_y = image.rows / 2.0;

    // 矩形的中心点
    double center_slots_x = (minX + maxX) / 2.0;
    double center_slots_y = (minY + maxY) / 2.0;

    // 缩放因子，保持宽高比例并留边距
    double scale_x = (image.cols * 0.8) / (maxX - minX);  // 留 20% 边距
    double scale_y = (image.rows * 0.8) / (maxY - minY);
    double scale = std::min(scale_x, scale_y);

    // 偏移量（将缩放后的矩形中心移到图像中心）
    double translate_x = center_img_x - center_slots_x * scale;
    double translate_y = center_img_y - center_slots_y * scale;

    // 应用缩放和平移并绘制矩形
    auto drawRectangles = [&](const std::vector<apaSlotInfo> &rectangles, const cv::Scalar &color) {
        for (const auto &rect : rectangles) {
            cv::Point topLeft(
                rect.rectInfo.pt[0].x * scale + translate_x,
                rect.rectInfo.pt[0].y * scale + translate_y
            );
            cv::Point bottomRight(
                rect.rectInfo.pt[2].x * scale + translate_x,
                rect.rectInfo.pt[2].y * scale + translate_y
            );

            cv::rectangle(image, topLeft, bottomRight, color, 1);

             // 绘制角点坐标
            for (int i = 0; i < 4; ++i) {
                cv::Point scaledPoint(
                    rect.rectInfo.pt[i].x * scale + translate_x,
                    rect.rectInfo.pt[i].y * scale + translate_y
                );

                // 在图像上绘制小圆点表示角点
                cv::circle(image, scaledPoint, 3, color, -1);

                // 在角点旁边绘制坐标文本
                std::string text = "(" + std::to_string(rect.rectInfo.pt[i].x) + ", " +
                                   std::to_string(rect.rectInfo.pt[i].y) + ")";
                cv::putText(image, text, scaledPoint + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }
        }
    };

    // 绘制 A 类矩形为红色
    drawRectangles(rectanglesA, cv::Scalar(0, 0, 255));
    // 绘制 B 类矩形为绿色
    drawRectangles(rectanglesB, cv::Scalar(0, 255, 0));

    // 保存图像
    cv::imwrite(filename, image);
    std::cout << "Image saved to: " << filename << std::endl;
}

void drawsingleRectanglesToJPG(const std::string &filename, std::vector<apaSlotInfo> &rectanglesA) {
    // 创建空白图像
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    // 计算输入的全局坐标范围
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::lowest();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::lowest();

    auto updateBounds = [&](const std::vector<apaSlotInfo> &rectangles) {
        for (const auto &rect : rectangles) {
            for (const auto &pt : rect.rectInfo.pt) {
                minX = std::min(minX, pt.x);
                maxX = std::max(maxX, pt.x);
                minY = std::min(minY, pt.y);
                maxY = std::max(maxY, pt.y);
            }
        }
    };

    // 更新范围
    updateBounds(rectanglesA);

    // 图像中心点
    double center_img_x = image.cols / 2.0;
    double center_img_y = image.rows / 2.0;

    // 矩形的中心点
    double center_slots_x = (minX + maxX) / 2.0;
    double center_slots_y = (minY + maxY) / 2.0;

    // 缩放因子，保持宽高比例并留边距
    double scale_x = (image.cols * 0.8) / (maxX - minX);  // 留 20% 边距
    double scale_y = (image.rows * 0.8) / (maxY - minY);
    double scale = std::min(scale_x, scale_y);

    // 偏移量（将缩放后的矩形中心移到图像中心）
    double translate_x = center_img_x - center_slots_x * scale;
    double translate_y = center_img_y - center_slots_y * scale;

    // 应用缩放和平移并绘制矩形
    auto drawRectangles = [&](const std::vector<apaSlotInfo> &rectangles, const cv::Scalar &color) {
        for (const auto &rect : rectangles) {
            cv::Point topLeft(
                rect.rectInfo.pt[0].x * scale + translate_x,
                rect.rectInfo.pt[0].y * scale + translate_y
            );
            cv::Point bottomRight(
                rect.rectInfo.pt[2].x * scale + translate_x,
                rect.rectInfo.pt[2].y * scale + translate_y
            );

            cv::rectangle(image, topLeft, bottomRight, color, 1);
        }
    };

    // 绘制矩形为红色
    drawRectangles(rectanglesA, cv::Scalar(0, 0, 255));

    // 保存图像
    cv::imwrite(filename, image);
    std::cout << "Image saved to: " << filename << std::endl;
}

void drawRDslotToJPG(const std::string &filename, const rd::QuadParkingSlots &data) {
    // 创建空白图像
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    // 计算输入的全局坐标范围
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    // 更新范围
    for (const auto &slot : data.quadParkingSlotList) {
        std::vector<rd::Point2f> points = {slot.tl, slot.tr, slot.bl, slot.br};
        for (const auto &pt : points) {
            minX = std::min(minX, pt.x);
            maxX = std::max(maxX, pt.x);
            minY = std::min(minY, pt.y);
            maxY = std::max(maxY, pt.y);
        }
    }

    // 如果没有有效的坐标，避免除零错误
    if (minX == maxX || minY == maxY) {
        std::cerr << "无法绘制矩形，坐标范围无效。" << std::endl;
        return;
    }

    // 图像中心点
    double center_img_x = image.cols / 2.0;
    double center_img_y = image.rows / 2.0;

    // 矩形的中心点
    double center_slots_x = (minX + maxX) / 2.0;
    double center_slots_y = (minY + maxY) / 2.0;

    // 缩放因子，保持宽高比例并留边距
    double scale_x = (image.cols * 0.8) / (maxX - minX);  // 留 20% 边距
    double scale_y = (image.rows * 0.8) / (maxY - minY);
    double scale = std::min(scale_x, scale_y);

    // 偏移量（将缩放后的矩形中心移到图像中心）
    double translate_x = center_img_x - center_slots_x * scale;
    double translate_y = center_img_y - center_slots_y * scale;

    // 绘制矩形
    for (const auto &slot : data.quadParkingSlotList) {
        std::vector<cv::Point> pts;
        pts.push_back(cv::Point(slot.tl.x * scale + translate_x, slot.tl.y * scale + translate_y));
        pts.push_back(cv::Point(slot.tr.x * scale + translate_x, slot.tr.y * scale + translate_y));
        pts.push_back(cv::Point(slot.br.x * scale + translate_x, slot.br.y * scale + translate_y));
        pts.push_back(cv::Point(slot.bl.x * scale + translate_x, slot.bl.y * scale + translate_y));

        // 使用多边形绘制矩形
        std::vector<std::vector<cv::Point>> contours;
        contours.push_back(pts);
        cv::polylines(image, contours, true, cv::Scalar(0, 0, 255), 2);
    }

    // 保存图像
    cv::imwrite(filename, image);
    std::cout << "图像已保存到: " << filename << std::endl;
}



// 测试用例：测试 mergeSlotLists 函数
TEST(MergeSlotListsTest, HandlesOverlapAndFusion) {
    // 创建 USS 和 VIS 的测试数据
    apaSlotListInfo outputSlot_USS;
    apaSlotListInfo outputSlot_VIS;
    apaSlotListInfo outputSlot_FUSION;

    // // USS 的车位列表
    // if (TESTCASE == 0){
    //     outputSlot_USS.WorldoutRect.push_back({createRect(2263, -11523, 2263, -8067, 8124, -8067,8123, -11523)}); // 车位1
    //     outputSlot_USS.WorldoutRect.push_back({createRect(2257, -5615, 2110, -2378, 7971, -2378, 8116, -5615)}); // 车位2
    //     outputSlot_USS.WorldoutRect.push_back({createRect(2092, -90, 2091, 3909, 7950, 3909, 7950, -90)});
    // }else if (TESTCASE == 1){
    //     outputSlot_USS.WorldoutRect.push_back({createRect(2244, -13365,2122, -10030, 7981, -10030, 8103, -13365)});
    // }else if (TESTCASE == 2){
    //     outputSlot_USS.WorldoutRect.push_back({createRect(2237, 5952, 2075, 9633, 7934, 9633, 8097, 5952)}); // 车位1
    //     outputSlot_USS.WorldoutRect.push_back({createRect(2075, 11317, 2075, 14347, 7935, 14347, 7934, 11317)}); 
    // }else if(TESTCASE == 3){
    //     outputSlot_USS.WorldoutRect.push_back({createRect(2216,-5399, 2064,-2030, 7924,-2030, 8076,-5399)}); 
    // }
    
    // // VIS 的车位列表
    // if (TESTCASE == 0){
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1975, -2424, 1875 ,220, 7600, 622, 7667, -2022)}); 
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1808, 2966, 1774, 5578, 7466, 5511, 7500, 2866)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1800, 220, 1875, 2966, 7500, 3033, 7600, 287)});
    // }else if(TESTCASE == 1){
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1607.0, 3837.0, 1573.0, 6448.0,7265.0, 6180.0, 7299.0, 3535.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1640.0, 1024.0, 1607.0, 3837.0,7332.0, 3837.0, 7366.0, 1024.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1741.0, -1620.0, 1640.0, 1057.0, 7366.0, 1325.0, 7433.0, -1352.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1808.0, -3596.0, 1741.0, -850.0, 7433.0, -281.0, 7533.0, -2993.0)}); 
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1908.0, -3596.0, 1841.0, -850.0, 7533.0, -381.0, 7600.0, -3127.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1975.0, -3462.0, 1908.0, -817.0, 7600.0, -214.0, 7667.0, -2892.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(2075.0, -3395.0, 1941.0, -716.0, 7667.0, -716.0, 7801.0, -3395.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(2142.0, -3328.0, 2075.0, -649.0, 7767.0, -113.0, 7834.0, -2792.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(2209.0, -2524.0, 2142.0, 20.0, 7834.0, 488.0, 7901.0, -2089.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(2310.0, -3361.0, 2209.0, -783.0, 7901.0, -247.0, 8002.0, -2859.0)});
    // }else if(TESTCASE == 2){
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(2310.0, -2290.0 , 2209.0, 287.0, 7935.0, 689.0, 8035.0, -1888.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(2142.0, 3033.0 , 2075.0, 5712.0, 7767.0, 5209.0, 7834.0, 2564.0)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(2209.0, 287.0, 2142.0, 3033.0, 7834.0, 3133.0, 7935.0, 388.0)});
    // }else if(TESTCASE == 3){
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1941, -2189 , 1875, 488, 7566, 823, 7633, -1854)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1875, 488 , 1741, 3200, 7466, 3301, 7566, 555)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(-4486, -3328 , -4553, -46, -10178, -817, -10145, -4098)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(1741, 3200 , 1741, 5845, 7433, 5678, 7466, 3033)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(-4553, -46 , -4620, 3100, -10279, 2598, -10212, -582)});
    //     outputSlot_VIS.WorldoutRect.push_back({createRect(-4620, 3100 , -4687, 6281, -10379, 6147, -10279, 2999)});
    // }
    

    //Draw RD info
    std::vector<json> allRDData;
    std::string filepath = "/home/gary/Downloads/patac-557e-emos_4.2.3_11211827/patac-557e-emos_4.2.3/patac-557e-emos/CodeRoot/src/psd_fusion_process/psd_fusion/src/RDinfo.json";
    loadAllData(filepath, allRDData);
    // 创建一个 rd::QuadParkingSlots 对象
    rd::QuadParkingSlots rd_info;
    int rd_time = 0;
    // 遍历所有数据
    for (const auto& data : allRDData) {
        // 提取顶层字段
        rd_info.frameTimeStampNs = data["frameTimeStampNs"];
        if (data["frameTimeStampNs"] == rd_time){
            break;
        }
        rd_info.sensorId = data["sensorId"];

        // 提取 header 信息（根据实际的字段）
        rd_info.header.seq = data["header"]["seq"];
        rd_info.header.frameId = data["header"]["frameId"];
        // 如果有其他需要的字段，继续提取

        // 提取 quadParkingSlotList 数组
        const auto& slotListArray = data["quadParkingSlotList"];
        for (const auto& slotJson : slotListArray) {
            // 创建一个 QuadParkingSlot 对象
            rd::QuadParkingSlot slot;

            // 提取四个顶点坐标
            slot.tl.x = slotJson["tl"]["x"];
            slot.tl.y = slotJson["tl"]["y"];

            slot.tr.x = slotJson["tr"]["x"];
            slot.tr.y = slotJson["tr"]["y"];

            slot.bl.x = slotJson["bl"]["x"];
            slot.bl.y = slotJson["bl"]["y"];

            slot.br.x = slotJson["br"]["x"];
            slot.br.y = slotJson["br"]["y"];

            // 提取其他字段
            slot.confidence = slotJson["confidence"];
            slot.label = slotJson["label"];
            slot.filtered = slotJson["filtered"];
            slot.slotType = slotJson["slotType"];
            slot.sTl = slotJson["sTl"];
            slot.sTr = slotJson["sTr"];
            slot.sBl = slotJson["sBl"];
            slot.sBr = slotJson["sBr"];

            slot.dirIn.x = slotJson["dirIn"]["x"];
            slot.dirIn.y = slotJson["dirIn"]["y"];

            slot.dirWidth.x = slotJson["dirWidth"]["x"];
            slot.dirWidth.y = slotJson["dirWidth"]["y"];

            slot.dirLength.x = slotJson["dirLength"]["x"];
            slot.dirLength.y = slotJson["dirLength"]["y"];

            slot.center.x = slotJson["center"]["x"];
            slot.center.y = slotJson["center"]["y"];

            slot.oppModify = slotJson["oppModify"];
            slot.isComplete = slotJson["isComplete"];
            slot.width = slotJson["width"];
            slot.length = slotJson["length"];
            slot.isVisited = slotJson["isVisited"];

            // 将 slot 添加到 quadParkingSlotList
            rd_info.quadParkingSlotList.push_back(slot);
            rd_time = data["frameTimeStampNs"];
        }

        // 此时，您已经将 JSON 数据还原为 QuadParkingSlots 对象
        // 可以根据需要对 quadParkingSlots 进行处理

        // 示例：输出 frameTimeStampNs 和 sensorId
        std::cout << "frameTimeStampNs: " << rd_info.frameTimeStampNs << std::endl;
        std::cout << "sensorId: " << rd_info.sensorId << std::endl;

        // 遍历并输出每个 QuadParkingSlot 的信息
        for (const auto& slot : rd_info.quadParkingSlotList) {
            std::cout << "Slot label: " << slot.label << std::endl;
            std::cout << "Confidence: " << slot.confidence << std::endl;
            std::cout << "Top-left corner: (" << slot.tl.x << ", " << slot.tl.y << ")" << std::endl;
            std::cout << "Bottom-right corner: (" << slot.br.x << ", " << slot.br.y << ")" << std::endl;
            // ... 输出其他需要的信息
        }

        std::string jpg = "RD_" + std::to_string(rd_time) + ".jpg";
        drawRDslotToJPG(jpg, rd_info);
    }

    // TODO: Draw DR info
    std::vector<json> allDRData;
    // std::string dr_filepath = "/home/gary/Downloads/patac-557e-emos_4.2.3_11211827/patac-557e-emos_4.2.3/patac-557e-emos/CodeRoot/src/psd_fusion_process/psd_fusion/src/RDinfo.json";
    // loadAllData(dr_filepath, allDRData);
    Loc::App2emap_DR dr_pose;
    int dr_time;
    
    // 遍历所有数据
    for(const auto &data : allDRData){
        if(dr_time == data["timeStamp"]){
            break;
        }
        dr_pose.x = data["x"];
        dr_pose.y = data["y"];
        dr_pose.canAng = data["canAng"];
        dr_pose.DRStatus = data["DRStatus"];
        dr_pose.timeStamp = data["timeStamp"];
        dr_time = dr_pose.timeStamp;
    }


    unsigned long long singleframeslotsID;
    singleframeslotsID = rd_info.frameTimeStampNs;
    // LOGT("[_test rd_info timestampNs] S32G RECEIVE timestampNs: %llu",singleframeslotsID);

    //singleframeslot
    std::vector<padVisionSlotCoord> singleframeslots;
    if (!rd_info.quadParkingSlotList.empty())
    {
        // LOGT("[_test rd_info singleframeslots] J5 SEND RD output slots size: %d",rd_info.quadParkingSlotList.size());
        for (const auto& parkingSlot : rd_info.quadParkingSlotList)
        {
            //emos -> 358-2
            padVisionSlotCoord oneslot; //中间结构体，转存rd单帧车位列表

            // 0xFF 作为默认值
            oneslot.bayType = (parkingSlot.slotType == 0) ? 0x00 : (parkingSlot.slotType == 1) ? 0x01 : 0xFF;  
            
            //@TODO 左右判断优化,按规划ABCD顺序输出车位角点
            if (parkingSlot.tl.x < 224 && parkingSlot.tr.x < 224)
            {
                oneslot.slotSide = 0x01; //x小于图像中心，判断为左
                oneslot.a.x = int(parkingSlot.tr.x);
                oneslot.a.y = int(parkingSlot.tr.y);
                oneslot.b.x = int(parkingSlot.tl.x);
                oneslot.b.y = int(parkingSlot.tl.y);
                oneslot.c.x = int(parkingSlot.bl.x);
                oneslot.c.y = int(parkingSlot.bl.y);
                oneslot.d.x = int(parkingSlot.br.x);
                oneslot.d.y = int(parkingSlot.br.y);
            }
            else 
            {
                oneslot.slotSide = 0x00;
                oneslot.a.x = int(parkingSlot.tl.x);
                oneslot.a.y = int(parkingSlot.tl.y);
                oneslot.b.x = int(parkingSlot.tr.x);
                oneslot.b.y = int(parkingSlot.tr.y);
                oneslot.c.x = int(parkingSlot.br.x);
                oneslot.c.y = int(parkingSlot.br.y);
                oneslot.d.x = int(parkingSlot.bl.x);
                oneslot.d.y = int(parkingSlot.bl.y);

            }
            singleframeslots.push_back(oneslot);
        }
    }

    // Draw Vison slot
    std::vector<json> allVisonData;
    std::string visonslotpath = "/home/gary/Downloads/patac-557e-emos_4.2.3_11211827/patac-557e-emos_4.2.3/patac-557e-emos/CodeRoot/src/psd_fusion_process/psd_fusion/src/VISapaSlotListInfo.json";
    loadAllData(visonslotpath, allVisonData);
    int timestamp = 0;
    // 遍历所有数据
    for (const auto& data : allVisonData) {
        // 创建一个 std::vector<apaSlotInfo> 容器
        std::vector<apaSlotInfo> apaSlotInfos;

        // 提取 slots_in_cur_frame 数组（根据您的 JSON 结构调整字段名）
        const auto& slotListArray = data["WorldoutRect"];
        const auto& ullFrameId = data["ullFrameId"];
        std::cout<<"hello"<<std::endl;
        if (data["ullFrameId"] == timestamp){
            continue;
        }

        for (const auto& slotJson : slotListArray) {
            // 创建一个 apaSlotInfo 对象
            apaSlotInfo slot;

            // 提取 rectInfo 信息
            slot.rectInfo.label = slotJson["rectInfo"]["label"];
            slot.rectInfo.PStype = slotJson["rectInfo"]["PStype"];

            // 提取坐标点信息
            for (int idx = 0; idx < RECTPointNum; ++idx) {
                slot.rectInfo.pt[idx].x = slotJson["rectInfo"]["points"][idx]["x"];
                slot.rectInfo.pt[idx].y = slotJson["rectInfo"]["points"][idx]["y"];
            }

            // 提取其他字段
            slot.detect_frame_count = slotJson["detect_frame_count"];
            slot.is_reliable = slotJson["is_reliable"];

            // 将 slot 添加到 apaSlotInfos
            apaSlotInfos.push_back(slot);
            timestamp = data["ullFrameId"];  
        }

        // 示例：输出信息
        std::cout << "Total slots: " << apaSlotInfos.size() << std::endl;

        // 遍历并输出每个 apaSlotInfo 的信息
        for (const auto& slot : apaSlotInfos) {
            std::cout << "Slot label: " << slot.rectInfo.label << std::endl;
            std::cout << "PStype: " << slot.rectInfo.PStype << std::endl;
            std::cout << "First corner: (" << slot.rectInfo.pt[0].x << ", " << slot.rectInfo.pt[0].y << ")" << std::endl;
            
            std::string jpg = "VISION_SLOT_" + std::to_string(timestamp) + ".jpg";
            drawsingleRectanglesToJPG(jpg, apaSlotInfos);
        }      
    }

    // drawsingleRectanglesToJPG("slot_fusion.jpg",outputSlot_FUSION.WorldoutRect);

    //Draw Fusion slot
    // drawRectanglesToJPG("slots_vis& slots_uss.jpg", outputSlot_USS.WorldoutRect, outputSlot_VIS.WorldoutRect);
    
    // 调用 mergeSlotLists
    // slotfusion.mergeSlotLists(outputSlot_USS, outputSlot_VIS, outputSlot_FUSION);

    // 验证融合后的车位数量是否正确
    EXPECT_EQ(outputSlot_FUSION.WorldoutRect.size(), 5); // 应该有3个车位
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
