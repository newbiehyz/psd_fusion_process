#include <vector>
#include <fstream>
#include <thread>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>
#include "json.hpp"
#include "psd_fusion_process_header.h"
#include "fusion.h" // 替换为你的实际头文件
#include "PSD_FusionModuleIF.h"

#define TESTCASE 0

using json = nlohmann::json;

std::filesystem::path currentPath = std::filesystem::current_path();
std::filesystem::path parentPath = currentPath.parent_path();


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

int CalcDistance(float ax, float ay, float bx,  float by)
{
    float dis = sqrt((ax-bx)*(ax-bx) + (ay-by)*(ay-by));
    return dis;
}

// Draw apaSlotListInfo
void drawapaSlotlistinfoToJPG(const std::string &filename,apaSlotListInfo &rectanglesA, Loc::App2emap_DR pose, Fus::PkEmapObs obs) {
    // 创建空白图像
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    std::string timestampText = "(RD)timestamp " + std::to_string(rectanglesA.ullFrameId);
    cv::putText(image, timestampText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    
    // 计算输入的全局坐标范围
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::lowest();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::lowest();

    auto updateBounds = [&](const apaSlotListInfo &rectangles) {
        for (const auto &rect : rectangles.WorldoutRect) {
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
    auto drawRectangles = [&](const apaSlotListInfo &rectangles, const cv::Scalar &color) {
        for (const auto &rect : rectangles.WorldoutRect) {
            
            cv::Point center;
            center.x = ((rect.rectInfo.pt[0].x + rect.rectInfo.pt[2].x) / 2.0)* scale + translate_x;
            center.y = ((rect.rectInfo.pt[0].y + rect.rectInfo.pt[2].y) / 2.0)* scale + translate_y;
            std::string id_text = "id: " + std::to_string(rect.rectInfo.label);
            cv::putText(image, id_text, center, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

            int width, length;
            length = CalcDistance(rect.rectInfo.pt[0].x, rect.rectInfo.pt[0].y, rect.rectInfo.pt[1].x, rect.rectInfo.pt[1].y);
            float width1 = CalcDistance(rect.rectInfo.pt[2].x, rect.rectInfo.pt[2].y, rect.rectInfo.pt[1].x, rect.rectInfo.pt[1].y);
            float width2 = CalcDistance(rect.rectInfo.pt[0].x, rect.rectInfo.pt[0].y, rect.rectInfo.pt[3].x, rect.rectInfo.pt[3].y);
            width = (width1+width2)/2.0;
            std::string lw_text = "W: " + std::to_string(width) + ",L:" + std::to_string(length);
            cv::putText(image, lw_text, center+cv::Point(0, -10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

            std::vector<cv::Point> scaledPoints;
            for (int i = 0; i < 4; ++i) {
                cv::Point scaledPoint(
                    rect.rectInfo.pt[i].x * scale + translate_x,
                    rect.rectInfo.pt[i].y * scale + translate_y
                );
                scaledPoints.push_back(scaledPoint);
            }

            // 绘制四边形
            cv::polylines(image, scaledPoints, true, color, 1);

            // 绘制角点坐标
            for (int i = 0; i < 4; ++i) {
                cv::Point scaledPoint = scaledPoints[i];

                // 在图像上绘制小圆点表示角点
                cv::circle(image, scaledPoint, 3, color, -1);

                // 在角点旁边绘制坐标文本
                std::string text = "(" + std::to_string(static_cast<int>((scaledPoint.x - translate_x)/scale)) + ", " +
                                   std::to_string(static_cast<int>((scaledPoint.y - translate_y)/scale )) + ")";
                cv::putText(image, text, scaledPoint + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }

            // 绘制车辆位姿
            // 定义矩形的大小
            int rectSize = 30; // 矩形从中心点向四周延伸的像素数
            cv::Point vehicle_pose(pose.x * scale + translate_x,
                                   pose.y * scale + translate_y);
            cv::Point cartopLeft(pose.x * scale + translate_x - rectSize, 
                              pose.y * scale + translate_y - rectSize);
            cv::Point carbottomRight(pose.x * scale + translate_x + rectSize, 
                              pose.y * scale + translate_y + rectSize);

            cv::rectangle(image, cartopLeft, carbottomRight, color, cv::FILLED);
        }
    };

    for (int i = 0; i < 50; ++i){
        float x = obs.pkEmapObs[i].obsCenter.x * 1000;
        float y = obs.pkEmapObs[i].obsCenter.y * 1000;
    }

    // 绘制矩形为红色
    drawRectangles(rectanglesA, cv::Scalar(0, 0, 255));

    // 保存图像
    cv::imwrite(filename, image);
    std::cout << "Image saved to: " << filename << std::endl;
}

void drawUSSslotToJPG(const std::string &filename, const UssIf_stPLVOutputInfo_t &data, Loc::App2emap_DR pose, UssIf_stPLVOutputInfo_t post_uss_info){
    // 创建空白图像
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    std::string timestampText = "(USS)timestamp " + std::to_string(data.UssIf_stSlotInfo->UssIf_stSlotProperty[0].u64Timestamp);
    cv::putText(image, timestampText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // 计算输入的全局坐标范围
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

     // Lambda 函数用于更新坐标范围
    auto updateBounds = [&](const UssIf_stPLVOutputInfo_t &info) {
        for (const auto &slotInfo : info.UssIf_stSlotInfo) {
            for (const auto &slotProperty : slotInfo.UssIf_stSlotProperty) {
                for (const auto &pt : slotProperty.stSlotPt) {
                    minX = std::min(minX, static_cast<float>(pt.x));
                    maxX = std::max(maxX, static_cast<float>(pt.x));
                    minY = std::min(minY, static_cast<float>(pt.y));
                    maxY = std::max(maxY, static_cast<float>(pt.y));
                }
                for (const auto &pt : slotProperty.stInSlotObstaclePt) {
                    minX = std::min(minX, static_cast<float>(pt.x));
                    maxX = std::max(maxX, static_cast<float>(pt.x));
                    minY = std::min(minY, static_cast<float>(pt.y));
                    maxY = std::max(maxY, static_cast<float>(pt.y));
                }
            }
        }
    };

    // 更新坐标范围
    updateBounds(data);

    // 图像中心点
    double center_img_x = image.cols / 2.0;
    double center_img_y = image.rows / 2.0;

    // 矩形的中心点
    double center_slots_x = (minX + maxX) / 2.0;
    double center_slots_y = (minY + maxY) / 2.0;

    // 缩放因子，保持宽高比例并留边距
    double scale_x = (image.cols * 0.8) / (maxX - minX);
    double scale_y = (image.rows * 0.8) / (maxY - minY);
    double scale = std::min(scale_x, scale_y);

    // 偏移量（将缩放后的矩形中心移到图像中心）
    double translate_x = center_img_x - center_slots_x * scale;
    double translate_y = center_img_y - center_slots_y * scale;

    // Lambda 函数用于绘制处理前的超声车位矩形和相关信息
    auto drawRectangles = [&](const UssIf_stPLVOutputInfo_t &info, const cv::Scalar &color) {
        for (const auto &slotInfo : info.UssIf_stSlotInfo) {
            for (const auto &slotProperty : slotInfo.UssIf_stSlotProperty) {
                if (slotProperty.u16SlotID == 0){
                    break;
                }
                // 计算矩形中心点
                float center_x = (slotProperty.stSlotPt[0].x + slotProperty.stSlotPt[2].x) / 2.0f;
                float center_y = (slotProperty.stSlotPt[0].y + slotProperty.stSlotPt[2].y) / 2.0f;
                cv::Point center(
                    static_cast<int>(center_x * scale + translate_x),
                    static_cast<int>(center_y * scale + translate_y)
                );

                // 添加 ID 文本
                std::string id_text = "ID: " + std::to_string(slotProperty.u16SlotID);
                cv::putText(image, id_text, center, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

                // 计算长度和宽度
                float length = CalcDistance(slotProperty.stSlotPt[0].x, slotProperty.stSlotPt[0].y, slotProperty.stSlotPt[1].x, slotProperty.stSlotPt[1].y);
                float width1 = CalcDistance(slotProperty.stSlotPt[2].x, slotProperty.stSlotPt[2].y, slotProperty.stSlotPt[1].x, slotProperty.stSlotPt[1].y);
                float width2 = CalcDistance(slotProperty.stSlotPt[0].x, slotProperty.stSlotPt[0].y, slotProperty.stSlotPt[3].x, slotProperty.stSlotPt[3].y);
                float width = (width1 + width2) / 2.0f;

                // 添加尺寸文本
                std::string lw_text = "W: " + std::to_string((int)width) + ", L: " + std::to_string((int)length);
                cv::putText(image, lw_text, center + cv::Point(0, -10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

                // 缩放和平移点
                std::vector<cv::Point> scaledPoints;
                for (int i = 0; i < 4; ++i) {
                    cv::Point scaledPoint(
                        static_cast<int>(slotProperty.stSlotPt[i].x * scale + translate_x),
                        static_cast<int>(slotProperty.stSlotPt[i].y * scale + translate_y)
                    );
                    scaledPoints.push_back(scaledPoint);
                }

                // 绘制四边形
                cv::polylines(image, scaledPoints, true, color, 1);

                // 绘制角点坐标
                for (int i = 0; i < 4; ++i) {
                    cv::Point scaledPoint = scaledPoints[i];
                    cv::circle(image, scaledPoint, 3, color, -1);
                    std::string text = "(" + std::to_string(static_cast<int>((scaledPoint.x - translate_x)/scale)) + ", " +
                                       std::to_string(static_cast<int>((scaledPoint.y - translate_y)/scale )) + ")";
                    cv::putText(image, text, scaledPoint + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.3, color, 1);
                }

                // 绘制障碍点
                // for (const auto &obsPt : slotProperty.stInSlotObstaclePt) {
                //     cv::Point scaledObsPt(
                //         static_cast<int>(obsPt.x * scale + translate_x),
                //         static_cast<int>(obsPt.y * scale + translate_y)
                //     );
                //     cv::circle(image, scaledObsPt, 3, cv::Scalar(0, 255, 255), -1);
                //     cv::putText(image, "Obstacle", scaledObsPt + cv::Point(5, 5), cv::FONT_HERSHEY_SIMPLEX, 0.3, cv::Scalar(0, 255, 255), 1);
                // }
            }
        }
    };

    // 调用绘制函数，使用红色
    drawRectangles(data, cv::Scalar(0, 0, 255));
    drawRectangles(post_uss_info, cv::Scalar(0,255,0));
    // 绘制车辆位姿
    int rectSize = 30; // 矩形从中心点向四周延伸的像素数
    cv::Point vehicle_pose(
        static_cast<int>(pose.x * scale + translate_x),
        static_cast<int>(pose.y * scale + translate_y)
    );
    cv::Point cartopLeft(vehicle_pose.x - rectSize, vehicle_pose.y - rectSize);
    cv::Point carbottomRight(vehicle_pose.x + rectSize, vehicle_pose.y + rectSize);
    cv::rectangle(image, cartopLeft, carbottomRight, cv::Scalar(0, 255, 0), cv::FILLED);
    cv::putText(image, "Vehicle", vehicle_pose + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

    // 保存图像
    if (cv::imwrite(filename, image)) {
        std::cout << "Image saved to: " << filename << std::endl;
    } else {
        std::cerr << "Failed to save image to: " << filename << std::endl;
    }
}

void drawRDslotToJPG(const std::string &filename, const rd::QuadParkingSlots &data) {
    // 创建空白图像
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    std::string timestampText = "(RD)timestamp " + std::to_string(data.frameTimeStampNs);
    cv::putText(image, timestampText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

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
    // 使用多边形绘制矩形
    std::vector<std::vector<cv::Point>> contours;

    // 绘制矩形
    for (const auto &slot : data.quadParkingSlotList) {
        std::vector<cv::Point> pts;
        pts.push_back(cv::Point(slot.tl.x * scale + translate_x, slot.tl.y * scale + translate_y));
        pts.push_back(cv::Point(slot.tr.x * scale + translate_x, slot.tr.y * scale + translate_y));
        pts.push_back(cv::Point(slot.br.x * scale + translate_x, slot.br.y * scale + translate_y));
        pts.push_back(cv::Point(slot.bl.x * scale + translate_x, slot.bl.y * scale + translate_y));
        // 绘制角点坐标
        for (int i = 0; i < 4; ++i) {
            cv::Point scaledPoint = pts[i];

            // 在图像上绘制小圆点表示角点
            cv::circle(image, scaledPoint, 3, cv::Scalar(0, 0, 255), -1);

            // 在角点旁边绘制坐标文本
            std::string text = "(" + std::to_string(static_cast<int>((scaledPoint.x - translate_x)/scale)) + ", " +
                                    std::to_string(static_cast<int>((scaledPoint.y - translate_y)/scale )) + ")";
            cv::putText(image, text, scaledPoint + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
        }
        contours.push_back(pts);
    }
    cv::polylines(image, contours, true, cv::Scalar(0, 0, 255), 2);
    
    

    // 保存图像
    cv::imwrite(filename, image);
    // std::cout << "图像已保存到: " << filename << std::endl;
}

std::vector<json> allRDData;
std::vector<json> allDRData;
std::vector<json> allAPAStatusData;
std::vector<json> allSelectSlotData;
std::vector<json> allSelectSlot2Data;
std::vector<json> allObsData;
std::vector<json> allVisionData;
std::vector<json> allUSSData;

bool rd_dataloaded = false;
bool dr_dataloaded = false;
bool apastatus_dataloaded = false;
bool selectslot_dataloaded = false;
bool selectslot2_dataloaded = false;
bool obs_dataloaded = false;
bool vison_dataloaded = false;
bool uss_dataloaded = false;
bool is_init = false;
PSD_FusionModuleIF PSD_FusionModuleIFrunable;

void TimeTrigger_Timer50(){
    
    //Get RD info
    if (!rd_dataloaded){

        // std::string filepath = "/home/gary/Downloads/patac-557e-emos_4.2.3_11211827/patac-557e-emos_4.2.3/patac-557e-emos/CodeRoot/src/psd_fusion_process/psd_fusion/src/RDinfo.json";
        std::string filepath = (parentPath / "RDinfo.json").string();
        
        loadAllData(filepath, allRDData);
        rd_dataloaded = true;
    }
    static size_t currentIndex = 0;
    if(currentIndex >= allRDData.size()){
        // std::cout<<"所有数据已处理完毕"<<std::endl;
        return;
    }
    const auto &data = allRDData[currentIndex];
    // 创建一个 rd::QuadParkingSlots 对象
    rd::QuadParkingSlots rd_info;
    rd_info.frameTimeStampNs = data["frameTimeStampNs"];
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
        }
    std::string jpg = "RD_" + std::to_string(rd_info.frameTimeStampNs) + ".jpg";
    drawRDslotToJPG(jpg, rd_info);
    
    // Get DR info
    if (!dr_dataloaded){
        std::string filepath = (parentPath / "DR_POSE.json").string();    
        loadAllData(filepath, allDRData);
        dr_dataloaded = true;
    }
    static size_t currentDRIndex = 0;
    if(currentDRIndex >= allDRData.size()){
        // std::cout<<"所有数据已处理完毕"<<std::endl;
        return;
    }
    const auto &dr_data = allDRData[currentDRIndex];
    Loc::App2emap_DR dr_pose;
    dr_pose.x = dr_data["x"];
    dr_pose.y = dr_data["y"];
    dr_pose.canAng = dr_data["canAng"];
    dr_pose.DRStatus = dr_data["DRStatus"];
    dr_pose.timeStamp = dr_data["timeStamp"];

    //Get obs info
    if (!obs_dataloaded){
        std::string filepath = (parentPath / "ObsInfo.json").string();
        loadAllData(filepath, allObsData);
        obs_dataloaded = true;
    }
    static size_t currentOBSIndex = 0;
    if(currentOBSIndex >= allObsData.size()){
        // std::cout<<"所有数据已处理完毕"<<std::endl;
        return;
    }
    const auto &obs_data = allObsData[currentOBSIndex];

    Fus::PkEmapObs obs_info;
    const auto& obs_info_array = obs_data["PkEmapObs"];
    for (int obs_index = 0; obs_index < 50; ++obs_index){
        const auto & obs_info_array_json = obs_info_array[obs_index];
        obs_info.pkEmapObs[obs_index].FrameIndex = obs_info_array_json["FrameIndex"];
        obs_info.pkEmapObs[obs_index].obsID = obs_info_array_json["obsID"];
        obs_info.pkEmapObs[obs_index].obsTyp = obs_info_array_json["obsTyp"];
        obs_info.pkEmapObs[obs_index].obsCenter.x = obs_info_array_json["obsCenter"]["x"];
        obs_info.pkEmapObs[obs_index].obsCenter.y = obs_info_array_json["obsCenter"]["y"];
        obs_info.pkEmapObs[obs_index].obsCenter.z = obs_info_array_json["obsCenter"]["z"];
        obs_info.pkEmapObs[obs_index].age = obs_info_array_json["age"];

        std::cout<<"FrameIndex:"<<obs_info.pkEmapObs[obs_index].FrameIndex<<std::endl;
    }


    // //Get apastatus info
    // if (!apastatus_dataloaded){
    //     std::string filepath = (parentPath / "APAStatus.json").string();
    //     loadAllData(filepath, allAPAStatusData);
    //     apastatus_dataloaded = true;
    // }
    // static size_t currentIndex = 0;
    // if(currentIndex >= allAPAStatusData.size()){
    //     // std::cout<<"所有数据已处理完毕"<<std::endl;
    //     return;
    // }
    // const auto &apastatus_data = allAPAStatusData[currentIndex];
    // int  apa_status;
    // apa_status = apastatus_data["aps_apaStatusReq"];

    // //Get selectslot(VCU) info
    // if (!selectslot_dataloaded){
    //     std::string filepath = (parentPath / "SelectSlot.json").string();
    //     loadAllData(filepath, allSelectSlotData);
    //     selectslot_dataloaded = true;
    // }
    // static size_t currentIndex = 0;
    // if(currentIndex >= allSelectSlotData.size()){
    //     // std::cout<<"所有数据已处理完毕"<<std::endl;
    //     return;
    // }
    // const auto &selectslot_data = allSelectSlotData[currentIndex];
    // int  VCU_select_ID_ON;
    // VCU_select_ID_ON = selectslot_data["SelectSlotID"];


    // //Get selectslot2(HMI) info
    // if (!selectslot2_dataloaded){
    //     std::string filepath = (parentPath / "SelectSlot2.json").string();
    //     loadAllData(filepath, allSelectSlot2Data);
    //     selectslot2_dataloaded = true;
    // }
    // static size_t currentIndex = 0;
    // if(currentIndex >= allSelectSlot2Data.size()){
    //     // std::cout<<"所有数据已处理完毕"<<std::endl;
    //     return;
    // }
    // const auto &selectslot2_data = allSelectSlotData2[currentIndex];
    // int  HMI_select_ID;
    // HMI_select_ID = selectslot2_data["SelectSlotID"];


    padVehiclePose  pose_globaldata;

    uint64_t singleframeslotsID;
    singleframeslotsID = rd_info.frameTimeStampNs;

    //singleframeslot
    std::vector<padVisionSlotCoord> singleframeslots;
    if (!rd_info.quadParkingSlotList.empty())
    {
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
    
    if (!is_init){
        PSD_FusionModuleIFrunable.Initialize();
        is_init = true;
    }
    

    pose_globaldata.coord.x = int(dr_pose.x);
    pose_globaldata.coord.y = int(dr_pose.y);
    pose_globaldata.yaw = dr_pose.canAng;
    std::cout<<"timestamp:"<<dr_pose.timeStamp<<std::endl;
    std::cout<<"coord.x:"<<pose_globaldata.coord.x<<std::endl;
    std::cout<<"coord.y:"<<pose_globaldata.coord.y<<std::endl;
    std::cout<<"coord.yaw:"<<pose_globaldata.yaw<<std::endl;
    PSD_FusionModuleIFrunable.UpdateVechiclePose(pose_globaldata);
    PSD_FusionModuleIFrunable.UpdateVisionSlots(singleframeslotsID, singleframeslots, 0, 0);
    
    apaSlotListInfo outputSlot_CALVIS = PSD_FusionModuleIFrunable.GetOutputSlot();
    std::cout<<"Update vision slot size: "<< outputSlot_CALVIS.WorldoutRect.size()<<std::endl;
    drawapaSlotlistinfoToJPG("Cal Vision slot.jpg",outputSlot_CALVIS, dr_pose, obs_info);

    // ********************************Load Obs data
    // Calculate stop distance
    // Fus::PkEmapObs obs;
    // obs.pkEmapObs[0].obsTyp = Fus::OBS_WHEELSTOP;
    // // obs.pkEmapObs[0].obsCenter.x = 7540;
    // // obs.pkEmapObs[0].obsCenter.y = 4649;
    // obs.pkEmapObs[0].obsCenter.x = 3800;
    // obs.pkEmapObs[0].obsCenter.y = 700;
    // obs.pkEmapObs[1].obsTyp = Fus::OBS_WHEELSTOP;
    // // obs.pkEmapObs[1].obsCenter.x = 7568;
    // // obs.pkEmapObs[1].obsCenter.y = 2280;
    // obs.pkEmapObs[1].obsCenter.x = 3800;
    // obs.pkEmapObs[1].obsCenter.y = 6300;
    float stop_dis;
    PSD_FusionModuleIFrunable.CalStopDistance(obs_info, stop_dis);
    std::cout<<"Stop distance:"<<stop_dis<<std::endl;

    // *******************************Load original USS data 
    // Get USS info
    // if (!uss_dataloaded){
    //     std::string filepath = (parentPath / "USSapaSlotListInfo.json").string();    
    //     loadAllData(filepath, allUSSData);
    //     uss_dataloaded = true;
    // }
    // static size_t currentUSSIndex = 0;
    // if(currentUSSIndex >= allUSSData.size()){
    //     // std::cout<<"所有数据已处理完毕"<<std::endl;
    //     return;
    // }
    // const auto &uss_data = allUSSData[currentUSSIndex];
    // UssIf_stPLVOutputInfo_t uss_info;
    // uss_info.enmPLVActiveSts =  static_cast<UssIf_enmActiveSts_t>((int)uss_data["enmPLVActiveSts"]);
    // // 解析 UssIf_stSlotInfo 数组
    // const auto& slotInfoArray = uss_data["UssIf_stSlotInfo"];
    // size_t slot_info_size = sizeof(uss_info.UssIf_stSlotInfo)/sizeof(uss_info.UssIf_stSlotInfo[0]);
    // // 注意：UssIf_stSlotInfo是std::array大小为4，如果JSON中超过4个只取前4个，少于4个则只解析那么多。
    // for (size_t i = 0; i < slot_info_size && i < slotInfoArray.size(); ++i) {
    //     const auto& slotInfoJson = slotInfoArray[i];
    //     UssIf_stSlotInfo_t uss_slot_info;
    //     uss_slot_info.u8SlotNum = slotInfoJson["u8SlotNum"];

    //     const auto& slotPropArray = slotInfoJson["UssIf_stSlotProperty"];
    //     // UssIf_stSlotProperty为std::array大小为3，同理进行安全访问
    //     size_t slot_prop_size = sizeof(uss_slot_info.UssIf_stSlotProperty)/sizeof(uss_slot_info.UssIf_stSlotProperty[0]);
    //     for (size_t p = 0; p < slot_prop_size && p < slotPropArray.size(); ++p) {
    //         const auto& slotPropJson = slotPropArray[p];
    //         UssIf_stSlotProperty_t uss_slot_prop;
    //         uss_slot_prop.u16SlotID = slotPropJson["u16SlotID"];
    //         uss_slot_prop.enmSlotType = static_cast<UssIf_enmSlotType_t>((int)slotPropJson["enmSlotType"]);
    //         uss_slot_prop.enmSlotBottomType = static_cast<UssIf_enmSlotBottomType_t>((int)slotPropJson["enmSlotBottomType"]);
    //         uss_slot_prop.u16SlotLength = slotPropJson["u16SlotLength"];
    //         uss_slot_prop.u16SlotDepth = slotPropJson["u16SlotDepth"];
    //         uss_slot_prop.u64Timestamp = slotPropJson["u64Timestamp"];

    //         // 解析 stSlotPt (4个点)
    //         const auto& slotPtArray = slotPropJson["stSlotPt"];
    //         size_t slot_pt_size = sizeof(uss_slot_prop.stSlotPt)/sizeof(uss_slot_prop.stSlotPt[0]);
    //         for (size_t ptIdx = 0; ptIdx < slot_pt_size && ptIdx < slotPtArray.size(); ++ptIdx) {
    //             uss_slot_prop.stSlotPt[ptIdx].x = slotPtArray[ptIdx]["x"];
    //             uss_slot_prop.stSlotPt[ptIdx].y = slotPtArray[ptIdx]["y"];
    //         }

    //         // // 如果有 stInSlotObstaclePt
    //         // if (slotPropJson.contains("stInSlotObstaclePt")) {
    //         //     const auto& obsPtArray = slotPropJson["stInSlotObstaclePt"];
    //         //     for (size_t obsIdx = 0; obsIdx < uss_slot_prop.stInSlotObstaclePt.size() && obsIdx < obsPtArray.size(); ++obsIdx) {
    //         //         uss_slot_prop.stInSlotObstaclePt[obsIdx].x = obsPtArray[obsIdx]["x"];
    //         //         uss_slot_prop.stInSlotObstaclePt[obsIdx].y = obsPtArray[obsIdx]["y"];
    //         //     }
    //         // }

    //         // 解析剩余字段
    //         if (slotPropJson.contains("u16UssOppositeSpace")) uss_slot_prop.u16UssOppositeSpace = slotPropJson["u16UssOppositeSpace"];
    //         if (slotPropJson.contains("u16UssTransverseSpace")) uss_slot_prop.u16UssTransverseSpace = slotPropJson["u16UssTransverseSpace"];
    //         if (slotPropJson.contains("u16ObjDistanceBetweenLineABToSlotBottom")) uss_slot_prop.u16ObjDistanceBetweenLineABToSlotBottom = slotPropJson["u16ObjDistanceBetweenLineABToSlotBottom"];
    //         if (slotPropJson.contains("enmDownSlotSODType")) uss_slot_prop.enmDownSlotSODType = static_cast<UssIf_enmDownSlotSODType_t>((int)slotPropJson["enmDownSlotSODType"]);

    //         uss_slot_info.UssIf_stSlotProperty[p] = uss_slot_prop;
    //     }

    //     uss_info.UssIf_stSlotInfo[i] = uss_slot_info;
    // }

    // auto pre_post_uss_slots = uss_info;
    // slotfusion sf;
    // sf.mergeUSSleftandright(total_slots, uss_info);
    // sf.postprocessUSSslots(uss_info);
    // std::string uss_jpg = "USS_" + std::to_string(uss_info.UssIf_stSlotInfo->UssIf_stSlotProperty[0].u64Timestamp) + ".jpg";
    // drawUSSslotToJPG(uss_jpg, pre_post_uss_slots, dr_pose, uss_info);

    // apaSlotListInfo outputSlot_FUSED;
    
    // sf.mergeSlotLists(apaUSSSlotlistInfos, apaSlotlistInfos, outputSlot_FUSED);

    currentIndex++;
    currentDRIndex++;
    // currentVisionIndex++;
    // currentUSSIndex++;
    currentOBSIndex++;
}

int main(int argc, char **argv) {
    while (true) {
        auto start = std::chrono::steady_clock::now();

        TimeTrigger_Timer50();  // 调用您的函数

        // 计算已经消耗的时间
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

        // 计算需要等待的时间，确保每次循环间隔为50毫秒
        auto sleepTime = std::chrono::milliseconds(100) - elapsed;
        if (sleepTime > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleepTime);
        } else {
            // 如果函数执行时间超过50毫秒，立即进行下一次调用
            std::cerr << "Warning: Function execution took longer than 50ms." << std::endl;
        }
    }

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    
}
