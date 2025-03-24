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

void drawapaSlotlistinfoToJPG(const std::string &filename,apaSlotListInfo &rectanglesA, Loc::App2emap_DR &pose, 
                              Fus::PkEmapObs &obs, std::vector<padVisionSlotCoord> &singleframeslots) {
    // 创建空白图像
    cv::Mat image(1200, 1200, CV_8UC3, cv::Scalar(255, 255, 255));

    std::string timestampText = "(RD)timestamp " + std::to_string(rectanglesA.ullFrameId);
    cv::putText(image, timestampText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);


    // 计算输入的全局坐标范围
    int minX = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::lowest();
    int minY = std::numeric_limits<int>::max();
    int maxY = std::numeric_limits<int>::lowest();

    auto updateBounds = [&](const apaSlotListInfo &rectangles) {
        // for (const auto &rect : rectangles.WorldoutRect) {
        for (const auto &rect : rectangles.slots_in_cur_frame) {
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
    // double center_slots_x = (minX + maxX) / 2.0;
    // double center_slots_y = (minY + maxY) / 2.0;

    // 缩放因子，保持宽高比例并留边距
    // double scale_x = (image.cols * 0.8) / (maxX - minX);  // 留 20% 边距
    // double scale_y = (image.rows * 0.8) / (maxY - minY);
    // double scale = std::min(scale_x, scale_y);
    double scale = 0.05;

    // 偏移量（将缩放后的矩形中心移到图像中心）
    double translate_x = center_img_x;
    double translate_y = center_img_y;

    // 画障碍物
    float obs_x, obs_y;
    for (int i = 0; i < 50; ++i) {
        obs_x = (obs.pkEmapObs[i].obsCenter.x * 1000);
        obs_y = (obs.pkEmapObs[i].obsCenter.y * 1000);

        // 如果坐标为 0，跳过绘制
        if (obs_x == 0 || obs_y == 0) {
            continue;
        }

        // 应用缩放和平移
        float scaled_obs_x = obs_x * scale + translate_x;
        float scaled_obs_y = obs_y * scale + translate_y;

        // // 绘制障碍物（圆）
        // cv::circle(image, cv::Point(scaled_obs_x, scaled_obs_y), 10, cv::Scalar(0, 0, 0), -1);

        // // 准备角点文字内容
        // std::string text = "(" + std::to_string(static_cast<int>(obs_x)) + ", " +
        //                 std::to_string(static_cast<int>(obs_y)) + ")";
        // // 确定文本位置（圆的正上方）
        // int text_offset_y = 25; // 文本与圆之间的垂直偏移量
        // cv::Point textPosition(scaled_obs_x, scaled_obs_y - text_offset_y);
        // // 绘制文本
        // cv::putText(image, text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

        // //准备位置文字内容
        // std::string text_loc;
        // switch (obs_location){
        //     case 0:
        //         text_loc = "NO OBS";
        //         break;
        //     case 1:
        //         text_loc = "near AB";
        //         break;
        //     case 2:
        //         text_loc = "near BC";
        //         break;
        //     case 3:
        //         text_loc = "near CD";
        //         break;
        //     case 4:
        //         text_loc = "near DA";
        //         break;
        //     default:
        //         break;
        // }
        // int text_offset_y_loc = 40;
        // cv::Point locTextPosition(scaled_obs_x, scaled_obs_y - text_offset_y_loc);
        // cv::putText(image, text_loc, locTextPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);

    }

    // // 绘制单帧车位
    // for (const auto &singleframeslot : singleframeslots) {
    //     cv::Point single_A((singleframeslot.a.x - 448) * scale * 20000 / 896 + translate_x,
    //                        ((singleframeslot.a.y - 448)  * -1 * 20000 / 896 + 1516.35) * scale + translate_y);
    //     cv::Point single_B((singleframeslot.b.x - 448) * scale * 20000 / 896 + translate_x,
    //                        ((singleframeslot.b.y - 448)  * -1 * 20000 / 896 + 1516.35) * scale + translate_y);
    //     cv::Point single_C((singleframeslot.c.x - 448) * scale * 20000 / 896 + translate_x,
    //                        ((singleframeslot.c.y - 448)  * -1 * 20000 / 896 + 1516.35) * scale + translate_y);
    //     cv::Point single_D((singleframeslot.d.x - 448) * scale * 20000 / 896 + translate_x,
    //                        ((singleframeslot.d.y - 448)  * -1 * 20000 / 896 + 1516.35) * scale + translate_y);
    //     std::vector<cv::Point> scaledSingleFramePoints{single_A, single_B, single_C, single_D};
    //     if (singleframeslot.occupy) {
    //         std::vector<std::vector<cv::Point>> contours_single = {scaledSingleFramePoints};
    //         cv::fillPoly(image, contours_single, cv::Scalar(230, 216, 173));
    //     }
    //     cv::polylines(image, scaledSingleFramePoints, true, cv::Scalar(255, 0, 0), 1);
    // }

    // 应用缩放和平移并绘制矩形
    auto drawRectangles = [&](const apaSlotListInfo &rectangles, const cv::Scalar &color) {
        // for (const auto &rect : rectangles.WorldoutRect) {
        for (const auto &rect : rectangles.slots_in_cur_frame) {
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
            std::vector<std::string> labels = {"A", "B", "C", "D"};
            for (int i = 0; i < 4; ++i) {
                cv::Point scaledPoint(
                    rect.rectInfo.pt[i].x * scale + translate_x,
                    rect.rectInfo.pt[i].y * scale + translate_y
                );
                scaledPoints.push_back(scaledPoint);
                // 在顶点处标注字母
                cv::putText(image, labels[i], scaledPoint, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
            }

            // 绘制四边形
            cv::polylines(image, scaledPoints, true, color, 1);

            // 占用显示
            if (rect.rectInfo.iSodType == 1) {
                std::vector<std::vector<cv::Point>> contours = {scaledPoints};
                cv::fillPoly(image, contours, cv::Scalar(193, 182, 255));
            }

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

            // 定义圆形的半径
            int radius = 30; // 根据需要调整半径大小，单位像素

            pose.x = 0;
            pose.y = 0;

            // 计算车辆中心点在图像上的位置
            cv::Point vehicle_pose_pt(
                static_cast<int>(pose.x * scale + translate_x),
                static_cast<int>(pose.y * scale + translate_y)
            );

            // 将 yaw 角度从度转换为弧度
            float yaw_rad = pose.canAng * M_PI / 180.0f - M_PI_2;

            // 绘制车辆的圆形表示
            cv::circle(image, vehicle_pose_pt, radius, color, cv::FILLED);

            // 可选：绘制圆形边框
            cv::circle(image, vehicle_pose_pt, radius, cv::Scalar(0, 0, 0), 2);

            // 绘制车辆的 yaw 角作为箭头
            float arrow_length = 70.0f; // 箭头长度，单位像素

            // 根据 yaw 角计算箭头终点
            // 注意：在图像坐标系中，Y 轴向下，因此 sin(yaw_rad) 应该取负
            cv::Point arrow_end_pt(
                static_cast<int>(vehicle_pose_pt.x + arrow_length * std::cos(yaw_rad)),
                static_cast<int>(vehicle_pose_pt.y - arrow_length * std::sin(yaw_rad))
            );

            // 绘制箭头
            cv::arrowedLine(image, vehicle_pose_pt, arrow_end_pt, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.3);

            // 可选：在箭头终点添加 "Yaw" 标签
            cv::putText(image, "Yaw", arrow_end_pt + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
                }
            };

            // 绘制矩形为红色
            drawRectangles(rectanglesA, cv::Scalar(0, 0, 255));

            // 保存图像
            cv::imshow("image", image);
            cv::waitKey(1000);
            cv::imwrite(filename, image);
            std::cout << "Image saved to: " << filename << std::endl;
}







// // Draw apaSlotListInfo
// void drawapaSlotlistinfoToJPG(const std::string &filename,apaSlotListInfo &rectanglesA, Loc::App2emap_DR pose, Fus::PkEmapObs obs) {
//     // 创建空白图像
//     cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

//     std::string timestampText = "(RD)timestamp " + std::to_string(rectanglesA.ullFrameId);
//     cv::putText(image, timestampText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    
//     // 计算输入的全局坐标范围
//     int minX = std::numeric_limits<int>::max();
//     int maxX = std::numeric_limits<int>::lowest();
//     int minY = std::numeric_limits<int>::max();
//     int maxY = std::numeric_limits<int>::lowest();

//     auto updateBounds = [&](const apaSlotListInfo &rectangles) {
//         // for (const auto &rect : rectangles.WorldoutRect) {
//         for (const auto &rect : rectangles.slots_in_cur_frame) {
//             for (const auto &pt : rect.rectInfo.pt) {
//                 minX = std::min(minX, pt.x);
//                 maxX = std::max(maxX, pt.x);
//                 minY = std::min(minY, pt.y);
//                 maxY = std::max(maxY, pt.y);
//             }
//         }
//     };

//     // 更新范围
//     updateBounds(rectanglesA);

//     // 图像中心点
//     double center_img_x = image.cols / 2.0;
//     double center_img_y = image.rows / 2.0;

//     // 矩形的中心点
//     double center_slots_x = (minX + maxX) / 2.0;
//     double center_slots_y = (minY + maxY) / 2.0;

//     // 缩放因子，保持宽高比例并留边距
//     double scale_x = (image.cols * 0.8) / (maxX - minX);  // 留 20% 边距
//     double scale_y = (image.rows * 0.8) / (maxY - minY);
//     double scale = std::min(scale_x, scale_y);

//     // 偏移量（将缩放后的矩形中心移到图像中心）
//     double translate_x = center_img_x - center_slots_x * scale;
//     double translate_y = center_img_y - center_slots_y * scale;

//     // 画障碍物
//     float obs_x, obs_y;
//     for (int i = 0; i < 50; ++i) {
//         obs_x = (obs.pkEmapObs[i].obsCenter.x * 1000);
//         obs_y = (obs.pkEmapObs[i].obsCenter.y * 1000);

//         // 如果坐标为 0，跳过绘制
//         if (obs_x == 0 || obs_y == 0) {
//             continue;
//         }

//         // 应用缩放和平移
//         float scaled_obs_x = obs_x * scale + translate_x;
//         float scaled_obs_y = obs_y * scale + translate_y;

//         // // 绘制障碍物（圆）
//         // cv::circle(image, cv::Point(scaled_obs_x, scaled_obs_y), 10, cv::Scalar(0, 0, 0), -1);

//         // // 准备角点文字内容
//         // std::string text = "(" + std::to_string(static_cast<int>(obs_x)) + ", " +
//         //                 std::to_string(static_cast<int>(obs_y)) + ")";
//         // // 确定文本位置（圆的正上方）
//         // int text_offset_y = 25; // 文本与圆之间的垂直偏移量
//         // cv::Point textPosition(scaled_obs_x, scaled_obs_y - text_offset_y);
//         // // 绘制文本
//         // cv::putText(image, text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 0), 1);

//         // //准备位置文字内容
//         // std::string text_loc;
//         // switch (obs_location){
//         //     case 0:
//         //         text_loc = "NO OBS";
//         //         break;
//         //     case 1:
//         //         text_loc = "near AB";
//         //         break;
//         //     case 2:
//         //         text_loc = "near BC";
//         //         break;
//         //     case 3:
//         //         text_loc = "near CD";
//         //         break;
//         //     case 4:
//         //         text_loc = "near DA";
//         //         break;
//         //     default:
//         //         break;
//         // }
//         // int text_offset_y_loc = 40;
//         // cv::Point locTextPosition(scaled_obs_x, scaled_obs_y - text_offset_y_loc);
//         // cv::putText(image, text_loc, locTextPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 2);

//     }

//     // 应用缩放和平移并绘制矩形
//     auto drawRectangles = [&](const apaSlotListInfo &rectangles, const cv::Scalar &color) {
//         // for (const auto &rect : rectangles.WorldoutRect) {
//         for (const auto &rect : rectangles.slots_in_cur_frame) {
//             cv::Point center;
//             center.x = ((rect.rectInfo.pt[0].x + rect.rectInfo.pt[2].x) / 2.0)* scale + translate_x;
//             center.y = ((rect.rectInfo.pt[0].y + rect.rectInfo.pt[2].y) / 2.0)* scale + translate_y;
//             std::string id_text = "id: " + std::to_string(rect.rectInfo.label);
//             cv::putText(image, id_text, center, cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

//             int width, length;
//             length = CalcDistance(rect.rectInfo.pt[0].x, rect.rectInfo.pt[0].y, rect.rectInfo.pt[1].x, rect.rectInfo.pt[1].y);
//             float width1 = CalcDistance(rect.rectInfo.pt[2].x, rect.rectInfo.pt[2].y, rect.rectInfo.pt[1].x, rect.rectInfo.pt[1].y);
//             float width2 = CalcDistance(rect.rectInfo.pt[0].x, rect.rectInfo.pt[0].y, rect.rectInfo.pt[3].x, rect.rectInfo.pt[3].y);
//             width = (width1+width2)/2.0;
//             std::string lw_text = "W: " + std::to_string(width) + ",L:" + std::to_string(length);
//             cv::putText(image, lw_text, center+cv::Point(0, -10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

//             std::vector<cv::Point> scaledPoints;
//             std::vector<std::string> labels = {"A", "B", "C", "D"};
//             for (int i = 0; i < 4; ++i) {
//                 cv::Point scaledPoint(
//                     rect.rectInfo.pt[i].x * scale + translate_x,
//                     rect.rectInfo.pt[i].y * scale + translate_y
//                 );
//                 scaledPoints.push_back(scaledPoint);
//                 // 在顶点处标注字母
//                 cv::putText(image, labels[i], scaledPoint, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
//             }

//             // 绘制四边形
//             cv::polylines(image, scaledPoints, true, color, 1);
            

//             // 绘制角点坐标
//             for (int i = 0; i < 4; ++i) {
//                 cv::Point scaledPoint = scaledPoints[i];

//                 // 在图像上绘制小圆点表示角点
//                 cv::circle(image, scaledPoint, 3, color, -1);

//                 // 在角点旁边绘制坐标文本
//                 std::string text = "(" + std::to_string(static_cast<int>((scaledPoint.x - translate_x)/scale)) + ", " +
//                                    std::to_string(static_cast<int>((scaledPoint.y - translate_y)/scale )) + ")";
//                 cv::putText(image, text, scaledPoint + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
//             }

//             // 定义圆形的半径
//             int radius = 30; // 根据需要调整半径大小，单位像素
            
//             pose.x = 0;
//             pose.y = 0;

//             // 计算车辆中心点在图像上的位置
//             cv::Point vehicle_pose_pt(
//                 static_cast<int>(pose.x * scale + translate_x),
//                 static_cast<int>(pose.y * scale + translate_y)
//             );

//             // 将 yaw 角度从度转换为弧度
//             float yaw_rad = pose.canAng * M_PI / 180.0f - M_PI_2;

//             // 绘制车辆的圆形表示
//             cv::circle(image, vehicle_pose_pt, radius, color, cv::FILLED);
    
//             // 可选：绘制圆形边框
//             cv::circle(image, vehicle_pose_pt, radius, cv::Scalar(0, 0, 0), 2);

//             // 绘制车辆的 yaw 角作为箭头
//             float arrow_length = 70.0f; // 箭头长度，单位像素

//             // 根据 yaw 角计算箭头终点
//             // 注意：在图像坐标系中，Y 轴向下，因此 sin(yaw_rad) 应该取负
//             cv::Point arrow_end_pt(
//                 static_cast<int>(vehicle_pose_pt.x + arrow_length * std::cos(yaw_rad)),
//                 static_cast<int>(vehicle_pose_pt.y - arrow_length * std::sin(yaw_rad))
//             );

//             // 绘制箭头
//             cv::arrowedLine(image, vehicle_pose_pt, arrow_end_pt, cv::Scalar(255, 0, 0), 2, cv::LINE_AA, 0, 0.3);

//             // 可选：在箭头终点添加 "Yaw" 标签
//             cv::putText(image, "Yaw", arrow_end_pt + cv::Point(5, -5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 0, 0), 1);
//                 }
//             };

//             // 绘制矩形为红色
//             drawRectangles(rectanglesA, cv::Scalar(0, 0, 255));

//             // 保存图像
//             cv::imwrite(filename, image);
//             std::cout << "Image saved to: " << filename << std::endl;
// }

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
    pose.x = 0;
    pose.y = 0;
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

void drawVCUslotToJPG(const std::string &filename, const Sfus::FusionSlotInfovector &data, Loc::App2emap_DR pose){
    // 创建空白图像
    cv::Mat image(800, 800, CV_8UC3, cv::Scalar(255, 255, 255));

    std::string timestampText = "(VCU)timestamp " + std::to_string(0);
    cv::putText(image, timestampText, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 0), 2);

    // 计算输入的全局坐标范围
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

     // Lambda 函数用于更新坐标范围
    auto updateBounds = [&](const Sfus::FusionSlotInfovector &info) {
        for (const auto &pt : info.FusionSlotInfo[0].pt) {
                    minX = std::min(minX, static_cast<float>(pt.x));
                    maxX = std::max(maxX, static_cast<float>(pt.x));
                    minY = std::min(minY, static_cast<float>(pt.y));
                    maxY = std::max(maxY, static_cast<float>(pt.y));
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
    double translate_x = center_img_x;
    double translate_y = center_img_y;

    // Lambda 函数用于绘制处理前的超声车位矩形和相关信息
    auto drawRectangles = [&](const Sfus::FusionSlotInfovector &info, const cv::Scalar &color) {
        // for (const auto &slotInfo : info.FusionSlotInfo) {
                // 计算矩形中心点
                float center_x = (info.FusionSlotInfo[0].pt[0].x + info.FusionSlotInfo[0].pt[2].x) / 2.0f;
                float center_y = (info.FusionSlotInfo[0].pt[0].y + info.FusionSlotInfo[0].pt[2].y) / 2.0f;
                cv::Point center(
                    static_cast<int>(center_x * scale + translate_x),
                    static_cast<int>(center_y * scale + translate_y)
                );

                // 计算长度和宽度
                float length = CalcDistance(info.FusionSlotInfo[0].pt[0].x, info.FusionSlotInfo[0].pt[0].y, info.FusionSlotInfo[0].pt[1].x, info.FusionSlotInfo[0].pt[1].y);
                float width1 = CalcDistance(info.FusionSlotInfo[0].pt[2].x, info.FusionSlotInfo[0].pt[2].y, info.FusionSlotInfo[0].pt[1].x, info.FusionSlotInfo[0].pt[1].y);
                float width2 = CalcDistance(info.FusionSlotInfo[0].pt[0].x, info.FusionSlotInfo[0].pt[0].y, info.FusionSlotInfo[0].pt[3].x, info.FusionSlotInfo[0].pt[3].y);
                float width = (width1 + width2) / 2.0f;

                // 添加尺寸文本
                std::string lw_text = "W: " + std::to_string((int)width) + ", L: " + std::to_string((int)length);
                cv::putText(image, lw_text, center + cv::Point(0, -10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

                // 缩放和平移点
                std::vector<cv::Point> scaledPoints;
                for (int i = 0; i < 4; ++i) {
                    cv::Point scaledPoint(
                        static_cast<int>(info.FusionSlotInfo[0].pt[i].x * scale + translate_x),
                        static_cast<int>(info.FusionSlotInfo[0].pt[i].y * scale + translate_y)
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
            // }
    };

    // 调用绘制函数，使用红色
    drawRectangles(data, cv::Scalar(0, 0, 255));
    // 绘制车辆位姿
    int radius = 10; // 根据需要调整半径大小，单位像素
    pose.x = 0.0;
    pose.y = 0.0;
    cv::Point vehicle_pose(
        static_cast<int>(pose.x * scale + translate_x),
        static_cast<int>(pose.y * scale + translate_y)
    );
    // 绘制车辆的圆形表示
    cv::circle(image, vehicle_pose, radius, cv::Scalar(0, 0, 255), cv::FILLED);
    
    // 可选：绘制圆形边框
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

inline static void Slot2Global(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw)
{
    float theta = yaw * acos(-1) / 180.;
    
    float slot_Apt_temp_x = slot.targetSlot.slotCorners.cornerA.x * cos(theta) + slot.targetSlot.slotCorners.cornerA.y * sin(theta) + x;
    float slot_Apt_temp_y = slot.targetSlot.slotCorners.cornerA.y * cos(theta) - slot.targetSlot.slotCorners.cornerA.x * sin(theta) + y;
    
    float slot_Bpt_temp_x = slot.targetSlot.slotCorners.cornerB.x * cos(theta) + slot.targetSlot.slotCorners.cornerB.y * sin(theta) + x;
    float slot_Bpt_temp_y = slot.targetSlot.slotCorners.cornerB.y * cos(theta) - slot.targetSlot.slotCorners.cornerB.x * sin(theta) + y;

    float slot_Cpt_temp_x = slot.targetSlot.slotCorners.cornerC.x * cos(theta) + slot.targetSlot.slotCorners.cornerC.y * sin(theta) + x;
    float slot_Cpt_temp_y = slot.targetSlot.slotCorners.cornerC.y * cos(theta) - slot.targetSlot.slotCorners.cornerC.x * sin(theta) + y;

    float slot_Dpt_temp_x = slot.targetSlot.slotCorners.cornerD.x * cos(theta) + slot.targetSlot.slotCorners.cornerD.y * sin(theta) + x;
    float slot_Dpt_temp_y = slot.targetSlot.slotCorners.cornerD.y * cos(theta) - slot.targetSlot.slotCorners.cornerD.x * sin(theta) + y;
    
    slot.targetSlot.slotCorners.cornerA.x = slot_Apt_temp_x;
    slot.targetSlot.slotCorners.cornerA.y = slot_Apt_temp_y;

    slot.targetSlot.slotCorners.cornerB.x = slot_Bpt_temp_x;
    slot.targetSlot.slotCorners.cornerB.y = slot_Bpt_temp_y;

    slot.targetSlot.slotCorners.cornerC.x = slot_Cpt_temp_x;
    slot.targetSlot.slotCorners.cornerC.y = slot_Cpt_temp_y;
    
    slot.targetSlot.slotCorners.cornerD.x = slot_Dpt_temp_x;
    slot.targetSlot.slotCorners.cornerD.y = slot_Dpt_temp_y;
}

inline static void Slot2Local(Sfus::Sfsuion2DecPlan &slot, const float &x, const float &y, const float &yaw)
{
    float theta = yaw * static_cast<float>(M_PI) / 180.0f;

    float tmp_A_x = slot.targetSlot.slotCorners.cornerA.x - x;
    float tmp_A_y = slot.targetSlot.slotCorners.cornerA.y - y;
    float tmp_B_x = slot.targetSlot.slotCorners.cornerB.x - x;
    float tmp_B_y = slot.targetSlot.slotCorners.cornerB.y - y;
    float tmp_C_x = slot.targetSlot.slotCorners.cornerC.x - x;
    float tmp_C_y = slot.targetSlot.slotCorners.cornerC.y - y;
    float tmp_D_x = slot.targetSlot.slotCorners.cornerD.x - x;
    float tmp_D_y = slot.targetSlot.slotCorners.cornerD.y - y;
    
    float slot_Apt_temp_x = tmp_A_x * cos(theta) - tmp_A_y * sin(theta);
    float slot_Apt_temp_y = tmp_A_x * sin(theta) + tmp_A_y * cos(theta);
    
    float slot_Bpt_temp_x = tmp_B_x * cos(theta) - tmp_B_y * sin(theta);
    float slot_Bpt_temp_y = tmp_B_x * sin(theta) + tmp_B_y * cos(theta);

    float slot_Cpt_temp_x = tmp_C_x * cos(theta) - tmp_C_y * sin(theta);
    float slot_Cpt_temp_y = tmp_C_x * sin(theta) + tmp_C_y * cos(theta);

    float slot_Dpt_temp_x = tmp_D_x * cos(theta) - tmp_D_y * sin(theta);
    float slot_Dpt_temp_y = tmp_D_x * sin(theta) + tmp_D_y * cos(theta);
    
    slot.targetSlot.slotCorners.cornerA.x = slot_Apt_temp_x;
    slot.targetSlot.slotCorners.cornerA.y = slot_Apt_temp_y;

    slot.targetSlot.slotCorners.cornerB.x = slot_Bpt_temp_x;
    slot.targetSlot.slotCorners.cornerB.y = slot_Bpt_temp_y;

    slot.targetSlot.slotCorners.cornerC.x = slot_Cpt_temp_x;
    slot.targetSlot.slotCorners.cornerC.y = slot_Cpt_temp_y;
    
    slot.targetSlot.slotCorners.cornerD.x = slot_Dpt_temp_x;
    slot.targetSlot.slotCorners.cornerD.y = slot_Dpt_temp_y;
}

float degreesToRadians(float degrees){
    return degrees * static_cast<float>(M_PI) / 180.0f;
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
    // drawRDslotToJPG(jpg, rd_info);
    
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



    if (rd_info.frameTimeStampNs > 1551331160812421000){
        currentIndex++;
        return;
    }

    if (std::llabs(rd_info.frameTimeStampNs - dr_pose.timeStamp) > 1500) 
    {
        printf("[TIMESYNC]RD timestamp: %llu, DR timestamp: %llu, Time synchronization not met, skipping execution.",rd_info.frameTimeStampNs, dr_pose.timeStamp);
        currentDRIndex++;
        return;
    }
    else if (std::llabs(dr_pose.timeStamp - rd_info.frameTimeStampNs) > 1500)
    {
        printf("[TIMESYNC]RD timestamp: %llu, DR timestamp: %llu, Time synchronization not met, skipping execution.",rd_info.frameTimeStampNs, dr_pose.timeStamp);
        currentIndex++;
        return;
    }
    printf("[TIMESYNC]RD timestamp: %llu, DR timestamp: %llu, Time synchronization achieved!",rd_info.frameTimeStampNs, dr_pose.timeStamp);

    

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
            if (parkingSlot.tl.x < 448 && parkingSlot.tr.x < 448)
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

    pose_globaldata.coord.x = int(dr_pose.x);
    pose_globaldata.coord.y = int(dr_pose.y);
    pose_globaldata.yaw = dr_pose.canAng;
    std::cout<<"timestamp:"<<dr_pose.timeStamp<<std::endl;
    std::cout<<"coord.x:"<<pose_globaldata.coord.x<<std::endl;
    std::cout<<"coord.y:"<<pose_globaldata.coord.y<<std::endl;
    std::cout<<"coord.yaw:"<<pose_globaldata.yaw<<std::endl;
    PSD_FusionModuleIFrunable.UpdateVechiclePose(pose_globaldata);
    PSD_FusionModuleIFrunable.UpdateVisionSlots(singleframeslotsID, singleframeslots, 2, 0);
    
    apaSlotListInfo outputSlot_CALVIS = PSD_FusionModuleIFrunable.GetOutputSlot();
    std::cout<<"Update vision slot size: "<< outputSlot_CALVIS.WorldoutRect.size()<<std::endl;
    // drawapaSlotlistinfoToJPG("Cal Vision slot.jpg",outputSlot_CALVIS, dr_pose);

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

    // ********************************Calculate OBS old
    // float stop_dis = 0.0;
    // int obs_location = -1;
    // PSD_FusionModuleIFrunable.CalStopDisAndLoc(obs_info, stop_dis, obs_location);
    // std::cout<<"Stop distance:"<<stop_dis<<std::endl;
    // std::cout<<"Obs location::"<<obs_location<<std::endl;


    // ********************************Calculate OBS new
    PSD_FusionModuleIFrunable.CalStopDisAndLoc(obs_info);

    // *******************************Load original USS data 
    // Get USS info
    if (!uss_dataloaded){
        std::string filepath = (parentPath / "USSapaSlotListInfo.json").string();    
        loadAllData(filepath, allUSSData);
        uss_dataloaded = true;
    }
    static size_t currentUSSIndex = 0;
    if(currentUSSIndex >= allUSSData.size()){
        // std::cout<<"所有数据已处理完毕"<<std::endl;
        return;
    }
    const auto &uss_data = allUSSData[currentUSSIndex];
    UssIf_stPLVOutputInfo_t uss_info;
    uss_info.enmPLVActiveSts =  static_cast<UssIf_enmActiveSts_t>((int)uss_data["enmPLVActiveSts"]);
    // 解析 UssIf_stSlotInfo 数组
    const auto& slotInfoArray = uss_data["UssIf_stSlotInfo"];
    size_t slot_info_size = sizeof(uss_info.UssIf_stSlotInfo)/sizeof(uss_info.UssIf_stSlotInfo[0]);
    // 注意：UssIf_stSlotInfo是std::array大小为4，如果JSON中超过4个只取前4个，少于4个则只解析那么多。
    for (size_t i = 0; i < slot_info_size && i < slotInfoArray.size(); ++i) {
        const auto& slotInfoJson = slotInfoArray[i];
        UssIf_stSlotInfo_t uss_slot_info;
        uss_slot_info.u8SlotNum = slotInfoJson["u8SlotNum"];

        const auto& slotPropArray = slotInfoJson["UssIf_stSlotProperty"];
        // UssIf_stSlotProperty为std::array大小为3，同理进行安全访问
        size_t slot_prop_size = sizeof(uss_slot_info.UssIf_stSlotProperty)/sizeof(uss_slot_info.UssIf_stSlotProperty[0]);
        for (size_t p = 0; p < slot_prop_size && p < slotPropArray.size(); ++p) {
            const auto& slotPropJson = slotPropArray[p];
            UssIf_stSlotProperty_t uss_slot_prop;
            uss_slot_prop.u16SlotID = slotPropJson["u16SlotID"];
            uss_slot_prop.enmSlotType = static_cast<UssIf_enmSlotType_t>((int)slotPropJson["enmSlotType"]);
            uss_slot_prop.enmSlotBottomType = static_cast<UssIf_enmSlotBottomType_t>((int)slotPropJson["enmSlotBottomType"]);
            uss_slot_prop.u16SlotLength = slotPropJson["u16SlotLength"];
            uss_slot_prop.u16SlotDepth = slotPropJson["u16SlotDepth"];
            uss_slot_prop.u64Timestamp = slotPropJson["u64Timestamp"];

            // 解析 stSlotPt (4个点)
            const auto& slotPtArray = slotPropJson["stSlotPt"];
            size_t slot_pt_size = sizeof(uss_slot_prop.stSlotPt)/sizeof(uss_slot_prop.stSlotPt[0]);
            for (size_t ptIdx = 0; ptIdx < slot_pt_size && ptIdx < slotPtArray.size(); ++ptIdx) {
                uss_slot_prop.stSlotPt[ptIdx].x = slotPtArray[ptIdx]["x"];
                uss_slot_prop.stSlotPt[ptIdx].y = slotPtArray[ptIdx]["y"];
            }

            // // 如果有 stInSlotObstaclePt
            // if (slotPropJson.contains("stInSlotObstaclePt")) {
            //     const auto& obsPtArray = slotPropJson["stInSlotObstaclePt"];
            //     for (size_t obsIdx = 0; obsIdx < uss_slot_prop.stInSlotObstaclePt.size() && obsIdx < obsPtArray.size(); ++obsIdx) {
            //         uss_slot_prop.stInSlotObstaclePt[obsIdx].x = obsPtArray[obsIdx]["x"];
            //         uss_slot_prop.stInSlotObstaclePt[obsIdx].y = obsPtArray[obsIdx]["y"];
            //     }
            // }

            // 解析剩余字段
            if (slotPropJson.contains("u16UssOppositeSpace")) uss_slot_prop.u16UssOppositeSpace = slotPropJson["u16UssOppositeSpace"];
            if (slotPropJson.contains("u16UssTransverseSpace")) uss_slot_prop.u16UssTransverseSpace = slotPropJson["u16UssTransverseSpace"];
            if (slotPropJson.contains("u16ObjDistanceBetweenLineABToSlotBottom")) uss_slot_prop.u16ObjDistanceBetweenLineABToSlotBottom = slotPropJson["u16ObjDistanceBetweenLineABToSlotBottom"];
            if (slotPropJson.contains("enmDownSlotSODType")) uss_slot_prop.enmDownSlotSODType = static_cast<UssIf_enmDownSlotSODType_t>((int)slotPropJson["enmDownSlotSODType"]);

            uss_slot_info.UssIf_stSlotProperty[p] = uss_slot_prop;
        }

        uss_info.UssIf_stSlotInfo[i] = uss_slot_info;
    }

    auto pre_post_uss_slots = uss_info;
    slotfusion sf;
    apaSlotListInfo outputSlotUSS;
    
    uint32_t time1 = uss_info.UssIf_stSlotInfo[0].UssIf_stSlotProperty[0].u64Timestamp;

    sf.fillVisonstruct(uss_info, outputSlotUSS);
    sf.postprocessUSSslots(uss_info);
    // std::string uss_jpg = "USS_" + std::to_string(uss_info.UssIf_stSlotInfo->UssIf_stSlotProperty[0].u64Timestamp) + ".jpg";
    int index = 0;
    std::string uss_jpg = "USS_" + std::to_string(index) + ".jpg";
    // index++;
    // drawUSSslotToJPG(uss_jpg, pre_post_uss_slots, dr_pose, uss_info);

    apaSlotListInfo outputSlot_FUSED;
    
    sf.mergeSlotLists(outputSlotUSS, outputSlot_CALVIS, outputSlot_FUSED);
    drawapaSlotlistinfoToJPG("Cal Vision slot.jpg",outputSlot_FUSED, dr_pose, obs_info,singleframeslots);
    

    // test VCU 
    Sfus::FusionSlotInfovector vcu_slot;
    Sfus::Sfsuion2DecPlan target_slot;

    // 360 degree
    // vcu_slot.FusionSlotInfo[0].pt[0].x = -1.526200;
    // vcu_slot.FusionSlotInfo[0].pt[0].y = 1.952000;
    // vcu_slot.FusionSlotInfo[0].pt[1].x = -1.421200;
    // vcu_slot.FusionSlotInfo[0].pt[1].y = 7.711000;
    // vcu_slot.FusionSlotInfo[0].pt[2].x = -3.794200;
    // vcu_slot.FusionSlotInfo[0].pt[2].y = 7.760000;
    // vcu_slot.FusionSlotInfo[0].pt[3].x = -3.899200;
    // vcu_slot.FusionSlotInfo[0].pt[3].y = 2.001000;
    // target_slot.targetSlot.slotCorners.cornerA.x = 2001.0;
    // target_slot.targetSlot.slotCorners.cornerA.y = 224.0;
    // target_slot.targetSlot.slotCorners.cornerB.x = 1952.0;
    // target_slot.targetSlot.slotCorners.cornerB.y = 2597.0;
    // target_slot.targetSlot.slotCorners.cornerC.x = 7711.0;
    // target_slot.targetSlot.slotCorners.cornerC.y = 2702.0;
    // target_slot.targetSlot.slotCorners.cornerD.x = 7760.0;
    // target_slot.targetSlot.slotCorners.cornerD.y = 329.0;
    // int dr_cul_x = 251;
    // int dr_cul_y = -8438;
    // float dr_cul_theta = 4.778870;

    // 180 degree
    // vcu_slot.FusionSlotInfo[0].pt[0].x = -0.428200;
    // vcu_slot.FusionSlotInfo[0].pt[0].y = -2.104000;
    // vcu_slot.FusionSlotInfo[0].pt[1].x = -2.700200;
    // vcu_slot.FusionSlotInfo[0].pt[1].y = -2.013000;
    // vcu_slot.FusionSlotInfo[0].pt[2].x = -2.902200;
    // vcu_slot.FusionSlotInfo[0].pt[2].y = -7.738000;
    // vcu_slot.FusionSlotInfo[0].pt[3].x = -0.630200;
    // vcu_slot.FusionSlotInfo[0].pt[3].y = -7.829000;
    // target_slot.targetSlot.slotCorners.cornerA.x = -2013.0;
    // target_slot.targetSlot.slotCorners.cornerA.y = 1423.0;
    // target_slot.targetSlot.slotCorners.cornerB.x = -2104.0;
    // target_slot.targetSlot.slotCorners.cornerB.y = 3695.0;
    // target_slot.targetSlot.slotCorners.cornerC.x = -7829.0;
    // target_slot.targetSlot.slotCorners.cornerC.y = 3493.0;
    // target_slot.targetSlot.slotCorners.cornerD.x = -7738.0;
    // target_slot.targetSlot.slotCorners.cornerD.y = 1221.0;
    // int dr_cul_x = -16413;
    // int dr_cul_y = 38608;
    // float dr_cul_theta = -177.667236;

    // 270 degree
    // vcu_slot.FusionSlotInfo[0].pt[0].x = -1.919200;
    // vcu_slot.FusionSlotInfo[0].pt[0].y = 1.764000;
    // vcu_slot.FusionSlotInfo[0].pt[1].x = -2.097200;
    // vcu_slot.FusionSlotInfo[0].pt[1].y = 7.534000;
    // vcu_slot.FusionSlotInfo[0].pt[2].x = -4.541200;
    // vcu_slot.FusionSlotInfo[0].pt[2].y = 7.472000;
    // vcu_slot.FusionSlotInfo[0].pt[3].x = -4.363200;
    // vcu_slot.FusionSlotInfo[0].pt[3].y = 1.702000;
    // target_slot.targetSlot.slotCorners.cornerA.x = 1702.0;
    // target_slot.targetSlot.slotCorners.cornerA.y = -240.0;
    // target_slot.targetSlot.slotCorners.cornerB.x = 1764.0;
    // target_slot.targetSlot.slotCorners.cornerB.y = 2204.0;
    // target_slot.targetSlot.slotCorners.cornerC.x = 7534.0;
    // target_slot.targetSlot.slotCorners.cornerC.y = 2026.0;
    // target_slot.targetSlot.slotCorners.cornerD.x = 7472.0;
    // target_slot.targetSlot.slotCorners.cornerD.y = -418.0;
    // int dr_cul_x = 49064;
    // int dr_cul_y = -3957;
    // float dr_cul_theta = -86.83;

    // if (dr_pose.timeStamp > 1736143069604){   // 360 degree
    // if (dr_pose.timeStamp > 1736132099769){  //180 degree      
    // // if (dr_pose.timeStamp > 1736227520046){   //270 degree 
    //     Slot2Global(target_slot, dr_cul_x, dr_cul_y, dr_cul_theta);
    //     Slot2Local(target_slot, dr_pose.x, dr_pose.y, dr_pose.canAng);
    // }
          
    
    // if (target_slot.targetSlot.slotCorners.cornerA.x <= 0 ||  target_slot.targetSlot.slotCorners.cornerB.x <= 0){
    //     vcu_slot.FusionSlotInfo[0].pt[0].x = (target_slot.targetSlot.slotCorners.cornerB.y) / 1000.0;
    //     vcu_slot.FusionSlotInfo[0].pt[0].y = target_slot.targetSlot.slotCorners.cornerB.x / 1000.0;

    //     vcu_slot.FusionSlotInfo[0].pt[1].x = (target_slot.targetSlot.slotCorners.cornerA.y) / 1000.0;
    //     vcu_slot.FusionSlotInfo[0].pt[1].y = target_slot.targetSlot.slotCorners.cornerA.x / 1000.0;
        
    //     vcu_slot.FusionSlotInfo[0].pt[2].x = (target_slot.targetSlot.slotCorners.cornerD.y) / 1000.0;
    //     vcu_slot.FusionSlotInfo[0].pt[2].y = target_slot.targetSlot.slotCorners.cornerD.x / 1000.0;
        
    //     vcu_slot.FusionSlotInfo[0].pt[3].x = (target_slot.targetSlot.slotCorners.cornerC.y) / 1000.0;
    //     vcu_slot.FusionSlotInfo[0].pt[3].y = target_slot.targetSlot.slotCorners.cornerC.x / 1000.0;
    // }else{
    //     vcu_slot.FusionSlotInfo[0].pt[0].x = (target_slot.targetSlot.slotCorners.cornerB.y) / 1000.0;
    //     vcu_slot.FusionSlotInfo[0].pt[0].y = target_slot.targetSlot.slotCorners.cornerB.x / 1000.0;

    //     vcu_slot.FusionSlotInfo[0].pt[1].x = (target_slot.targetSlot.slotCorners.cornerC.y) / 1000.0;
    //     vcu_slot.FusionSlotInfo[0].pt[1].y = target_slot.targetSlot.slotCorners.cornerC.x / 1000.0;
        
    //     vcu_slot.FusionSlotInfo[0].pt[2].x = (target_slot.targetSlot.slotCorners.cornerD.y) / 1000.0;
    //     vcu_slot.FusionSlotInfo[0].pt[2].y = target_slot.targetSlot.slotCorners.cornerD.x / 1000.0;
        
    //     vcu_slot.FusionSlotInfo[0].pt[3].x = (target_slot.targetSlot.slotCorners.cornerA.y) / 1000.0;
    //     vcu_slot.FusionSlotInfo[0].pt[3].y = target_slot.targetSlot.slotCorners.cornerA.x / 1000.0;
    // }
    // drawVCUslotToJPG("VCU slot.jpg", vcu_slot, dr_pose);

    currentIndex++;
    currentDRIndex++;
    // currentVisionIndex++;
    currentUSSIndex++;
    currentOBSIndex++;
}

int main(int argc, char **argv) {
    while (true) {
        auto start = std::chrono::steady_clock::now();

        TimeTrigger_Timer50();  // 调用您的函数

        // 计算已经消耗的时间
        auto end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout<<"Algo time is:"<< elapsed.count() <<std::endl;
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
