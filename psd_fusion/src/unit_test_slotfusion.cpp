#include <vector>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <gtest/gtest.h>
#include "fusion.h" // 替换为你的实际头文件

#define TESTCASE 0
slotfusion slotfusion;

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


// 测试用例：测试 mergeSlotLists 函数
TEST(MergeSlotListsTest, HandlesOverlapAndFusion) {
    // 创建 USS 和 VIS 的测试数据
    apaSlotListInfo outputSlot_USS;
    apaSlotListInfo outputSlot_VIS;
    apaSlotListInfo outputSlot_FUSION;

    // USS 的车位列表
    if (TESTCASE == 0){
        outputSlot_USS.WorldoutRect.push_back({createRect(2263, -11523, 2263, -8067, 8124, -8067,8123, -11523)}); // 车位1
        outputSlot_USS.WorldoutRect.push_back({createRect(2257, -5615, 2110, -2378, 7971, -2378, 8116, -5615)}); // 车位2
        outputSlot_USS.WorldoutRect.push_back({createRect(2092, -90, 2091, 3909, 7950, 3909, 7950, -90)});
    }else if (TESTCASE == 1){
        outputSlot_USS.WorldoutRect.push_back({createRect(2244, -13365,2122, -10030, 7981, -10030, 8103, -13365)});
    }else if (TESTCASE == 2){
        outputSlot_USS.WorldoutRect.push_back({createRect(2237, 5952, 2075, 9633, 7934, 9633, 8097, 5952)}); // 车位1
        outputSlot_USS.WorldoutRect.push_back({createRect(2075, 11317, 2075, 14347, 7935, 14347, 7934, 11317)}); 
    }else if(TESTCASE == 3){
        outputSlot_USS.WorldoutRect.push_back({createRect(2216,-5399, 2064,-2030, 7924,-2030, 8076,-5399)}); 
    }
    
    // VIS 的车位列表
    if (TESTCASE == 0){
        outputSlot_VIS.WorldoutRect.push_back({createRect(1975, -2424, 1875 ,220, 7600, 622, 7667, -2022)}); 
        outputSlot_VIS.WorldoutRect.push_back({createRect(1808, 2966, 1774, 5578, 7466, 5511, 7500, 2866)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(1800, 220, 1875, 2966, 7500, 3033, 7600, 287)});
    }else if(TESTCASE == 1){
        outputSlot_VIS.WorldoutRect.push_back({createRect(1607.0, 3837.0, 1573.0, 6448.0,7265.0, 6180.0, 7299.0, 3535.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(1640.0, 1024.0, 1607.0, 3837.0,7332.0, 3837.0, 7366.0, 1024.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(1741.0, -1620.0, 1640.0, 1057.0, 7366.0, 1325.0, 7433.0, -1352.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(1808.0, -3596.0, 1741.0, -850.0, 7433.0, -281.0, 7533.0, -2993.0)}); 
        outputSlot_VIS.WorldoutRect.push_back({createRect(1908.0, -3596.0, 1841.0, -850.0, 7533.0, -381.0, 7600.0, -3127.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(1975.0, -3462.0, 1908.0, -817.0, 7600.0, -214.0, 7667.0, -2892.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(2075.0, -3395.0, 1941.0, -716.0, 7667.0, -716.0, 7801.0, -3395.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(2142.0, -3328.0, 2075.0, -649.0, 7767.0, -113.0, 7834.0, -2792.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(2209.0, -2524.0, 2142.0, 20.0, 7834.0, 488.0, 7901.0, -2089.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(2310.0, -3361.0, 2209.0, -783.0, 7901.0, -247.0, 8002.0, -2859.0)});
    }else if(TESTCASE == 2){
        outputSlot_VIS.WorldoutRect.push_back({createRect(2310.0, -2290.0 , 2209.0, 287.0, 7935.0, 689.0, 8035.0, -1888.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(2142.0, 3033.0 , 2075.0, 5712.0, 7767.0, 5209.0, 7834.0, 2564.0)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(2209.0, 287.0, 2142.0, 3033.0, 7834.0, 3133.0, 7935.0, 388.0)});
    }else if(TESTCASE == 3){
        outputSlot_VIS.WorldoutRect.push_back({createRect(1941, -2189 , 1875, 488, 7566, 823, 7633, -1854)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(1875, 488 , 1741, 3200, 7466, 3301, 7566, 555)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(-4486, -3328 , -4553, -46, -10178, -817, -10145, -4098)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(1741, 3200 , 1741, 5845, 7433, 5678, 7466, 3033)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(-4553, -46 , -4620, 3100, -10279, 2598, -10212, -582)});
        outputSlot_VIS.WorldoutRect.push_back({createRect(-4620, 3100 , -4687, 6281, -10379, 6147, -10279, 2999)});
    }
    
    
    
    
    // outputSlot_VIS.WorldoutRect.push_back({createRect(-4654,3502,-4754,6649,-10412,6515,-10345,3368)});
    // outputSlot_VIS.WorldoutRect.push_back({createRect(1875,-1888, 1808,790, 7500,1091, 7600,-1587)});
    
    // outputSlot_VIS.WorldoutRect.push_back({createRect(-4419,10663,-4520,13877,-10145,13040,-10078,9792)});
    // outputSlot_VIS.WorldoutRect.push_back({createRect(2343,11858,1875,14436,7600,14939,8069,12394)});

    // 调用 mergeSlotLists
    slotfusion.mergeSlotLists(outputSlot_USS, outputSlot_VIS, outputSlot_FUSION);

    // 输出到 SVG 文件
    drawRectanglesToJPG("slots_vis& slots_uss.jpg", outputSlot_USS.WorldoutRect, outputSlot_VIS.WorldoutRect);
    drawsingleRectanglesToJPG("slot_fusion.jpg",outputSlot_FUSION.WorldoutRect);
    // 验证融合后的车位数量是否正确
    EXPECT_EQ(outputSlot_FUSION.WorldoutRect.size(), 5); // 应该有3个车位

    // 验证具体车位
    // EXPECT_EQ(outputSlot_FUSION.WorldoutRect[0].rectInfo.pt[0].x, 5);  // 验证 VIS 的第一个车位
    // EXPECT_EQ(outputSlot_FUSION.WorldoutRect[1].rectInfo.pt[0].x, 50); // 验证 VIS 的第二个车位
    // EXPECT_EQ(outputSlot_FUSION.WorldoutRect[2].rectInfo.pt[0].x, 0);  // 验证 USS 的独立车位
}

// 测试用例：测试完全没有重叠
// TEST(MergeSlotListsTest, HandlesNoOverlap) {
//     apaSlotListInfo outputSlot_USS;
//     apaSlotListInfo outputSlot_VIS;
//     apaSlotListInfo outputSlot_FUSION;

//     // USS 的车位列表
//     outputSlot_USS.WorldoutRect.push_back({createRect(0, 0, 10, 10)});

//     // VIS 的车位列表
//     outputSlot_VIS.WorldoutRect.push_back({createRect(20, 20, 30, 30)});

//     // 调用 mergeSlotLists
//     slotfusion.mergeSlotLists(outputSlot_USS, outputSlot_VIS, outputSlot_FUSION);

//     // 验证融合后的车位数量是否正确
//     EXPECT_EQ(outputSlot_FUSION.WorldoutRect.size(), 2); // 应该有2个车位
// }

// 测试用例：测试完全重叠
// TEST(MergeSlotListsTest, HandlesFullOverlap) {
//     apaSlotListInfo outputSlot_USS;
//     apaSlotListInfo outputSlot_VIS;
//     apaSlotListInfo outputSlot_FUSION;

//     // USS 的车位列表
//     outputSlot_USS.WorldoutRect.push_back({createRect(0, 0, 10, 10)});

//     // VIS 的车位列表
//     outputSlot_VIS.WorldoutRect.push_back({createRect(0, 0, 10, 10)}); // 完全重叠

//     // 调用 mergeSlotLists
//     slotfusion.mergeSlotLists(outputSlot_USS, outputSlot_VIS, outputSlot_FUSION);

//     // 验证融合后的车位数量是否正确
//     EXPECT_EQ(outputSlot_FUSION.WorldoutRect.size(), 1); // 应该只有1个车位
// }

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
