#include "fusion.h"

using namespace std;

const double IOU_THRESHOLD = 0.6;

slotfusion::slotfusion()
{

}

slotfusion::~slotfusion()
{

}

void slotfusion::mergeSlotLists(const apaSlotListInfo &outputSlot_USS,const apaSlotListInfo &outputSlot_VIS,apaSlotListInfo &outputSlot_FUSION) 
{
    std::cout<<"The Vison slot num is:"<<outputSlot_VIS.slots_in_cur_frame.size()<<std::endl;
    std::cout<<"The USS slot num is:"<<outputSlot_USS.slots_in_cur_frame.size()<<std::endl;
    outputSlot_FUSION = outputSlot_VIS;
    for (const auto &slot_USS : outputSlot_USS.slots_in_cur_frame) 
    {
        bool overlapFound = false; 
        for (const auto &slot_VIS : outputSlot_VIS.slots_in_cur_frame){
            
            double overlap = calculateOverlap(slot_USS.rectInfo, slot_VIS.rectInfo);
            std::cout<<"The overlap is:"<<overlap<<std::endl;
            std::cout<<"VIS slot id is:"<<slot_VIS.rectInfo.label<<std::endl;
            std::cout<<"USS slot id is:"<<slot_USS.rectInfo.label<<std::endl;
            if (overlap > IOU_THRESHOLD){
                overlapFound = true;
                std::cout<<"Found overlap slots"<<std::endl;
                break;
            }
        }

        if (!overlapFound){
            //如果视觉车位和超声车不位重叠，则将USS车位列表加入融合列表内
            outputSlot_FUSION.slots_in_cur_frame.push_back(slot_USS);
            std::cout<<"USS slot not match in vison slot and push bash to Fusion slots!"<<std::endl;
        }
    }
    std::cout<<"The fusion slot num is:"<<outputSlot_FUSION.slots_in_cur_frame.size()<<std::endl;
}

double slotfusion::calculateOverlap(const APA_SPACE::SApaPSRect& rect1, const APA_SPACE::SApaPSRect& rect2) {
    // 计算两个矩形的交集面积
    double intersectionArea = calculateIntersectionArea(rect1, rect2);
    std::cout<<"intersectionArea:"<<intersectionArea<<std::endl;
    // 计算两个矩形的并集面积
    double unionArea = calculateArea(rect1) + calculateArea(rect2) - intersectionArea;
    std::cout<<"area1:"<<calculateArea(rect1)<<std::endl;
    std::cout<<"area2:"<<calculateArea(rect1)<<std::endl;
    std::cout<<"unionArea:"<<unionArea<<std::endl;
    if (unionArea == 0) return 0; // 避免除零错误
    return intersectionArea / unionArea;
}

double slotfusion::calculateIntersectionArea(const APA_SPACE::SApaPSRect& rect1, const APA_SPACE::SApaPSRect& rect2) {
     // 打印输入矩形信息
    std::cout << "Slot1: (" << rect1.pt[0].x << ", " << rect1.pt[0].y << ") , ("
              << rect1.pt[2].x << ", " << rect1.pt[2].y << ")" << std::endl;

    std::cout << "Slot2: (" << rect2.pt[0].x << ", " << rect2.pt[0].y << ") , ("
              << rect2.pt[2].x << ", " << rect2.pt[2].y << ")" << std::endl;

    // 确保矩形合法并规范化
    auto normalizeRect = [](const APA_SPACE::SApaPSRect& rect) {
        double minX = std::min({rect.pt[0].x, rect.pt[1].x, rect.pt[2].x, rect.pt[3].x});
        double maxX = std::max({rect.pt[0].x, rect.pt[1].x, rect.pt[2].x, rect.pt[3].x});
        double minY = std::min({rect.pt[0].y, rect.pt[1].y, rect.pt[2].y, rect.pt[3].y});
        double maxY = std::max({rect.pt[0].y, rect.pt[1].y, rect.pt[2].y, rect.pt[3].y});

        return std::make_tuple(minX, maxX, minY, maxY);
    };

    double minX1, maxX1, minY1, maxY1;
    std::tie(minX1, maxX1, minY1, maxY1) = normalizeRect(rect1);

    double minX2, maxX2, minY2, maxY2;
    std::tie(minX2, maxX2, minY2, maxY2) = normalizeRect(rect2);

    // 计算交集坐标
    double xLeft = std::max(minX1, minX2);
    double yBottom = std::max(minY1, minY2);
    double xRight = std::min(maxX1, maxX2);
    double yTop = std::min(maxY1, maxY2);

    // 打印交集边界
    std::cout << "xLeft: " << xLeft << ", yTop: " << yTop
              << ", xRight: " << xRight << ", yBottom: " << yBottom << std::endl;

    // 如果没有交集，返回0
    if (xRight <= xLeft || yBottom >= yTop) return 0;

    return (xRight - xLeft) * (yTop - yBottom);
}

double slotfusion::calculateArea(const APA_SPACE::SApaPSRect& rect) {
    // 假设车位是矩形，可以直接通过宽度 * 高度计算面积
    int width = abs(rect.pt[2].x - rect.pt[0].x);
    int height = abs(rect.pt[2].y - rect.pt[0].y);
    
    return width * height;
}
