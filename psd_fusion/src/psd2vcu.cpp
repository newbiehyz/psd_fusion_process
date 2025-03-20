#include "psd2vcu.h"

// 滤波参数
const float POSITION_THRESHOLD = 0.2f; // 允许的最大跳动
const float STANDARD_SLOT_WIDTH = 2.5f;  // 车位标准宽度 (米)
const float STANDARD_SLOT_LENGTH = 5.0f; // 车位标准长度 (米)

// 滤波参数
const float POSITION_THRESHOLD_X = 0.5f;  // X 方向跳动阈值
const float POSITION_THRESHOLD_Y = 0.18f; // Y 方向跳动阈值
const float SMOOTHING_FACTOR = 0.6f;      // 平滑权重 (用于大跳动情况)

// 计算欧几里得距离
float computeDistance(const Sfus::SlotPoint& p1, const Sfus::SlotPoint& p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// 对车位进行去噪滤波
void filterSlotPosition(Sfus::FusionSlotInfo& slot, const Sfus::FusionSlotInfo& prevSlot) {
    for (int i = 0; i < 4; i++) {
        float jitter_x = std::abs(slot.pt[i].x - prevSlot.pt[i].x);
        float jitter_y = std::abs(slot.pt[i].y - prevSlot.pt[i].y);

        if (jitter_x < POSITION_THRESHOLD_X) {
            slot.pt[i].x = (slot.pt[i].x + prevSlot.pt[i].x) / 2.0f; // 均值平滑
        } else {
            slot.pt[i].x = prevSlot.pt[i].x * SMOOTHING_FACTOR + slot.pt[i].x * (1 - SMOOTHING_FACTOR); // 权重平滑
        }

        if (jitter_y < POSITION_THRESHOLD_Y) {
            slot.pt[i].y = (slot.pt[i].y + prevSlot.pt[i].y) / 2.0f;
        } else {
            slot.pt[i].y = prevSlot.pt[i].y * SMOOTHING_FACTOR + slot.pt[i].y * (1 - SMOOTHING_FACTOR);
        }
    }
}

// 调整车位大小
void normalizeSlotSize(Sfus::FusionSlotInfo& slot) {
    // 计算车位的当前长和宽
    float width = computeDistance(slot.pt[0], slot.pt[1]);   // 宽度 (前边界)
    float length = computeDistance(slot.pt[0], slot.pt[3]);  // 长度 (左边界)

    // 计算调整比例
    float widthScale = STANDARD_SLOT_WIDTH / width;
    float lengthScale = STANDARD_SLOT_LENGTH / length;

    // 以 pt[0] 为基准点进行缩放调整
    for (int i = 1; i < 4; i++) {
        if (i == 1 || i == 2) { // 调整宽度方向
            slot.pt[i].x = slot.pt[0].x + (slot.pt[i].x - slot.pt[0].x) * widthScale;
            slot.pt[i].y = slot.pt[0].y + (slot.pt[i].y - slot.pt[0].y) * widthScale;
        }
        if (i == 2 || i == 3) { // 调整长度方向
            slot.pt[i].x = slot.pt[0].x + (slot.pt[i].x - slot.pt[0].x) * lengthScale;
            slot.pt[i].y = slot.pt[0].y + (slot.pt[i].y - slot.pt[0].y) * lengthScale;
        }
    }
}

// 车位滤波主函数
void PSD2VCUSlotsFilter(Sfus::FusionSlotInfovector& psd2vcu, const Sfus::FusionSlotInfovector& prevPsd2vcu) {
    for (int i = 0; i < psd2vcu.slotNum; i++) {
        if (i < prevPsd2vcu.slotNum) {
            filterSlotPosition(psd2vcu.FusionSlotInfo[i], prevPsd2vcu.FusionSlotInfo[i]);
        }
        normalizeSlotSize(psd2vcu.FusionSlotInfo[i]);
    }
}

