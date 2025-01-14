#include "fusion.h"

using namespace std;

const double IOU_THRESHOLD = 0.6;
const int PARALLEL_SLOT_WIDTH = 8000;
const int PARALLEL_OFFSET_1 = 250;
const int PARALLEL_OFFSET_2 = 600;
const int PARALLEL_OFFSET_3 = 2500;
const int PARALLEL_OFFSET_4 = 6800;
const int PERPENDICULAR_SLOT_WIDTH = 4000;
const int PERPENDICULAR_OFFSET_1 = 300;
const int PERPENDICULAR_OFFSET_2 = 6000;
const int PERPENDICULAR_OFFSET_3 = 3400;
#define PI 3.14

typedef IOU::Vec2<double> Vec2d;
typedef Vec2d VPoint;
typedef std::vector<VPoint> Vertexes;

slotfusion::slotfusion()
{

}

slotfusion::~slotfusion()
{

}

void slotfusion::mergeSlotLists(const apaSlotListInfo &outputSlot_USS,apaSlotListInfo &outputSlot_VIS,apaSlotListInfo &outputSlot_FUSION) 
{
    // LOGD("The Vison slot num is: %d",outputSlot_VIS.slots_in_cur_frame.size());
    // LOGD("The USS slot num is: %d",outputSlot_USS.slots_in_cur_frame.size());
    outputSlot_FUSION = outputSlot_VIS;
    // CTransformation transtoworld;
    // TPose local_point, global_point;
    // if (outputSlot_USS.slots_in_cur_frame.size() != 0 &&
    //     outputSlot_VIS.WorldoutRect.size() != 0){
    //         global_point.point.x = outputSlot_VIS.WorldoutRect[0].rectInfo.pt[0].x;
    //         global_point.point.y = outputSlot_VIS.WorldoutRect[0].rectInfo.pt[0].y;
    //         local_point.point.x = outputSlot_USS.slots_in_cur_frame[0].rectInfo.pt[0].x;
    //         local_point.point.y = outputSlot_USS.slots_in_cur_frame[0].rectInfo.pt[0].y;
    //     }
    
    // transtoworld.SetPoseToPose(local_point, global_point);

    for (const auto &slot_USS : outputSlot_USS.slots_in_cur_frame) 
    {
        // IOU
        bool mis_detect_flag = true;
        auto it = existed_in_psinfo(slot_USS, outputSlot_VIS, mis_detect_flag);
        if(!mis_detect_flag) {
            break;
        }else{
            outputSlot_FUSION.slots_in_cur_frame.push_back(slot_USS);
            std::cout<<"USS slot not match in vison slot and push bash to Fusion slots!"<<std::endl;
        }
        
        // bool overlapFound = false; 
        // double overlap = 0.0;
        // for (const auto &slot_VIS : outputSlot_VIS.slots_in_cur_frame){
        //     // overlap = calculateOverlap(slot_USS.rectInfo, slot_VIS.rectInfo);
        //     // std::cout<<"The overlap is:"<<overlap<<std::endl;
        //     std::cout<<"VIS slot id is:"<<slot_VIS.rectInfo.label<<std::endl;
        //     std::cout<<"USS slot id is:"<<slot_USS.rectInfo.label<<std::endl;
        //     // if (overlap > IOU_THRESHOLD){
        //     //     overlapFound = true;
        //     //     std::cout<<"Found overlap slots"<<std::endl;
        //     //     break;
        //     // }

            

        // }

        // if (!overlapFound){
        //     //如果视觉车位和超声车不位重叠，则将USS车位列表加入融合列表内
        //     outputSlot_FUSION.slots_in_cur_frame.push_back(slot_USS);
        //     std::cout<<"USS slot not match in vison slot and push bash to Fusion slots!"<<std::endl;
        // }
    }
    // LOGD("The fusion slot num is: %d",outputSlot_FUSION.slots_in_cur_frame.size());
}

vector<apaSlotInfo>::iterator slotfusion::existed_in_psinfo(const apaSlotInfo& rect_new, apaSlotListInfo& vison_slot_list, bool& mis_detect_flag)
{

    double iou = 0;;
    if(vison_slot_list.slots_in_cur_frame.size() == 0) {
        vector<apaSlotInfo>::iterator it = vison_slot_list.slots_in_cur_frame.end();
        return it;
    }

    //size>0时，反向遍历psinfo。IOU>0.4重复，0.2-0.4misdetect，<0.2认为没有相同车位继续循环
    vector<apaSlotInfo>::iterator it = vison_slot_list.slots_in_cur_frame.end() - 1;
    for( ; it >= vison_slot_list.slots_in_cur_frame.begin(); it--) 
    {
        std::cout<<"VIS slot id is:"<<it->rectInfo.label<<std::endl;
        std::cout<<"USS slot id is:"<<rect_new.rectInfo.label<<std::endl;
        Vertexes vert_new, vert;
        IOU::changePoint(rect_new, vert_new);
        IOU::changePoint(*it, vert);
        iou = IOU::iouEx(vert_new, vert);
        
        if (iou >= 0.2) {
            mis_detect_flag = false;
            break;
        }
        else if (iou >= 0.0 && iou < 0.2) {
            mis_detect_flag = true;
        }
        
    }
    return it;
}


static inline float Getslotangle(UssIf_stSlotProperty_t uss_point){
    Eigen::Vector2f dir1 = {floor((uss_point.stSlotPt[1].x - uss_point.stSlotPt[2].x) / 2), floor((uss_point.stSlotPt[1].y - uss_point.stSlotPt[2].y) / 2)};
    dir1 = dir1.normalized();
    Eigen::Vector2f dir2 = {floor((uss_point.stSlotPt[1].x - uss_point.stSlotPt[0].x) / 2), floor((uss_point.stSlotPt[1].y - uss_point.stSlotPt[0].y) / 2)};
    dir2 = dir2.normalized();
    Eigen::Vector2f dir3 = {floor((uss_point.stSlotPt[0].x - uss_point.stSlotPt[3].x) / 2), floor((uss_point.stSlotPt[0].y - uss_point.stSlotPt[3].y) / 2)};
    dir3 = dir3.normalized();

    float slot_angle;
    float slot_angle_1 = atan2(dir1.x(), dir1.y());
    float slot_angle_2 = atan2(dir3.x(), dir3.y());
    slot_angle = PI/2 - (slot_angle_1 + slot_angle_2) /2.0;
    return slot_angle;
}

static inline int CalcDistance(int ax, int ay, int bx, int by)
{
    int dis = sqrt((ax-bx)*(ax-bx) + (ay-by)*(ay-by));
    return dis;
}

void slotfusion::postprocessUSSslots(UssIf_stPLVOutputInfo_t &total_uss_slot){
    //TODO
    for (int jcnt = 0; jcnt < 2; jcnt++){
        for (int icnt = 0; icnt < total_uss_slot.UssIf_stSlotInfo[jcnt].u8SlotNum; ++icnt){
            if (total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].enmSlotType == USSIF_SLOT_TYPE_PARALLEL_E && total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].u16SlotLength <= PARALLEL_SLOT_WIDTH){
                // 不是单边超声垂直车位
                if (CalcDistance(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y) == 0){
                                    continue;
                                 }
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x += PARALLEL_OFFSET_2;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y += PARALLEL_OFFSET_1;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x -= PARALLEL_OFFSET_2;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y += PARALLEL_OFFSET_1;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].x -= PARALLEL_OFFSET_2;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y - PARALLEL_OFFSET_3;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].x += PARALLEL_OFFSET_2;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y - PARALLEL_OFFSET_3;

            }else if(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].enmSlotType == USSIF_SLOT_TYPE_PARALLEL_E && total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].u16SlotLength > PARALLEL_SLOT_WIDTH){
                // 是单边超声水平车位
                if (CalcDistance(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y) == 0){
                                    continue;
                                 }
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x -= PARALLEL_OFFSET_2;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y += PARALLEL_OFFSET_1;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x - PARALLEL_OFFSET_4;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y += total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y;
                
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].x -= PARALLEL_OFFSET_2;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y - PARALLEL_OFFSET_3;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].x - PARALLEL_OFFSET_4;
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y - PARALLEL_OFFSET_3;
            }else if(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].enmSlotType == USSIF_SLOT_TYPE_PERPENDICULAR_E ||
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].enmSlotType == USSIF_SLOT_TYPE_ANGULAR_E && total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].u16SlotLength <= PARALLEL_SLOT_WIDTH){
                // 不是单边超声垂直车位
                if (CalcDistance(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y) == 0){
                                    continue;
                                 }
                if(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x < 0){
                    // 车辆左侧
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x;
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y += PERPENDICULAR_OFFSET_1/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));
                    
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x;
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y -= PERPENDICULAR_OFFSET_1/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));
                    
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x - PERPENDICULAR_OFFSET_2;
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].y -= PERPENDICULAR_OFFSET_1/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));

                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].x;  
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y;
                }else{
                    // 车辆右侧
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x;
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y -= PERPENDICULAR_OFFSET_1/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));
                    
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x;
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y += PERPENDICULAR_OFFSET_1/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));
                    
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x + PERPENDICULAR_OFFSET_2;
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y;

                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].x;  
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y;
                }
                
                
            }else if(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].enmSlotType == USSIF_SLOT_TYPE_PERPENDICULAR_E ||
                    total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].enmSlotType == USSIF_SLOT_TYPE_ANGULAR_E && total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].u16SlotLength > PARALLEL_SLOT_WIDTH){
                // 是单边超声垂直车位
                if (CalcDistance(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x,
                                 total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y) == 0){
                                    continue;
                                 }
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x -= PERPENDICULAR_OFFSET_1/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y;
                
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x -  PERPENDICULAR_OFFSET_3/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y;
                
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].x -= PERPENDICULAR_OFFSET_1/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[2].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].y - PERPENDICULAR_OFFSET_2;
                
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].x = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[1].x -  PERPENDICULAR_OFFSET_3/std::cos(Getslotangle(total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt]));
                total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[3].y = total_uss_slot.UssIf_stSlotInfo[jcnt].UssIf_stSlotProperty[icnt].stSlotPt[0].y - PERPENDICULAR_OFFSET_2;
            }
        }
    }
}

bool slotfusion::deleteinvalidslot(UssIf_stSlotProperty_t uss_slot){
    // delete invalid slots
     // 一个车位有效的条件
    // if (uss_slot.enmSlotType == 0 || uss_slot.enmSlotType == 3) {
    //     return false; // 类型为0和3的车位无效
    // }
    if (uss_slot.enmSlotType == 1) { // chuizhi
        if (uss_slot.u16SlotDepth < 605 || uss_slot.u16SlotLength < 250) {
            LOGD("USS_DEL:The USS slot is be deleted!");
            return false; // 类型为1的车位深度小于500或长度小于250无效
        }
    }

    if (uss_slot.enmSlotType == 2) { // pingxing
        if (uss_slot.u16SlotDepth < 250 || uss_slot.u16SlotLength < 605) {
            LOGD("USS_DEL:The USS slot is be deleted!");
            return false; // 类型为2的车位深度小于250或长度小于500无效
        }
    }

    return true; // 其他情况，车位有效
}

void slotfusion::fillVisonstruct(const UssIf_stPLVOutputInfo_t &total_uss_slot, apaSlotListInfo &uss_slots){
    // 与视觉车位类型统一的PLV车位列表
    // 数据类型统一，含义统一
    uss_slots.slots_in_cur_frame.clear();
    int USS_total_slotnum = total_uss_slot.UssIf_stSlotInfo[0].u8SlotNum + total_uss_slot.UssIf_stSlotInfo[1].u8SlotNum;
    uss_slots.slots_in_cur_frame.reserve(USS_total_slotnum);
    int slot_id = 10000;
    LOGD("USS_DEL:The USS slot size is:%d",USS_total_slotnum);
    for(int icnt = 0; icnt < 2; icnt++){
        if(total_uss_slot.UssIf_stSlotInfo[icnt].u8SlotNum  != 0){
            for (int i = 0; i < total_uss_slot.UssIf_stSlotInfo[icnt].u8SlotNum; ++i) 
            {
                UssIf_stSlotProperty_t slotProperty = total_uss_slot.UssIf_stSlotInfo[icnt].UssIf_stSlotProperty[i];
                if(!deleteinvalidslot(slotProperty)){break;}
                apaSlotInfo slotInfo;
                APA_SPACE::SApaPSRect& rectInfo = slotInfo.rectInfo;

                for (int j = 0; j < RECTPointNum; ++j)
                {
                    rectInfo.pt[j].x = slotProperty.stSlotPt[j].x;
                    rectInfo.pt[j].y = slotProperty.stSlotPt[j].y;
                }

                rectInfo.label = slot_id; //USS slot ID从10000开始
                slot_id++;
                rectInfo.PStype = slottype_uss2rd(slotProperty.enmSlotType);
                rectInfo.iSodType = slotProperty.enmInSlotObstacleStatus;
                rectInfo.iDownSlotSOD = slotProperty.enmDownSlotSODType;
                rectInfo.iMinOtherSideDist = slotProperty.u16UssOppositeSpace;
                rectInfo.iRoadEdgeDist = slotProperty.u16ObjDistanceBetweenLineABToSlotBottom;
                
                uss_slots.slots_in_cur_frame.push_back(slotInfo);
            }
        }
    }
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
