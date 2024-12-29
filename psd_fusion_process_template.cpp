#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include <iostream>
#include <typeinfo>

#define OBS_READY false // OBS接口是否接入数据

// 1227试驾 OV049车专用 offset调优角点性能, y=kx+b
#define OFFSET_FOR_RIDE false // 1227试驾 OV049车专用 offset调优角点性能


#define VEHICLE_LENGTH 5259.9 
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7 
#define MM_TO_M 1000

float cal_k = 1;
float b = -200;

bool vcuclearflag = 0;

int apa_status;
StatusDecFusionInput psd2statemachine;
Sfus::Sfsuion2DecPlan psd2planning; //动态车位列表
Sfus::Sfsuion2DecPlan psd2planning_fixed; //静态目标车位

int HMI_select_ID = 0; //HMI只发1s。HMI_select是HMI发的ID，
int HMI_temp_ID = 0;  //HMI_temp_ID是存下来的ID
int VCU_select_ID_ON = 0; //VCU发送的ID
int final_select_ID = 0; //VCU和HMI最终统一的ID

CDT_PSD_FUSION_PROCESS_TEMPLATE(cpsd_fusion_process)

cpsd_fusion_process::cpsd_fusion_process()
{

}

cpsd_fusion_process::~cpsd_fusion_process()
{

}

tResult cpsd_fusion_process::Init()
{
    LOGW("PSD Process Start Success!");
    // Load Config
    if (!LoadFromFile("psd_config.json")) {
        std::cerr << "Load config failed!" << std::endl;
    }

    // part1 初始化
    static int init_flag = 0;
    if(init_flag == 0){
        //updatevisionslots 初始化
        PSD_FusionModuleIFrunable.Initialize();
        init_flag = 1;

        //statemachine 初始化
        psd2statemachine.aps_apaParkType = 0;
        psd2statemachine.aps_apaAvailableSlot = 0;
        psd2statemachine.aps_apaHighlightSlot = 0;
        psd2statemachine.aps_apaNarrowSlot = 0;
        psd2statemachine.aps_apaParkFusionType = 0;
        psd2statemachine.aps_apaParkPlaceNum = 0;

        //planning 初始化
        psd2planning.targetSlot.slotType = Sfus::SLOTTYP_NULL;
        psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_NULL;
        psd2planning.targetSlot.slotCorners.cornerA.x = 0;
        psd2planning.targetSlot.slotCorners.cornerA.y = 0;
        psd2planning.targetSlot.slotCorners.cornerB.x = 0;
        psd2planning.targetSlot.slotCorners.cornerB.y = 0;
        psd2planning.targetSlot.slotCorners.cornerC.x = 0;
        psd2planning.targetSlot.slotCorners.cornerC.y = 0;
        psd2planning.targetSlot.slotCorners.cornerD.x = 0;
        psd2planning.targetSlot.slotCorners.cornerD.y = 0;
    }

    RETURN_NOERROR;
}

tResult cpsd_fusion_process::Term()
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::Start()
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::Stop()
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::ThreadTrigger_thread()
{
    RETURN_NOERROR;
}

bool cpsd_fusion_process::LoadFromFile(const std::string& filename){
    std::ifstream inFile(filename);
    if(!inFile.is_open()){
        DEBUG = false;
        std::cerr << "无法打开配置文件: " << filename << ", DEBUG:"<< DEBUG <<std::endl;
        return false;
    }

    try{
        json j;
        inFile >> j;

        //解析文件路径
        j.at("debug").at("save_to_json").get_to(DEBUG);
    }
    catch (json::exception& e) {
            std::cerr << "配置文件解析错误: " << e.what() << std::endl;
            return false;
    }
    inFile.close();
    return true;
}

tResult cpsd_fusion_process::TimeTrigger_Timer50()
{
    // auto start50 = std::chrono::steady_clock::now();
    // auto end50 = std::chrono::steady_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end50 - start50);
    // std::cout<<"[TIMECOST]Timetrigger50 time is:"<< elapsed.count() <<std::endl;
    RETURN_NOERROR;
}

slotfusion fusionslot;

tResult cpsd_fusion_process::TimeTrigger_Timer100()
{
    auto start = std::chrono::steady_clock::now();
    
    // part2 输入，上游：RD, DR,statemachine
    //***********get rd (frameid and singleframeslot)
    rd::QuadParkingSlots rd_info;
    unsigned long long singleframeslotsID;
    std::vector<padVisionSlotCoord> singleframeslots;  //中间结构体，转存rd单帧车位列表
    padVisionSlotCoord oneslot; 
    if (apa_status != 1){
        EMC_TROS_Bridge_Parking_GetFieldQuadParkingSlots(rd_info); //rd::Header, rd::QuadParkingSlot
        if (DEBUG == true){
            filetojson.SaveQuadParkingSlotsInfoToJson(rd_info, "RDinfo.json");
        }
        //frameid
        // LOGD("[_test rd_info timestampNs] J5 SEND timestampNs: %llu",rd_info.frameTimeStampNs);
        
        singleframeslotsID = rd_info.frameTimeStampNs;
        // LOGD("[_test rd_info timestampNs] S32G RECEIVE timestampNs: %llu",singleframeslotsID);
        //singleframeslot
        
        if (!rd_info.quadParkingSlotList.empty()){
            LOGD("[_test rd_info singleframeslots] J5 SEND RD output slots size: %d",rd_info.quadParkingSlotList.size());
            for (const auto& parkingSlot : rd_info.quadParkingSlotList){
                // LOGD("[_test rd_info singleframeslots] J5 SEND slottype(chuizhi0shuiping1xiexiang2): %d, label(0buzhanyong): %u",parkingSlot.slotType,parkingSlot.label);
                LOGD("[_test rd_info singleframeslots] J5 SEND tl:(%f,%f), bl:(%f,%f), tr:(%f,%f), br:(%f,%f)",parkingSlot.tl.x,parkingSlot.tl.y,parkingSlot.bl.x,parkingSlot.bl.y,
                parkingSlot.tr.x,parkingSlot.tr.y,parkingSlot.br.x,parkingSlot.br.y);

                // 0xFF 作为默认值
                if (parkingSlot.slotType == 0) {
                    oneslot.bayType = 0x00;
                } else if (parkingSlot.slotType == 1) {
                    oneslot.bayType = 0x01;
                } else if (parkingSlot.slotType == 2) {
                    oneslot.bayType = 0x02;
                } else {
                    oneslot.bayType = 0xFF;
                }

                oneslot.occupy = parkingSlot.label; //0 unoccupied, 1 occupied
                
                //左右判断,按规划/定位ABCD顺序输出车位角点
                if (parkingSlot.tl.x < 224 && parkingSlot.tr.x < 224){
                    oneslot.slotSide = 0x01; //x小于图像中心，判断为左
                    oneslot.a.x = int(parkingSlot.tr.x);
                    oneslot.a.y = int(parkingSlot.tr.y);
                    oneslot.b.x = int(parkingSlot.tl.x);
                    oneslot.b.y = int(parkingSlot.tl.y);
                    oneslot.c.x = int(parkingSlot.bl.x);
                    oneslot.c.y = int(parkingSlot.bl.y);
                    oneslot.d.x = int(parkingSlot.br.x);
                    oneslot.d.y = int(parkingSlot.br.y);
                    // LOGD("[_test rd_info singleframeslots] S32G RECEIVE LEFT SLOTS tl:(%d,%d), tr:(%d,%d), br:(%d,%d), bl:(%d,%d)",oneslot.b.x,oneslot.b.y,oneslot.a.x,oneslot.a.y,
                // oneslot.d.x,oneslot.d.y,oneslot.c.x,oneslot.c.y);
                }
                else {
                    oneslot.slotSide = 0x00;
                    oneslot.a.x = int(parkingSlot.tl.x);
                    oneslot.a.y = int(parkingSlot.tl.y);
                    oneslot.b.x = int(parkingSlot.tr.x);
                    oneslot.b.y = int(parkingSlot.tr.y);
                    oneslot.c.x = int(parkingSlot.br.x);
                    oneslot.c.y = int(parkingSlot.br.y);
                    oneslot.d.x = int(parkingSlot.bl.x);
                    oneslot.d.y = int(parkingSlot.bl.y);
                //     LOGD("[_test rd_info singleframeslots] S32G RECEIVE RIGHT SLOTS tl:(%d,%d), tr:(%d,%d), br:(%d,%d), bl:(%d,%d)",oneslot.a.x,oneslot.a.y,oneslot.b.x,oneslot.b.y,
                // oneslot.c.x,oneslot.c.y,oneslot.d.x,oneslot.d.y);
                }
                singleframeslots.push_back(oneslot);
            }
        }
        else{
            // LOGD("[_test rd_info singleframeslots] no parking slots received")
        }
    }
    
    Loc::App2emap_DR dr_pose;
    padVehiclePose  pose_globaldata;
    //***********get dr
    if (apa_status != 1){
        EMC_TROS_Bridge_Parking_GetFieldApp2emap_DR(dr_pose); //Loc::App2emap_DR
        if (DEBUG == true){
            filetojson.SaveDRInfoToJson(dr_pose, "DR_POSE.json");
        }
        // LOGD("[_test dr_pose] J5 SEND x: %f, y: %f, yaw: %f, timestamp: %llu",dr_pose.x, dr_pose.y, dr_pose.canAng,dr_pose.timeStamp);
        //@TODO dr_pose long int
        
        pose_globaldata.coord.x = int(dr_pose.x);
        pose_globaldata.coord.y = int(dr_pose.y);
        pose_globaldata.yaw = dr_pose.canAng; //dr是角度，转换为弧度
        LOGD("[_test dr_pose] S32G RECEIVE x:%d, y: %d, yaw: %f",pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);
    }

    // part3 算法
    //进入search才开始车位融合
    if (apa_status != 1) {
        PSD_FusionModuleIFrunable.UpdateVechiclePose(pose_globaldata);
        PSD_FusionModuleIFrunable.UpdateVisionSlots(singleframeslotsID, singleframeslots, apa_status);
        
        
        if(apa_status == 0 || apa_status == 1 || apa_status == 6 || apa_status == 7){
            memset(&outputSlot_FUSED, 0, sizeof(apaSlotListInfo));
            std::cout<<"status is 2 and clear slot map"<<std::endl;
        }
        
        outputSlot_VIS = PSD_FusionModuleIFrunable.GetOutputSlot();
        
        if (DEBUG == true){
            filetojson.SaveapaSlotListInfoToJson(outputSlot_VIS,"VISapaSlotListInfo.json");
        }
        fusionslot.mergeSlotLists(outputSlot_USS, outputSlot_VIS , outputSlot_FUSED);
    }
    if (apa_status == 1){
        clear_flag = false;
        memset(&outputSlot_FUSED, 0, sizeof(apaSlotListInfo));
        memset(&outputSlot_VIS, 0, sizeof(apaSlotListInfo));
    }
    
    
    // 当泊车完成或者中断，清空车位列表
    if (apa_status == 1 || apa_status == 6 || apa_status == 7){
        outputSlot_FUSED.slots_in_cur_frame.clear();
    }

    // // 输出视觉原始角点
    // for (auto & psd_m_output : outputSlot_VIS.slots_in_cur_frame){
    //     LOGD("ORIGIN VISSLOTS: TOTAL SLOT NUM: %d, Slot#%d, type: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
    //     outputSlot_VIS.slots_in_cur_frame.size(),
    //     psd_m_output.rectInfo.label,
    //     psd_m_output.rectInfo.PStype,
    //     psd_m_output.rectInfo.pt[0].x,
    //     psd_m_output.rectInfo.pt[0].y,
    //     psd_m_output.rectInfo.pt[1].x,
    //     psd_m_output.rectInfo.pt[1].y,
    //     psd_m_output.rectInfo.pt[2].x,
    //     psd_m_output.rectInfo.pt[2].y,
    //     psd_m_output.rectInfo.pt[3].x,
    //     psd_m_output.rectInfo.pt[3].y);
    // }

    // 1227试驾  OV049车专用 offset调优角点性能。表现为y轴方向融合后y 大于 实测值y 230mm
    if (OFFSET_FOR_RIDE){
        for (auto & offset_slot : outputSlot_VIS.slots_in_cur_frame){
            offset_slot.rectInfo.pt[0].y = cal_k * (offset_slot.rectInfo.pt[0].y) + b; 
            offset_slot.rectInfo.pt[1].y = cal_k * (offset_slot.rectInfo.pt[1].y) + b; 
            offset_slot.rectInfo.pt[2].y = cal_k * (offset_slot.rectInfo.pt[2].y) + b; 
            offset_slot.rectInfo.pt[3].y = cal_k * (offset_slot.rectInfo.pt[3].y) + b; 
        }
    }
    // // 输出OFFSET角点
    // if (OFFSET_FOR_RIDE){
    //     for (auto & psd_m_output : outputSlot_VIS.slots_in_cur_frame){
    //         LOGD("OFFSET VISSLOTS: TOTAL SLOT NUM: %d, Slot#%d, type: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
    //         outputSlot_VIS.slots_in_cur_frame.size(),
    //         psd_m_output.rectInfo.label,
    //         psd_m_output.rectInfo.PStype,
    //         psd_m_output.rectInfo.pt[0].x,
    //         psd_m_output.rectInfo.pt[0].y,
    //         psd_m_output.rectInfo.pt[1].x,
    //         psd_m_output.rectInfo.pt[1].y,
    //         psd_m_output.rectInfo.pt[2].x,
    //         psd_m_output.rectInfo.pt[2].y,
    //         psd_m_output.rectInfo.pt[3].x,
    //         psd_m_output.rectInfo.pt[3].y);
    //     }
    // }

    // part4 输出，下游：APAHANDLE, PERCEPTION, VCU, PLANNING，STATEMACHINE
    //***********APAHANDLE 发送车位列表
    Fsm::FusionSlotInfo2Location psd2location;
    int tempsize;
    tempsize = outputSlot_FUSED.slots_in_cur_frame.size();
    psd2location.slotNum = tempsize;
    if (psd2location.slotNum > 0){
        int j = 0;

        for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame){
            if (j >= tempsize || j >= 50){
                std::cout<<"die in APAhandle and size is:"<<tempsize<<std::endl;
                break;
            }
            psd2location.fusionSlotInfo[j].slotLabel = psd_m_output.rectInfo.label; //ID
            psd2location.fusionSlotInfo[j].slotType = slottype_rd2vcu(psd_m_output.rectInfo.PStype);
            psd2location.fusionSlotInfo[j].pt[0].x = psd_m_output.rectInfo.pt[0].x;
            psd2location.fusionSlotInfo[j].pt[0].y = psd_m_output.rectInfo.pt[0].y;
            psd2location.fusionSlotInfo[j].pt[1].x = psd_m_output.rectInfo.pt[1].x;
            psd2location.fusionSlotInfo[j].pt[1].y = psd_m_output.rectInfo.pt[1].y;
            psd2location.fusionSlotInfo[j].pt[2].x = psd_m_output.rectInfo.pt[2].x;
            psd2location.fusionSlotInfo[j].pt[2].y = psd_m_output.rectInfo.pt[2].y;
            psd2location.fusionSlotInfo[j].pt[3].x = psd_m_output.rectInfo.pt[3].x;
            psd2location.fusionSlotInfo[j].pt[3].y = psd_m_output.rectInfo.pt[3].y;

            // LOGD("[PSD2APAHANDLE] TOTAL SLOT NUM: %d, Slot#%d, type: %d (%d, %d) (%d, %d) (%d, %d) (%d, %d)",
            //         psd2location.slotNum,
            //         psd2location.fusionSlotInfo[j].slotLabel,
            //         psd2location.fusionSlotInfo[j].slotType,
            //         psd2location.fusionSlotInfo[j].pt[0].x,
            //         psd2location.fusionSlotInfo[j].pt[0].y,
            //         psd2location.fusionSlotInfo[j].pt[1].x,
            //         psd2location.fusionSlotInfo[j].pt[1].y,
            //         psd2location.fusionSlotInfo[j].pt[2].x,
            //         psd2location.fusionSlotInfo[j].pt[2].y,
            //         psd2location.fusionSlotInfo[j].pt[3].x,
            //         psd2location.fusionSlotInfo[j].pt[3].y);
            j++;

        }
        EMC_psd_fusion_process_SetFieldFusionSlotInfo2Location(psd2location);
    }

    //***********PLANNING/HMI 发送车位列表
    //check psd output to planning(1 slot list)  
    //规划暂时不用车位列表，需等待预规划模块ready后。暂时用于HMI显示车位列表
    int planning_slotnum = outputSlot_FUSED.slots_in_cur_frame.size();
    int k = 0;
    for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame){
        if (k >= tempsize || k >= 50){
            std::cout<<"die in planning and size is:"<< tempsize << std::endl;
            break;
        }
        psd2planning.SfusionSrchSlots[k].slotID = psd_m_output.rectInfo.label;
        psd2planning.SfusionSrchSlots[k].slotType = slottype_rd2decplan(psd_m_output.rectInfo.PStype);
        psd2planning.SfusionSrchSlots[k].slotSource = Sfus::SLOTSRC_VIS;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerA.x = psd_m_output.rectInfo.pt[0].x;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerA.y = psd_m_output.rectInfo.pt[0].y;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerB.x = psd_m_output.rectInfo.pt[1].x;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerB.y = psd_m_output.rectInfo.pt[1].y;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerC.x = psd_m_output.rectInfo.pt[2].x;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerC.y = psd_m_output.rectInfo.pt[2].y;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerD.x = psd_m_output.rectInfo.pt[3].x;
        psd2planning.SfusionSrchSlots[k].slotCorners.cornerD.y = psd_m_output.rectInfo.pt[3].y;

        LOGD("[PSD2PLANNING] TOTAL SLOT NUM: %d, Slot#%d, type: %d, source: %d (%.1f, %.1f) (%.1f, %.1f) (%.1f, %.1f) (%.1f, %.1f)",
                planning_slotnum,
                psd2planning.SfusionSrchSlots[k].slotID,
                psd2planning.SfusionSrchSlots[k].slotType,
                psd2planning.SfusionSrchSlots[k].slotSource,
                psd2planning.SfusionSrchSlots[k].slotCorners.cornerA.x,
                psd2planning.SfusionSrchSlots[k].slotCorners.cornerA.y,
                psd2planning.SfusionSrchSlots[k].slotCorners.cornerB.x,
                psd2planning.SfusionSrchSlots[k].slotCorners.cornerB.y,
                psd2planning.SfusionSrchSlots[k].slotCorners.cornerC.x,
                psd2planning.SfusionSrchSlots[k].slotCorners.cornerC.y,
                psd2planning.SfusionSrchSlots[k].slotCorners.cornerD.x,
                psd2planning.SfusionSrchSlots[k].slotCorners.cornerD.y);
        k++;
    }
    // EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);


    //**********VCU,HMI 双终端接收点选的目标车位，STATEMACHINE根据点选车位交互
    //HMI 部分
    //中间变量保存HMI发送的 [0 - ID - 0]，一秒内发送五次
    if (HMI_select_ID)
    {
        HMI_temp_ID = HMI_select_ID;
    }
    //VCU 部分，从OnSelectSlot接收
    //VCU接收的点选车位 与 HMI接收的点选车位 二选一
    if (VCU_select_ID_ON != 0 && HMI_temp_ID == 0) {
        final_select_ID = VCU_select_ID_ON; 
    } 
    else if (VCU_select_ID_ON == 0 && HMI_temp_ID != 0) {
        final_select_ID = HMI_temp_ID;
    } 
    else if (VCU_select_ID_ON != 0 && HMI_temp_ID != 0 && (VCU_select_ID_ON == HMI_temp_ID)) {
        final_select_ID = HMI_temp_ID;
    }
    else if (VCU_select_ID_ON == 0 && HMI_temp_ID == 0){
        final_select_ID = 0;
    }
    else {
        final_select_ID = HMI_temp_ID;
    }
    // APAStatus == standby/finish/error时，清零目标车位
    Fsm::Slotlabel psd2apahandel_targetID;
    if (apa_status == 1 || apa_status == 6 || apa_status == 7){
        final_select_ID = 0;
        VCU_select_ID_ON = 0;
        // psd2apahandel_targetID.targetSlotLabel = final_select_ID;
        // EMC_psd_fusion_process_SetFieldSlotlabel(psd2apahandel_targetID);
    }
    // 目标车位ID发送给下游规划，VCU（psd2vcu结构体）
    if (final_select_ID){
        psd2apahandel_targetID.targetSlotLabel = final_select_ID;
        EMC_psd_fusion_process_SetFieldSlotlabel(psd2apahandel_targetID);
    }
    LOGD("[SELECTID] HMI %d, VCU %d, final %d",HMI_temp_ID,VCU_select_ID_ON,final_select_ID);

    //***********PLANNING 发送目标车位
    //check psd output to planning(2 target slot)
    //拿到目标车位ID后，发送目标车位信息给planning
    if (final_select_ID > 0 && apa_status != 5){ //进入guidance后固定目标车位角点
        for (int i = 0; i < outputSlot_FUSED.slots_in_cur_frame.size();++i){
            if (final_select_ID == outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.label){
                psd2planning.targetSlot.slotCorners.cornerA.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].x; 
                psd2planning.targetSlot.slotCorners.cornerA.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[0].y;
                psd2planning.targetSlot.slotCorners.cornerB.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].x;
                psd2planning.targetSlot.slotCorners.cornerB.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[1].y;
                psd2planning.targetSlot.slotCorners.cornerC.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].x;
                psd2planning.targetSlot.slotCorners.cornerC.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[2].y;
                psd2planning.targetSlot.slotCorners.cornerD.x = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].x;
                psd2planning.targetSlot.slotCorners.cornerD.y = outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.pt[3].y;
                psd2planning.targetSlot.slotType = slottype_rd2decplan(outputSlot_FUSED.slots_in_cur_frame[i].rectInfo.PStype);
                psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
            }
        }
    }
    
    if (apa_status == 1 || apa_status == 6 || apa_status == 7 || apa_status == 0){
        memset(&psd2planning, 0, sizeof(Sfus::Sfsuion2DecPlan));
        LOGD("psd2planning now: %f",psd2planning.targetSlot.slotCorners.cornerA.x);
        LOGD("psd2planning clear! status: %d",apa_status);
        EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
    }else{
        LOGD("[PSD2PLANNING] apastatus: %d, target slot: TYPE: %d, SOURCE: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
        apa_status,
        psd2planning.targetSlot.slotType,
        psd2planning.targetSlot.slotSource,
        psd2planning.targetSlot.slotCorners.cornerA.x,
        psd2planning.targetSlot.slotCorners.cornerA.y,
        psd2planning.targetSlot.slotCorners.cornerB.x,
        psd2planning.targetSlot.slotCorners.cornerB.y,/*  */
        psd2planning.targetSlot.slotCorners.cornerC.x,
        psd2planning.targetSlot.slotCorners.cornerC.y,
        psd2planning.targetSlot.slotCorners.cornerD.x,
        psd2planning.targetSlot.slotCorners.cornerD.y);
        EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);

    }

    // if (apa_status != 1 && apa_status != 6 && apa_status != 7 && apa_status != 0){
    //     LOGD("[PSD2PLANNING] apastatus: %d, target slot: TYPE: %d, SOURCE: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
    //     apa_status,
    //     psd2planning.targetSlot.slotType,
    //     psd2planning.targetSlot.slotSource,
    //     psd2planning.targetSlot.slotCorners.cornerA.x,
    //     psd2planning.targetSlot.slotCorners.cornerA.y,
    //     psd2planning.targetSlot.slotCorners.cornerB.x,
    //     psd2planning.targetSlot.slotCorners.cornerB.y,/*  */
    //     psd2planning.targetSlot.slotCorners.cornerC.x,
    //     psd2planning.targetSlot.slotCorners.cornerC.y,
    //     psd2planning.targetSlot.slotCorners.cornerD.x,
    //     psd2planning.targetSlot.slotCorners.cornerD.y);
    //     EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);
    // }
    
    //**********VCU 发送车位列表
    Sfus::FusionSlotInfovector psd2vcu;
    
    // 当状态变成泊车时
    if (apa_status != 5){
        std::cout<<"The apa staus is not 5!"<<std::endl;
        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
        psd2vcu.slotNum = tempsize;
        if (psd2vcu.slotNum > 0){
            int i = 0;
            LOGD("PSD2VCU apastatus: %d, outputslot_fused size: %d",apa_status,outputSlot_FUSED.slots_in_cur_frame.size());

            for (auto& psd_m_output : outputSlot_FUSED.slots_in_cur_frame){
                if (i >= tempsize || i >= 50){
                    std::cout<<"die in VCU and size is:"<<tempsize<<std::endl;
                    break;
                }

                //psd2vcu.FusionSlotInfo[i].slotType = slottype_rd2vcu(psd_m_output.rectInfo.PStype);
                psd2vcu.FusionSlotInfo[i].slotLabel = psd_m_output.rectInfo.label; //ID
                psd2vcu.FusionSlotInfo[i].displayLabel = 0;

                //ABCD顺序调整为VCU专用顺序
                //左侧
                if (psd_m_output.rectInfo.pt[0].x <= 0 || psd_m_output.rectInfo.pt[1].x <= 0 || psd_m_output.rectInfo.pt[2].x < 0){
                    psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) ) / MM_TO_M; //mm 转 m , VCU坐标系上x右y
                    psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[1].x / MM_TO_M; //后轴中心转前保中心
                    psd2vcu.FusionSlotInfo[i].pt[0].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) ) / MM_TO_M;
                    // psd2vcu.FusionSlotInfo[i].pt[1].y = psd2vcu.FusionSlotInfo[i].pt[0].y;
                    psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[0].x/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                    // psd2vcu.FusionSlotInfo[i].pt[3].y = psd2vcu.FusionSlotInfo[i].pt[2].y;
                    psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[2].x/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 3;
                    psd2vcu.FusionSlotInfo[i].backInAvailableFlag = 1;
                    psd2vcu.FusionSlotInfo[i].parkInHeadInSoftButtonCurrentValue = 1;
                }
                //右侧
                else{
                    psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M; //mm 转 m
                    psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[1].x / MM_TO_M; //后轴中心转前保中心
                    psd2vcu.FusionSlotInfo[i].pt[0].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[2].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[1].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                    // psd2vcu.FusionSlotInfo[i].pt[2].y = psd2vcu.FusionSlotInfo[i].pt[1].y;
                    psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[2].z = 0;

                    psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                    // psd2vcu.FusionSlotInfo[i].pt[3].y = psd2vcu.FusionSlotInfo[i].pt[0].y;
                    psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M;
                    psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                    psd2vcu.FusionSlotInfo[i].slotStatusType = 3;
                    psd2vcu.FusionSlotInfo[i].backInAvailableFlag = 1;
                    psd2vcu.FusionSlotInfo[i].parkInHeadInSoftButtonCurrentValue = 1;
                }

                //ID选择后互斥
                // ID从1000开始算
                if (final_select_ID-1000 >= 0 && tempsize > 0){
                    for (int i = 0; i < tempsize; i++) {
                        if (psd2vcu.FusionSlotInfo[i].slotLabel == final_select_ID) {
                            psd2vcu.FusionSlotInfo[i].slotStatusType = 5; // 设置为SELECTED状态
                        } else {
                            psd2vcu.FusionSlotInfo[i].slotStatusType = 3; // 设置为AVAILABLE状态
                        }
                    }
                }
                i++;
            }
        }
        EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
    }else{
        std::cout<<"The apa staus is 5!"<<std::endl;

        memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
        psd2vcu.FusionSlotInfo[0].displayLabel = 0;
        psd2vcu.FusionSlotInfo[0].slotLabel = final_select_ID;

        //ABCD顺序调整为VCU专用顺序
        //左侧
        if (psd2planning.targetSlot.slotCorners.cornerA.x <= 0 ||  psd2planning.targetSlot.slotCorners.cornerB.x <= 0){
            psd2vcu.FusionSlotInfo[0].pt[0].x = (psd2planning.targetSlot.slotCorners.cornerB.y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) ) / MM_TO_M; //mm 转 m , VCU坐标系上x右y
            psd2vcu.FusionSlotInfo[0].pt[0].y = psd2planning.targetSlot.slotCorners.cornerB.x / MM_TO_M; //后轴中心转前保中心
            psd2vcu.FusionSlotInfo[0].pt[0].z = 0;
            psd2vcu.FusionSlotInfo[0].pt[1].x = (psd2planning.targetSlot.slotCorners.cornerA.y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) ) / MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[1].y = psd2planning.targetSlot.slotCorners.cornerA.x/ MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[1].z = 0;
            psd2vcu.FusionSlotInfo[0].pt[2].x = (psd2planning.targetSlot.slotCorners.cornerD.y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[2].y = psd2planning.targetSlot.slotCorners.cornerD.x / MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[2].z = 0;
            psd2vcu.FusionSlotInfo[0].pt[3].x = (psd2planning.targetSlot.slotCorners.cornerC.y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[3].y = psd2planning.targetSlot.slotCorners.cornerC.x/ MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[3].z = 0;
            psd2vcu.FusionSlotInfo[0].slotStatusType = 3;
            psd2vcu.FusionSlotInfo[0].backInAvailableFlag = 1;
            psd2vcu.FusionSlotInfo[0].parkInHeadInSoftButtonCurrentValue = 1;
        }
        //右侧
        else{
            psd2vcu.FusionSlotInfo[0].pt[0].x = (psd2planning.targetSlot.slotCorners.cornerB.y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M; //mm 转 m
            psd2vcu.FusionSlotInfo[0].pt[0].y = psd2planning.targetSlot.slotCorners.cornerB.x / MM_TO_M; //后轴中心转前保中心
            psd2vcu.FusionSlotInfo[0].pt[0].z = 0;
            psd2vcu.FusionSlotInfo[0].pt[1].x = (psd2planning.targetSlot.slotCorners.cornerC.y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[1].y = psd2planning.targetSlot.slotCorners.cornerC.x / MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[1].z = 0;
            psd2vcu.FusionSlotInfo[0].pt[2].x = (psd2planning.targetSlot.slotCorners.cornerD.y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[2].y = psd2planning.targetSlot.slotCorners.cornerD.x / MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[2].z = 0;
            psd2vcu.FusionSlotInfo[0].pt[3].x = (psd2planning.targetSlot.slotCorners.cornerA.y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[3].y = psd2planning.targetSlot.slotCorners.cornerA.x / MM_TO_M;
            psd2vcu.FusionSlotInfo[0].pt[3].z = 0;
            psd2vcu.FusionSlotInfo[0].slotStatusType = 3;
            psd2vcu.FusionSlotInfo[0].backInAvailableFlag = 1;
            psd2vcu.FusionSlotInfo[0].parkInHeadInSoftButtonCurrentValue = 1;
        }
        LOGD("[SELECTID] selected slot:(%f,%f),(%f,%f),(%f,%f),(%f,%f)", psd2vcu.FusionSlotInfo[0].pt[0].x,psd2vcu.FusionSlotInfo[0].pt[0].y,
                                                                         psd2vcu.FusionSlotInfo[0].pt[1].x,psd2vcu.FusionSlotInfo[0].pt[1].y,
                                                                         psd2vcu.FusionSlotInfo[0].pt[2].x,psd2vcu.FusionSlotInfo[0].pt[2].y,
                                                                         psd2vcu.FusionSlotInfo[0].pt[3].x,psd2vcu.FusionSlotInfo[0].pt[3].y)
        LOGD("[SELECTID] SEND VCU TARGET SLOT!!!!")
        EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
    }

    //**********PERCEPTION 发送目标车位
    // 拿到目标车位后，发送给planning的目标车位信息，再给perception
    Sfus::SfusionSlots psd2perception;
    if (psd2planning.targetSlot.slotCorners.cornerA.x != 0){
        psd2perception.slotCorners.cornerA.x = psd2planning.targetSlot.slotCorners.cornerA.x;
        psd2perception.slotCorners.cornerA.y = psd2planning.targetSlot.slotCorners.cornerA.y;
        psd2perception.slotCorners.cornerB.x = psd2planning.targetSlot.slotCorners.cornerB.x;
        psd2perception.slotCorners.cornerB.y = psd2planning.targetSlot.slotCorners.cornerB.y;
        psd2perception.slotCorners.cornerC.x = psd2planning.targetSlot.slotCorners.cornerC.x;
        psd2perception.slotCorners.cornerC.y = psd2planning.targetSlot.slotCorners.cornerC.y;
        psd2perception.slotCorners.cornerD.x = psd2planning.targetSlot.slotCorners.cornerD.x;
        psd2perception.slotCorners.cornerD.y = psd2planning.targetSlot.slotCorners.cornerD.y;
        psd2perception.slotType = psd2planning.targetSlot.slotType;
        psd2perception.slotSource = psd2planning.targetSlot.slotSource;
    }
    EMC_psd_fusion_process_SetFieldSfusionSlots(psd2perception);


    //**********STATEMACHINE 交互
    psd2statemachine.aps_apaParkPlaceNum = tempsize;

    if (apa_status == 1 || apa_status == 6 || apa_status == 7){
        psd2statemachine.aps_apaParkType = 0;
        psd2statemachine.aps_apaParkPlaceNum = 0;
        psd2statemachine.aps_apaHighlightSlot = 0;
        final_select_ID = 0;
        HMI_temp_ID = 0;
    }
    if (final_select_ID > 0){
        psd2statemachine.aps_apaParkType = slottype_decplan2statemachine(psd2planning.targetSlot.slotType);
        if (final_select_ID >= 10000){
            psd2statemachine.aps_apaParkFusionType = 1;
        }
        else{
            psd2statemachine.aps_apaParkFusionType = 0;
        }
        psd2statemachine.aps_apaNarrowSlot = 0; //@TODO 窄车位
        if (tempsize > 0){
            psd2statemachine.aps_apaAvailableSlot = 1;
            psd2statemachine.aps_apaHighlightSlot = 1;
        }
    }
    // LOGD("SELECT ID: %d, aps_parktype = %d, aps_apaParkPlaceNum: %d",final_select_ID,psd2statemachine.aps_apaParkType,psd2statemachine.aps_apaParkPlaceNum);
    S2S_MCore_Bridge_SetSigStatusDecFusionInput(&psd2statemachine);

    auto end = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout<<"[TIMECOST]Timetrigger100 time is:"<< elapsed.count() <<std::endl;
    
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnVehicleCanData(const VehicleCanData& userData)
{
    RETURN_NOERROR;
}


tResult cpsd_fusion_process::OnStatusDecOutput(const StatusDecOutput& userData)
{
    apa_status = userData.aps_apaStatusReq;
    RETURN_NOERROR;
}


UssIf_stSlotInfo_t PLV_total_slots;

tResult cpsd_fusion_process::OnUssIf_stPLVOutputInfo(const UssIf_stPLVOutputInfo_t& userData)
{
    if (DEBUG == true){
        filetojson.SaveUssInfoToJson(userData,"USSapaSlotListInfo.json");
    }

    fusionslot.fillVisonstruct(userData, outputSlot_USS);
    LOGD("USSSLOT SIZE IS:%d",outputSlot_USS.slots_in_cur_frame.size());
    auto uss_info = userData;
    fusionslot.postprocessUSSslots(uss_info);

    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnAPAControlPlanOutput(const APAControlPlanOutput& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnStatusDecFusionOutput(const StatusDecFusionOutput& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnAPAControlDebugOutput(const APAControlDebugOutput& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnStatusDec2FusionDebug(const StatusDec2FusionDebug& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnEmapWorkMode(const Fus::EmapWorkMode& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnFusionTimeStamp(const Fus::FusionTimeStamp& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnVagueEmapGrid(const Fus::VagueEmapGrid& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnPreciseEmapGrid(const Fus::PreciseEmapGrid& userData)
{
    RETURN_NOERROR;
}

Fus::PkEmapObs obs_info;

tResult cpsd_fusion_process::OnPkEmapObs(const Fus::PkEmapObs& userData)
{
    if (OBS_READY){
        obs_info = userData;
        filetojson.SaveObsToJson(obs_info,"ObsInfo.json");
    }
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnEmapSlotVector(const Fus::EmapSlotVector& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnStableEmapObs(const Fus::StableEmapObs& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnFusionSlotInfo(const Fus::FusionSlotInfo& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnApp2emap_DR(const Loc::App2emap_DR& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnMapInfo(const Loc::MapInfo& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnObstacles(const od::Obstacles& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSApaPSInfo(const rd::SApaPSInfo& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnQuadParkingSlots(const rd::QuadParkingSlots& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnImage(const rd::Image& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnDecPlan2Emap(const Pla::DecPlan2Emap& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnHMI_InputInfo(const HMI_InputInfo& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSelectSlot(const Sfus::SelectSlot& userData)
{
    VCU_select_ID_ON = userData.SelectSlotID;
    LOGD("[SELECTID] GET VCU ID!!!!")
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnParkInHeadInSwitch(const Sfus::ParkInHeadInSwitch& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSelectSlot2(const Sfus::SelectSlot& userData)
{
    HMI_select_ID = userData.SelectSlotID;
    LOGD("DEBUG1225 KBD HMI select OnSelectSlot2: %d",HMI_select_ID);
    RETURN_NOERROR;
}