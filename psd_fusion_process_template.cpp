#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "PSD_FusionModuleIF.h"
#include <iostream>
#include <typeinfo>

#define DEBUG false

#define VEHICLE_LENGTH 5259.9 
#define REAR_AXLE_CENTER_VEHICLE_REAR 1136.7 
#define MM_TO_M 1000

int apa_states;
StatusDecFusionInput psd2statemachine;
Sfus::Sfsuion2DecPlan psd2planning;
bool select_flag = false;
int VCU_select_ID = 0;


CDT_PSD_FUSION_PROCESS_TEMPLATE(cpsd_fusion_process)

cpsd_fusion_process::cpsd_fusion_process()
{

}

cpsd_fusion_process::~cpsd_fusion_process()
{

}

tResult cpsd_fusion_process::Init()
{
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

PSD_FusionModuleIF PSD_FusionModuleIFrunable;
Sfus::SelectSlot VCU_select;

tResult cpsd_fusion_process::TimeTrigger_Timer50()
{
    //PSD_FusionModuleIFrunable.Initialize()只运行一次
    static int init_flag = 0;
    if(init_flag == 0)
    {
        //updatevisionslots 初始化
        PSD_FusionModuleIFrunable.Initialize();
        init_flag = 1;

        //上下游 初始化
        psd2statemachine.aps_apaParkType = 0;
        psd2statemachine.aps_apaAvailableSlot = 0;
        psd2statemachine.aps_apaHighlightSlot = 0;
        psd2statemachine.aps_apaNarrowSlot = 0;
        psd2statemachine.aps_apaParkFusionType = 0;
        psd2statemachine.aps_apaParkPlaceNum = 0;

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
    LOGT("[_test init] start!")

    // 上游：statemachine, RD, DR

    //***********get frameid and singleframeslot
    //frameid
    rd::QuadParkingSlots rd_info;
    EMC_TROS_Bridge_Parking_GetFieldQuadParkingSlots(rd_info); //rd::Header, rd::QuadParkingSlot
    if (DEBUG == true){
        filetojson.SaveQuadParkingSlotsInfoToJson(rd_info, "RDinfo.json");
    }
    
    LOGT("[_test rd_info timestampNs] J5 SEND timestampNs: %llu",rd_info.frameTimeStampNs);
    //emos -> 358-2
    //@TODO 时间戳作为frameid的risk
    unsigned long long singleframeslotsID;
    singleframeslotsID = rd_info.frameTimeStampNs;
    LOGT("[_test rd_info timestampNs] S32G RECEIVE timestampNs: %llu",singleframeslotsID);

    //singleframeslot
    std::vector<padVisionSlotCoord> singleframeslots;
    if (!rd_info.quadParkingSlotList.empty())
    {
        LOGT("[_test rd_info singleframeslots] J5 SEND RD output slots size: %d",rd_info.quadParkingSlotList.size());
        for (const auto& parkingSlot : rd_info.quadParkingSlotList)
        {
            LOGT("[_test rd_info singleframeslots] J5 SEND slottype(chuizhi0shuiping1xiexiang2): %d, label(0buzhanyong): %u",parkingSlot.slotType,parkingSlot.label);
            LOGT("[_test rd_info singleframeslots] J5 SEND tl:(%f,%f), bl:(%f,%f), tr:(%f,%f), br:(%f,%f)",parkingSlot.tl.x,parkingSlot.tl.y,parkingSlot.bl.x,parkingSlot.bl.y,
            parkingSlot.tr.x,parkingSlot.tr.y,parkingSlot.br.x,parkingSlot.br.y);

            //emos -> 358-2
            padVisionSlotCoord oneslot; //中间结构体，转存rd单帧车位列表

            // 0xFF 作为默认值
            oneslot.bayType = (parkingSlot.slotType == 0) ? 0x00 : (parkingSlot.slotType == 1) ? 0x01 : 0xFF;  
            
            //左右判断,按规划/定位ABCD顺序输出车位角点
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
                LOGT("[_test rd_info singleframeslots] S32G RECEIVE LEFT SLOTS tl:(%d,%d), tr:(%d,%d), br:(%d,%d), bl:(%d,%d)",oneslot.b.x,oneslot.b.y,oneslot.a.x,oneslot.a.y,
            oneslot.d.x,oneslot.d.y,oneslot.c.x,oneslot.c.y);
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
                LOGT("[_test rd_info singleframeslots] S32G RECEIVE RIGHT SLOTS tl:(%d,%d), tr:(%d,%d), br:(%d,%d), bl:(%d,%d)",oneslot.a.x,oneslot.a.y,oneslot.b.x,oneslot.b.y,
            oneslot.c.x,oneslot.c.y,oneslot.d.x,oneslot.d.y);
            }
            singleframeslots.push_back(oneslot);
        }
    }
    else
    {
        LOGT("[_test rd_info singleframeslots] no parking slots received")
    }
    
    //***********get dr
    Loc::App2emap_DR dr_pose;
    EMC_TROS_Bridge_Parking_GetFieldApp2emap_DR(dr_pose); //Loc::App2emap_DR
    LOGT("[_test dr_pose] J5 SEND x: %f, y: %f, yaw: %f, timestamp: %llu",dr_pose.x, dr_pose.y, dr_pose.canAng,dr_pose.timeStamp);
    //emos -> 358-2 @TODO dr_pose long int
    if (DEBUG == true){
        filetojson.SaveDRInfoToJson(dr_pose, "DR_POSE.json");
    }
    padVehiclePose  pose_globaldata;
    pose_globaldata.coord.x = int(dr_pose.x);
    pose_globaldata.coord.y = int(dr_pose.y);
    pose_globaldata.yaw = dr_pose.canAng;
    pose_globaldata.timestamps = dr_pose.timeStamp;
    LOGT("[_test dr_pose] S32G RECEIVE x:%d, y: %d, yaw: %f",pose_globaldata.coord.x, pose_globaldata.coord.y, pose_globaldata.yaw);

    //@TODO 进入search才开始车位融合
    //if (apa_states == 2) {
    LOGT("[_test update dr begin]");
    PSD_FusionModuleIFrunable.UpdateVechiclePose(pose_globaldata);

    // @TODO 多帧滤波
    if (apa_states != 5){
        PSD_FusionModuleIFrunable.UpdateVisionSlots(singleframeslotsID, singleframeslots);
    }
    
    //输出车位列表
    apaSlotListInfo outputSlot_VIS = PSD_FusionModuleIFrunable.GetOutputSlot();
    
    if (DEBUG == true){
        filetojson.SaveapaSlotListInfoToJson(outputSlot_VIS,"VISapaSlotListInfo.json");
    }
    
    LOGT("[_test apastatus]: %d", apa_states);
    // 当泊车完成或者中断，清空车位列表
    if (apa_states == 6 || apa_states == 7){
        outputSlot_VIS.WorldoutRect.clear();
    }

    //psd输出的原始车位列表（原点为后轴中心）
    LOGT("[_test psd output origin slotlist] size is %d",outputSlot_VIS.WorldoutRect.size());

    for (auto& psd_m_output : outputSlot_VIS.WorldoutRect)
    {
        LOGD("[_test psd output origin slotlist] m_output_slot list: ");
        LOGD("ID: %d, type: %d, occupied: %d", 
        psd_m_output.rectInfo.label,psd_m_output.rectInfo.PStype,psd_m_output.rectInfo.iSodType);
        for (int i = 0; i < RECTPointNum; ++i) 
        {
            LOGD(" (%d,%d)",psd_m_output.rectInfo.pt[i].x,psd_m_output.rectInfo.pt[i].y);
        }
        LOGD("\n");
    }


    // 下游：APAHANDEL, PERCEPTION, VCU, PLANNING

    //***********APAHANDEL (ready)
    Fsm::FusionSlotInfo2Location psd2location;
    int tempsize;
    tempsize = outputSlot_FUSED.WorldoutRect.size();
    psd2location.slotNum = tempsize;
    LOGT("[_test] tempsize:%d, slotnum: %d",tempsize,psd2location.slotNum);
    if (psd2location.slotNum > 0)
    {
        int j = 0;

        for (auto& psd_m_output : outputSlot_FUSED.WorldoutRect)
        {
            if (j >= tempsize)
            {
                break;
            }
            psd2location.fusionSlotInfo[j].slotType = slottype_rd2vcu(psd_m_output.rectInfo.PStype);
            psd2location.fusionSlotInfo[j].slotLabel = psd_m_output.rectInfo.label; //ID
            psd2location.fusionSlotInfo[j].pt[0].x = psd_m_output.rectInfo.pt[0].x;
            psd2location.fusionSlotInfo[j].pt[0].y = psd_m_output.rectInfo.pt[0].y;
            psd2location.fusionSlotInfo[j].pt[1].x = psd_m_output.rectInfo.pt[1].x;
            psd2location.fusionSlotInfo[j].pt[1].y = psd_m_output.rectInfo.pt[1].y;
            psd2location.fusionSlotInfo[j].pt[2].x = psd_m_output.rectInfo.pt[2].x;
            psd2location.fusionSlotInfo[j].pt[2].y = psd_m_output.rectInfo.pt[2].y;
            psd2location.fusionSlotInfo[j].pt[3].x = psd_m_output.rectInfo.pt[3].x;
            psd2location.fusionSlotInfo[j].pt[3].y = psd_m_output.rectInfo.pt[3].y;


            LOGT("[SlotFusion location] TOTAL SLOT NUM: %d, Slot#%d, type: %d ",
                    psd2location.slotNum,
                    psd2location.fusionSlotInfo[j].slotLabel,
                    psd2location.fusionSlotInfo[j].slotType);
            LOGT("(%d, %d , %d, %d, %d, %d, %d, %d)\n",
                    psd2location.fusionSlotInfo[j].pt[0].x,
                    psd2location.fusionSlotInfo[j].pt[0].y,
                    psd2location.fusionSlotInfo[j].pt[1].x,
                    psd2location.fusionSlotInfo[j].pt[1].y,
                    psd2location.fusionSlotInfo[j].pt[2].x,
                    psd2location.fusionSlotInfo[j].pt[2].y,
                    psd2location.fusionSlotInfo[j].pt[3].x,
                    psd2location.fusionSlotInfo[j].pt[3].y);
            j++;

        }
        EMC_psd_fusion_process_SetFieldFusionSlotInfo2Location(psd2location);
    }

    //**********PERCEPTION
    Sfus::SfusionSlots psd2perception;
    int perception_slotnum = outputSlot_FUSED.WorldoutRect.size();
    int m = 0;
    for (auto& psd_m_output : outputSlot_FUSED.WorldoutRect)
    {
        if (m >= perception_slotnum)
        {
            break;
        }
        psd2perception.slotID = psd_m_output.rectInfo.label;
        psd2perception.slotType = slottype_rd2decplan(psd_m_output.rectInfo.PStype);
        psd2perception.slotSource = Sfus::SLOTSRC_VIS;
        psd2perception.slotCorners.cornerA.x = psd_m_output.rectInfo.pt[0].x;
        psd2perception.slotCorners.cornerA.y = psd_m_output.rectInfo.pt[0].y;
        psd2perception.slotCorners.cornerB.x = psd_m_output.rectInfo.pt[1].x;
        psd2perception.slotCorners.cornerB.y = psd_m_output.rectInfo.pt[1].y;
        psd2perception.slotCorners.cornerC.x = psd_m_output.rectInfo.pt[2].x;
        psd2perception.slotCorners.cornerC.y = psd_m_output.rectInfo.pt[2].y;
        psd2perception.slotCorners.cornerD.x = psd_m_output.rectInfo.pt[3].x;
        psd2perception.slotCorners.cornerD.y = psd_m_output.rectInfo.pt[3].y;

        LOGT("[SlotFusion perception] TOTAL SLOT NUM: %d, Slot#%d, type: %d, source: %d ",
                perception_slotnum,
                psd2perception.slotID,
                psd2perception.slotType,
                psd2perception.slotSource);
        LOGT("(%f, %f , %f, %f, %f, %f, %f, %f)",
                psd2perception.slotCorners.cornerA.x,
                psd2perception.slotCorners.cornerA.y,
                psd2perception.slotCorners.cornerB.x,
                psd2perception.slotCorners.cornerB.y,
                psd2perception.slotCorners.cornerC.x,
                psd2perception.slotCorners.cornerC.y,
                psd2perception.slotCorners.cornerD.x,
                psd2perception.slotCorners.cornerD.y);
        m++;
    }
    printf("\n");
    EMC_psd_fusion_process_SetFieldSfusionSlots(psd2perception);

    //**********VCU
    Sfus::FusionSlotInfovector psd2vcu;
    memset(&psd2vcu, 0, sizeof(Sfus::FusionSlotInfovector));
    psd2vcu.slotNum = tempsize;
    if (psd2vcu.slotNum > 0){
        int i = 0;

        for (auto& psd_m_output : outputSlot_FUSED.WorldoutRect){
            if (i >= tempsize){
                break;
            }
            //psd2vcu.FusionSlotInfo[i].slotType = slottype_rd2vcu(psd_m_output.rectInfo.PStype);
            psd2vcu.FusionSlotInfo[i].slotLabel = psd_m_output.rectInfo.label; //ID
            psd2vcu.FusionSlotInfo[i].displayLabel = psd2vcu.FusionSlotInfo[i].slotLabel;
            psd2vcu.FusionSlotInfo[i].slotStatusType = 3;

            //ABCD顺序调整为VCU专用顺序
            //左侧
            if (psd_m_output.rectInfo.pt[0].x <= 0 || psd_m_output.rectInfo.pt[1].x <= 0 || psd_m_output.rectInfo.pt[2].x < 0){
                psd2vcu.FusionSlotInfo[i].pt[0].x = (psd_m_output.rectInfo.pt[1].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) ) / MM_TO_M; //mm 转 m , VCU坐标系上x右y
                psd2vcu.FusionSlotInfo[i].pt[0].y = psd_m_output.rectInfo.pt[1].x / MM_TO_M; //后轴中心转前保中心
                psd2vcu.FusionSlotInfo[i].pt[0].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[1].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) ) / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[1].y = psd_m_output.rectInfo.pt[0].x/ MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[1].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[2].x = (psd_m_output.rectInfo.pt[3].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[2].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[2].x/ MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                psd2vcu.FusionSlotInfo[i].backInAvailableFlag = 0;
                psd2vcu.FusionSlotInfo[i].parkInHeadInSoftButtonCurrentValue = 0;
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
                psd2vcu.FusionSlotInfo[i].pt[2].y = psd_m_output.rectInfo.pt[3].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[2].z = 0;
                psd2vcu.FusionSlotInfo[i].pt[3].x = (psd_m_output.rectInfo.pt[0].y - (VEHICLE_LENGTH - REAR_AXLE_CENTER_VEHICLE_REAR) )/ MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].y = psd_m_output.rectInfo.pt[0].x / MM_TO_M;
                psd2vcu.FusionSlotInfo[i].pt[3].z = 0;
                psd2vcu.FusionSlotInfo[i].backInAvailableFlag = 0;
                psd2vcu.FusionSlotInfo[i].parkInHeadInSoftButtonCurrentValue = 0;
            }
            LOGT("[SlotFusion vcu] TOTAL SLOT NUM: %d, Slot#%d, Displaylabel: %d, Slotstatus: %d A(%.3f, %.3f) B(%.3f, %.3f) C(%.3f, %.3f) D(%.3f, %.3f)",
                    psd2vcu.slotNum,
                    psd2vcu.FusionSlotInfo[i].slotLabel,
                    psd2vcu.FusionSlotInfo[i].displayLabel,
                    psd2vcu.FusionSlotInfo[i].slotStatusType,
                    psd2vcu.FusionSlotInfo[i].pt[0].x,
                    psd2vcu.FusionSlotInfo[i].pt[0].y,
                    psd2vcu.FusionSlotInfo[i].pt[1].x,
                    psd2vcu.FusionSlotInfo[i].pt[1].y,
                    psd2vcu.FusionSlotInfo[i].pt[2].x,
                    psd2vcu.FusionSlotInfo[i].pt[2].y,
                    psd2vcu.FusionSlotInfo[i].pt[3].x,
                    psd2vcu.FusionSlotInfo[i].pt[3].y);
            i++;
        }
        EMC_psd_fusion_process_SetFieldFusionSlotInfovector(psd2vcu);
    }

    //***********PLANNING, HMI（ready）
    //check psd output to planning(1 slot list)  
    //规划暂时不用车位列表，需等待预规划模块ready后。暂时用于HMI显示车位列表
    int planning_slotnum = outputSlot_FUSED.WorldoutRect.size();
    int k = 0;
    for (auto& psd_m_output : outputSlot_FUSED.WorldoutRect)
    {
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

        LOGT("[SlotFusion planning] TOTAL SLOT NUM: %d, Slot#%d, type: %d, source: %d ",
                planning_slotnum,
                psd2planning.SfusionSrchSlots[k].slotID,
                psd2planning.SfusionSrchSlots[k].slotType,
                psd2planning.SfusionSrchSlots[k].slotSource);
        LOGT("(%.1f, %.1f , %.1f, %.1f, %.1f, %.1f, %.1f, %.1f)\n",
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
    printf("\n");
    EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);


    //用于接受HMI发送的选择车位ID。HMI只发送1s，跳转为0。通过aps_apaParkType使statemachine跳转
    Fsm::Slotlabel psd2apahandel_targetID;
    //memset(&psd2apahandel_targetID, 0, sizeof(Fsm::Slotlabel));

    if (apa_states == 6 || apa_states == 7){
        psd2statemachine.aps_apaParkType = 0;
        VCU_select_ID = VCU_select.SelectSlotID;
        // VCU_select_ID = VCU_select.SelectSlotID; //考虑用VCU_select_ID = 0
        // memset(&psd2apahandel_targetID, 0, sizeof(Fsm::Slotlabel));
        LOGT("select target slot clear!")
    }

    //VCU - s32g
    if (VCU_select.SelectSlotID)
    {
        VCU_select_ID = VCU_select.SelectSlotID;
        psd2statemachine.aps_apaParkType = 1;
    }
    //print target slot info
    if (VCU_select_ID > 0)
    {
        psd2apahandel_targetID.targetSlotLabel = VCU_select_ID;
        EMC_psd_fusion_process_SetFieldSlotlabel(psd2apahandel_targetID);

        // @TODO states == 7 时 core dump
        // printf("[_test select target slot] #%d: ",VCU_select_ID);
        //     for (int i = 0; i < RECTPointNum; ++i) 
        //     {
        //         printf(" (%d,%d)",outputSlot_VIS.WorldoutRect[VCU_select_ID-1].rectInfo.pt[i].x,outputSlot_VIS.WorldoutRect[VCU_select_ID-1].rectInfo.pt[i].y - 4123.2);
        //     }
        //     printf("\n");
    }
    LOGT("[_test S32G RECEIVE VCU] select slot ID is: #%d, %d",VCU_select_ID,VCU_select_ID-1);
    LOGT("[_test statemachine] send slottype:%d", psd2statemachine.aps_apaParkType);

    
    S2S_MCore_Bridge_SetSigStatusDecFusionInput(&psd2statemachine);


    //check psd output to planning(2 target slot)
    //拿到目标车位ID后，发送目标车位信息给planning
    if (VCU_select_ID > 0)
    {
        for (int i = 0; i < outputSlot_FUSED.WorldoutRect.size();i++){
            if (VCU_select_ID == outputSlot_FUSED.WorldoutRect[i].rectInfo.label){
                psd2planning.targetSlot.slotCorners.cornerA.x = outputSlot_FUSED.WorldoutRect[i].rectInfo.pt[0].x; 
                psd2planning.targetSlot.slotCorners.cornerA.y = outputSlot_FUSED.WorldoutRect[i].rectInfo.pt[0].y;
                psd2planning.targetSlot.slotCorners.cornerB.x = outputSlot_FUSED.WorldoutRect[i].rectInfo.pt[1].x;
                psd2planning.targetSlot.slotCorners.cornerB.y = outputSlot_FUSED.WorldoutRect[i].rectInfo.pt[1].y;
                psd2planning.targetSlot.slotCorners.cornerC.x = outputSlot_FUSED.WorldoutRect[i].rectInfo.pt[2].x;
                psd2planning.targetSlot.slotCorners.cornerC.y = outputSlot_FUSED.WorldoutRect[i].rectInfo.pt[2].y;
                psd2planning.targetSlot.slotCorners.cornerD.x = outputSlot_FUSED.WorldoutRect[i].rectInfo.pt[3].x;
                psd2planning.targetSlot.slotCorners.cornerD.y = outputSlot_FUSED.WorldoutRect[i].rectInfo.pt[3].y;
                psd2planning.targetSlot.slotType = slottype_rd2decplan(outputSlot_FUSED.WorldoutRect[i].rectInfo.PStype);
                psd2planning.targetSlot.slotSource = Sfus::SLOTSRC_VIS;
            }
        }
        

        LOGT("[_test psd2planning Corners] target slot: TYPE: %d, SOURCE: %d (%f,%f) (%f,%f) (%f,%f) (%f,%f)",
                psd2planning.targetSlot.slotType,
                psd2planning.targetSlot.slotSource,
                psd2planning.targetSlot.slotCorners.cornerA.x,
                psd2planning.targetSlot.slotCorners.cornerA.y,
                psd2planning.targetSlot.slotCorners.cornerB.x,
                psd2planning.targetSlot.slotCorners.cornerB.y,
                psd2planning.targetSlot.slotCorners.cornerC.x,
                psd2planning.targetSlot.slotCorners.cornerC.y,
                psd2planning.targetSlot.slotCorners.cornerD.x,
                psd2planning.targetSlot.slotCorners.cornerD.y);
    }
    EMC_psd_fusion_process_SetFieldSfsuion2DecPlan(psd2planning);

    // @TODO 泊车过程是否完成或者失败：重新初始化；

    RETURN_NOERROR;
}

tResult cpsd_fusion_process::TimeTrigger_Timer100()
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnVehicleCanData(const VehicleCanData& userData)
{
    RETURN_NOERROR;
}


tResult cpsd_fusion_process::OnStatusDecOutput(const StatusDecOutput& userData)
{
    apa_states = userData.aps_apaStatusReq;
    RETURN_NOERROR;
}

apaSlotListInfo outputSlot_USS;
UssIf_stSlotInfo_t PLV_total_slots;

tResult cpsd_fusion_process::OnUssIf_stPLVOutputInfo(const UssIf_stPLVOutputInfo_t& userData)
{
    //***打印PLV原始输出
    LOGT("[SlotFusion USS PLV] LEFT slot num: %d",userData.UssIf_stSlotInfo[0].u8SlotNum);
    for (int i = 0; i < userData.UssIf_stSlotInfo[0].u8SlotNum; ++i)
    {
        LOGT("[SlotFusion USS PLV] LEFT slot ID: %d, slot type: %d, slot bottom type: %d, slot length: %d, depth: %d, (%d, %d) (%d, %d) (%d, %d) (%d, %d)"
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].u16SlotID
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].enmSlotType
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].enmSlotBottomType
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].u16SlotLength
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].u16SlotDepth
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[0].x
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[0].y
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[1].x
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[1].y
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[2].x
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[2].y
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[3].x
                ,userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[3].y
            );
    }
    LOGT("[SlotFusion USS PLV] RIGHT slot num: %d",userData.UssIf_stSlotInfo[1].u8SlotNum);
    for (int i = 0; i < userData.UssIf_stSlotInfo[1].u8SlotNum; ++i)
    {
        LOGT("[SlotFusion USS PLV] RIGHT slot ID: %d, slot type: %d, slot bottom type: %d, slot length: %d, depth: %d, (%d, %d, %d, %d, %d, %d, %d, %d)"
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].u16SlotID
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].enmSlotType
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].enmSlotBottomType
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].u16SlotLength
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].u16SlotDepth
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].stSlotPt[0].x
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].stSlotPt[0].y
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].stSlotPt[1].x
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].stSlotPt[1].y
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].stSlotPt[2].x
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].stSlotPt[2].y
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].stSlotPt[3].x
                ,userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[i].stSlotPt[3].y
            );
    }

    //左右车位列表拼成一个
    int USS_total_slotnum = 0;
    USS_total_slotnum = userData.UssIf_stSlotInfo[0].u8SlotNum + userData.UssIf_stSlotInfo[1].u8SlotNum;
    PLV_total_slots.u8SlotNum = USS_total_slotnum;
    if (PLV_total_slots.u8SlotNum > 0)
    {
        for (int USS_index = 0; USS_index < USS_total_slotnum; USS_index++)
        {
            for (int i = 0; i < userData.UssIf_stSlotInfo[0].u8SlotNum; i++)
            {
                PLV_total_slots.UssIf_stSlotProperty[USS_index].u16SlotID = 10000 + USS_index;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].enmSlotType = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].enmSlotType;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].enmSlotBottomType = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].enmSlotBottomType;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].u16SlotDepth = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].u16SlotDepth;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].u16SlotLength = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].u16SlotLength;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[0].x = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[0].x;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[0].y = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[0].y;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[1].x = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[1].x;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[1].y = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[1].y;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[2].x = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[2].x;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[2].y = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[2].y;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[3].x = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[3].x;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[3].y = userData.UssIf_stSlotInfo[0].UssIf_stSlotProperty[i].stSlotPt[3].y;
                USS_index++;
            }
            for (int j = 0; j < userData.UssIf_stSlotInfo[1].u8SlotNum; j++)
            {
                PLV_total_slots.UssIf_stSlotProperty[USS_index].u16SlotID = 10000 + USS_index;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].enmSlotType = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].enmSlotType;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].enmSlotBottomType = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].enmSlotBottomType;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].u16SlotDepth = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].u16SlotDepth;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].u16SlotLength = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].u16SlotLength;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[0].x = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].stSlotPt[0].x;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[0].y = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].stSlotPt[0].y;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[1].x = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].stSlotPt[1].x;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[1].y = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].stSlotPt[1].y;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[2].x = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].stSlotPt[2].x;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[2].y = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].stSlotPt[2].y;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[3].x = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].stSlotPt[3].x;
                PLV_total_slots.UssIf_stSlotProperty[USS_index].stSlotPt[3].y = userData.UssIf_stSlotInfo[1].UssIf_stSlotProperty[j].stSlotPt[3].y;
                USS_index++;
            }
        }
    }

    // //@TODO 构建规则，缺少角度，缺少斜列车位构建
    // if(PLV_total_slots.u8SlotNum  != 0){
    //     for (auto& uss_oneslot : PLV_total_slots.UssIf_stSlotProperty){
    //         if(uss_oneslot.enmSlotType == 1){
    //             if (uss_oneslot.u16SlotLength <= 400)
    //             {
    //                 uss_oneslot.stSlotPt[0].x = uss_oneslot.stSlotPt[0].x;
    //                 uss_oneslot.stSlotPt[0].y = uss_oneslot.stSlotPt[0].y + 300;
    //                 uss_oneslot.stSlotPt[1].x = uss_oneslot.stSlotPt[1].x;
    //                 uss_oneslot.stSlotPt[1].y = uss_oneslot.stSlotPt[1].y - 300;
    //                 uss_oneslot.stSlotPt[2].x = uss_oneslot.stSlotPt[1].x + 6000;
    //                 uss_oneslot.stSlotPt[2].y = uss_oneslot.stSlotPt[1].y;
    //                 uss_oneslot.stSlotPt[3].x = uss_oneslot.stSlotPt[0].x + 6000;
    //                 uss_oneslot.stSlotPt[3].y = uss_oneslot.stSlotPt[0].y;
    //             }
    //         }

    //     }
        

    // } 
    

    // 与视觉车位类型统一的PLV车位列表
    // @TODO 数据类型统一，含义统一
    outputSlot_USS.WorldoutRect.clear();
    if(PLV_total_slots.u8SlotNum  != 0){
        for (int i = 0; i < USS_total_slotnum; ++i) 
        {
            UssIf_stSlotProperty_t slotProperty = PLV_total_slots.UssIf_stSlotProperty[i];
            
            apaSlotInfo slotInfo;
            APA_SPACE::SApaPSRect& rectInfo = slotInfo.rectInfo;

            for (int j = 0; j < RECTPointNum; ++j)
            {
                rectInfo.pt[j].x = slotProperty.stSlotPt[j].x;
                rectInfo.pt[j].y = slotProperty.stSlotPt[j].y;
            }

            rectInfo.label = slotProperty.u16SlotID; //USS slot ID从10000开始
            rectInfo.PStype = slottype_uss2rd(slotProperty.enmSlotType);
            rectInfo.iSodType = slotProperty.enmInSlotObstacleStatus;
            rectInfo.iDownSlotSOD = slotProperty.enmDownSlotSODType;
            rectInfo.iMinOtherSideDist = slotProperty.u16UssOppositeSpace;
            rectInfo.iRoadEdgeDist = slotProperty.u16ObjDistanceBetweenLineABToSlotBottom;
            
            outputSlot_USS.WorldoutRect.push_back(slotInfo);
            LOGT("SlotFusionUSS push back time: %d",i);
        }

        if (DEBUG == true){
            filetojson.SaveapaSlotListInfoToJson(outputSlot_USS,"USSapaSlotListInfo.json");
            filetojson.SaveapaSlotListInfoToJson(outputSlot_FUSED,"FusedapaSlotListInfo.json");
        }
        
        // 打印USS车位列表
        for (auto& psd_m_output : outputSlot_USS.WorldoutRect)
        {
            LOGT("[SlotFusionUSS left and right] ID: %d, type: %d, occupied: %d", 
                    psd_m_output.rectInfo.label,psd_m_output.rectInfo.PStype,psd_m_output.rectInfo.iSodType);
            for (int i = 0; i < RECTPointNum; ++i) 
            {
                LOGT(" (%d,%d)",psd_m_output.rectInfo.pt[i].x,psd_m_output.rectInfo.pt[i].y);
            }
            printf("\n");
        }
    }


     //***融合
    slotfusion fusionslot;
    outputSlot_VIS = PSD_FusionModuleIFrunable.GetOutputSlot();
    outputSlot_USS.ullFrameId = outputSlot_VIS.ullFrameId;
    fusionslot.mergeSlotLists(outputSlot_USS, outputSlot_VIS , outputSlot_FUSED);
   

    // //打印融合车位列表
    // for (auto& psd_m_output : outputSlot_FUSED.WorldoutRect)
    // {
    //     printf("[_test psd output fused slotlist] ID: %d, type: %d, occupied: %d", 
    //     psd_m_output.rectInfo.label,psd_m_output.rectInfo.PStype,psd_m_output.rectInfo.iSodType);
    //     for (int i = 0; i < RECTPointNum; ++i) 
    //     {
    //         printf(" (%d,%d)",psd_m_output.rectInfo.pt[i].x,psd_m_output.rectInfo.pt[i].y);
    //     }
    //     printf("\n");
    // }


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

tResult cpsd_fusion_process::OnPkEmapObs(const Fus::PkEmapObs& userData)
{
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
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnParkInHeadInSwitch(const Sfus::ParkInHeadInSwitch& userData)
{
    RETURN_NOERROR;
}

tResult cpsd_fusion_process::OnSelectSlot2(const Sfus::SelectSlot& userData)
{
    VCU_select.SelectSlotID = userData.SelectSlotID;
    RETURN_NOERROR;
}