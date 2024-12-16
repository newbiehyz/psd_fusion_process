#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"
// input 统一为RD类型
// rd: int, 0 chuizhi 1 shuiping 2 xielie
// uss: enum, 0 weizhi 1chuizhi 2 shuiping 3 xielie

// output 统一为VCU类型
// vcu: int, 0 shuiping 1 chuizhi 2 xielie
// decplan: enum, 0 null 1 chuizhi 2 xielie 3 shuiping
// apahandel: int, 0 shuiping 1 chuizhi 2 xielie 
// statemachine: int, 0 default 1 vertical 2 para 3 diag 4 zixuan


int slottype_uss2rd(UssIf_enmSlotType_t uss_type) 
{
    switch (uss_type) 
    {
        default:
            return 0; //20241125 switch屏蔽异常输入，默认统一输出垂直车位
        case USSIF_SLOT_TYPE_PERPENDICULAR_E:
            return 0; 
        case USSIF_SLOT_TYPE_PARALLEL_E:
            return 1;     
        case USSIF_SLOT_TYPE_ANGULAR_E:
            return 2; 
        case USSIF_SLOT_TYPE_UNKNOW_E:
            return 0; //20241125 switch屏蔽异常输入，默认统一输出垂直车位
    }
}

int slottype_rd2vcu(int rd_type) 
{
    switch (rd_type) 
    {
        default:
            printf("invalid rd_type: %d input!",rd_type);
            return 1;  //20241125 switch屏蔽异常输入，默认统一输出垂直车位
        case 0:
            return 1; 
        case 1:
            return 0; 
        case 2:
            return 2; 
    }
}

Sfus::_tSfusionSlotType slottype_rd2decplan(int rd_type)
{
    switch (rd_type) 
    {
        default:
            return Sfus::SLOTTYP_NULL; 
        case 0:
            return Sfus::SLOTTYP_PER; 
        case 1:
            return Sfus::SLOTTYP_PARA; 
        case 2:
            return Sfus::SLOTTYP_OBL;
    }
}

int decplan2statemachine(Sfus::_tSfusionSlotType decplan_type)
{
    switch (decplan_type) 
    {
        default:
            return 0; 
        case Sfus::SLOTTYP_PER:
            return 1; 
        case Sfus::SLOTTYP_PARA:
            return 2; 
        case Sfus::SLOTTYP_OBL:
            return 3;
    } 
}