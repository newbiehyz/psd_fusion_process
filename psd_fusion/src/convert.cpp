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


int slottype_uss2rd(UssIf_enmSlotType_t uss_type) 
{
    switch (uss_type) 
    {
        case USSIF_SLOT_TYPE_PERPENDICULAR_E:
            return 0; 
        case USSIF_SLOT_TYPE_PARALLEL_E:
            return 1;     
        case USSIF_SLOT_TYPE_ANGULAR_E:
            return 2; 
        case USSIF_SLOT_TYPE_UNKNOW_E:
            break;
    }
}

int slottype_rd2vcu(int rd_type) 
{
    switch (rd_type) 
    {
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