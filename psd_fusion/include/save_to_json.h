#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "json.hpp"
#include "psd_fusion_process_header.h"
#include "apa_define.h"

using json = nlohmann::json;

class SaveFileToJson{
 public:
    void SaveapaSlotListInfoToJson(apaSlotListInfo &info, const std::string &filename);
    void SaveQuadParkingSlotsInfoToJson(rd::QuadParkingSlots &info,const std::string &filename);
    void SaveDRInfoToJson(Loc::App2emap_DR &info,const std::string &filename);
    void SaveObsToJson(Fus::PkEmapObs &info,const std::string &filename);
};

