#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include "json.hpp"

#include "apa_define.h"

using json = nlohmann::json;

class SaveFileToJson{
 public:
    void SaveapaSlotListInfoToJson(apaSlotListInfo &info, const std::string &filename, json& j);
};

