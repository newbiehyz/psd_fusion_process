#include "ConfigManager.h"
#include <fstream>
#include <sstream>
#include "save_to_json.h"


ConfigManager& ConfigManager::getInstance() {
    static ConfigManager instance;
    return instance;
}

bool ConfigManager::loadFromFile(const std::string& filename) {
    std::ifstream inFile(filename);
    if (!inFile.is_open()) {
        currentConfig.debug = false;
        currentConfig.faraway_filter = false;
        LOGD("[ERROR] unable to load config: %s", filename.c_str());
        return false;
    }

    try {
        nlohmann::json j;
        inFile >> j;

        j.at("debug").at("save_to_json").get_to(currentConfig.debug);

        j.at("calib").at("FARAWAY_FILTER").get_to(currentConfig.faraway_filter);
        auto faraway_slots_left_range = j.at("calib").at("FARAWAY_SLOTS_LEFT");
        currentConfig.faraway_slots_left[0] = faraway_slots_left_range[0];
        currentConfig.faraway_slots_left[1] = faraway_slots_left_range[1];
        auto faraway_slots_right_range = j.at("calib").at("FARAWAY_SLOTS_RIGHT");
        currentConfig.faraway_slots_right[0] = faraway_slots_right_range[0];
        currentConfig.faraway_slots_right[1] = faraway_slots_right_range[1];
        j.at("calib").at("FARAWAY_SLOTS_REAR").get_to(currentConfig.faraway_slots_rear);
        j.at("calib").at("FARAWAY_SLOTS_FRONT").get_to(currentConfig.faraway_slots_front);

        j.at("calib").at("ANGEL_FILTER").get_to(currentConfig.angel_filter);
        j.at("calib").at("ANGEL_FILTER_LIMIT").get_to(currentConfig.angel_filter_limit);

        j.at("calib").at("VCU_TOO_SMALL_FILTER").get_to(currentConfig.vcu_too_small_filter);
        j.at("calib").at("VCU_TOO_SMALL").get_to(currentConfig.vcu_too_small);

        j.at("calib").at("PARALLEL_VECTOR_FILTER").get_to(currentConfig.parallel_vector_filter);
        j.at("calib").at("PARALLEL_VECTOR_LIMIT").get_to(currentConfig.parallel_vector_limit);

        j.at("calib").at("NARROWSLOT_THRESHOLD").get_to(currentConfig.narrowslot_threshold);
    } catch (const nlohmann::json::exception& e) {
        LOGE("[Warning] config parsing error: %s", e.what());
        inFile.close();
        return false;
    }
    inFile.close();
    return true;
}

const PsdConfig& ConfigManager::getConfig() const {
    return currentConfig;
}