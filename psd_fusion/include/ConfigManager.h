#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <vector>
#include "psd_fusion_process_header.h"
#include "psd_fusion_process_template.h"
#include "apa_define.h"

struct PsdConfig {
    bool debug = false;
    bool faraway_filter = false;
    float faraway_slots_left[2] = {-99999.0f, 0.0f};
    float faraway_slots_right[2] = {0.0f, 99999.0f};
    float faraway_slots_rear = -99999.0f;
    float faraway_slots_front = 99999.0f;
    bool angel_filter = false;
    float angel_filter_limit = -99999.0f;
    bool vcu_too_small_filter = false;
    float vcu_too_small = -99999.0f;
    bool parallel_vector_filter = false;
    float parallel_vector_limit = -99999.0f;
    float narrowslot_threshold = -99999.0f;

    // ... 添加所有其他需要从配置文件加载的参数
};

class ConfigManager {
public:
    static ConfigManager& getInstance(); // Singleton pattern
    bool loadFromFile(const std::string& filename);
    const PsdConfig& getConfig() const;

private:
    ConfigManager() = default; // Private constructor for singleton
    ConfigManager(const ConfigManager&) = delete; // Delete copy constructor
    ConfigManager& operator=(const ConfigManager&) = delete; // Delete assignment operator

    PsdConfig currentConfig;
};

#endif // CONFIG_MANAGER_H