cmake_minimum_required(VERSION 3.10)
project(UnitTestExample)

# 启用 C++17（或其他标准）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 如果在 Linux 平台
add_definitions(-D_ON_LINUX_)

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# 添加源文件
set(SOURCES fusion.cpp)

# 头文件路径
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    /home/gary/Downloads/patac-557e-emos_4.2.3_11211827/patac-557e-emos_4.2.3/patac-557e-emos/CodeRoot/src/psd_fusion_process/psd_fusion/include/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/em_swc/psd_fusion_process/stub/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/include/emc/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/include/emc/c_utils/kernel/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/em_swc/comintf/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/em_swc/comintf/MCore_Bridge/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/em_swc/comintf/perception_fusion_process/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/em_swc/comintf/TROS_Bridge_Parking/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/em_swc/comintf/planning_process/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/em_swc/comintf/SoAd_Bridge/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/src/psd_fusion_process/
    /data/slotfusion/code/patac-557e-emos_4.2.3_fusion/patac-557e-emos/CodeRoot/src/psd_fusion_process/psd_fusion/include/
)

# 添加可执行文件
add_executable(test_executable unit_test_slotfusion.cpp ${SOURCES})


# 链接 Google Test 库
target_link_libraries(test_executable gtest gtest_main pthread ${OpenCV_LIBS})
