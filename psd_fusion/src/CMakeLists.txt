cmake_minimum_required(VERSION 3.10)
project(UnitTestExample)

# 启用 C++17（或其他标准）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

set(DEFAULT_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "{CMAKE_CXX_FLAGS} -o0 -ggbd")
add_compile_options(-g)

# 查找 Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 定义项目根目录变量
set(PROJECT_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../../..)

# 如果在 Linux 平台
add_definitions(-D_ON_LINUX_)

# 查找 OpenCV
find_package(OpenCV REQUIRED)

# 添加源文件
set(SOURCES 
    fusion.cpp
    PSD_FusionModuleIF.cpp
    iou.cpp
    Kalman_filter.cpp
    kd_tree.cpp
    convert.cpp
    )

# 头文件路径
include_directories(
    ${OpenCV_INCLUDE_DIRS}
     ${PROJECT_ROOT_DIR}/CodeRoot/src/psd_fusion_process/psd_fusion/include/
     ${PROJECT_ROOT_DIR}/CodeRoot/em_swc/psd_fusion_process/stub/
     ${PROJECT_ROOT_DIR}/CodeRoot/include/emc/
     ${PROJECT_ROOT_DIR}/CodeRoot/include/emc/c_utils/kernel/
     ${PROJECT_ROOT_DIR}/CodeRoot/em_swc/comintf/
     ${PROJECT_ROOT_DIR}/CodeRoot/em_swc/comintf/MCore_Bridge/
     ${PROJECT_ROOT_DIR}/CodeRoot/em_swc/comintf/perception_fusion_process/
     ${PROJECT_ROOT_DIR}/CodeRoot/em_swc/comintf/TROS_Bridge_Parking/
     ${PROJECT_ROOT_DIR}/CodeRoot/em_swc/comintf/planning_process/
     ${PROJECT_ROOT_DIR}/CodeRoot/em_swc/comintf/SoAd_Bridge/
     ${PROJECT_ROOT_DIR}/CodeRoot/src/psd_fusion_process/
     ${PROJECT_ROOT_DIR}/CodeRoot/src/psd_fusion_process/psd_fusion/include/
     ${PROJECT_ROOT_DIR}/CodeRoot/em_swc/comintf/Parking_HMI/
)

# 添加可执行文件
add_executable(test_executable unit_test_slotfusion.cpp ${SOURCES})


# 链接 Google Test 库
target_link_libraries(test_executable gtest gtest_main pthread ${OpenCV_LIBS})
# 包含 Eigen3 头文件
target_include_directories(test_executable PRIVATE ${EIGEN3_INCLUDE_DIR})
