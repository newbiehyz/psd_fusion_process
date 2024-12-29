#------------------------------------------------
# Project files
#------------------------------------------------

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


project(psd_fusion_process)
file(GLOB_RECURSE SOURCES "*.*")
include(${CMAKE_SOURCE_DIR}/em_swc/${PROJECT_NAME}/${PROJECT_NAME}.cmake)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
# 移除不需要的文件
list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/psd_fusion/src/unit_test_slotfusion.cpp)

#------------------------------------------------
# Include directories
#------------------------------------------------
#INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/demo/api)  # The header file path of the linked library
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/psd_fusion/include






)  
#------------------------------------------------
# Librarys
#------------------------------------------------
#LINK_DIRECTORIES(${FRAMEWORK_DIR})  # The path of the linked library
#set(LIBRARIES ${LIBRARIES} -ldemo)  # Name of the linked library


#------------------------------------------------
# Final target
#------------------------------------------------
#add_executable(${PROJECT_NAME} ${SOURCES})
#add_library(${PROJECT_NAME} STATIC ${SOURCES})
add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${LIBRARIES})



set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".comp")
#------------------------------------------------
# set property
#------------------------------------------------
set_property(TARGET ${PROJECT_NAME} PROPERTY FOLDER "neo")
set_property(TARGET ${PROJECT_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_DEBUG ${FRAMEWORK_DIR})
set_property(TARGET ${PROJECT_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY_RELEASE ${FRAMEWORK_DIR})
set_property(TARGET ${PROJECT_NAME} PROPERTY ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${FRAMEWORK_DIR})
set_property(TARGET ${PROJECT_NAME} PROPERTY ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${FRAMEWORK_DIR})
set_property(TARGET ${PROJECT_NAME} PROPERTY LIBRARY_OUTPUT_DIRECTORY_DEBUG ${FRAMEWORK_DIR})
set_property(TARGET ${PROJECT_NAME} PROPERTY LIBRARY_OUTPUT_DIRECTORY_RELEASE ${FRAMEWORK_DIR})
#------------------------------------------------
# Install target
#------------------------------------------------
framework_install(${PROJECT_NAME})
sdk_library_install(${PROJECT_NAME})
