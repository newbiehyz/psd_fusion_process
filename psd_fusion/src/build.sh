#!/bin/bash

# 脚本启动
echo "Starting build and run process..."

# 进入 build 目录
if [ ! -d "build" ]; then
    echo "Build directory not found. Creating it..."
    mkdir build
fi
cd build

# 清理 build 目录
echo "Cleaning build directory..."
rm -rf *

# 运行 CMake 配置
echo "Running CMake..."
cmake ..

# 编译项目
echo "Building project..."
make

# 检查是否生成了可执行文件
if [ -f "./test_executable" ]; then
    echo "Build successful. Running the executable..."
    # 运行可执行文件
    ./test_executable
else
    echo "Build failed. Executable not found."
    exit 1
fi
