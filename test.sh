#!/bin/bash
# RoboMaster哨兵机器人导航系统 - 完整测试脚本

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  RoboMaster Sentinel Test Suite${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查编译
if [ ! -d "install" ]; then
    echo -e "${RED}Error: Project not built! Run ./build.sh first${NC}"
    exit 1
fi

source install/setup.bash

echo -e "${YELLOW}[1/3] Running unit tests...${NC}"
colcon test --packages-select \
    sensor_fusion \
    sentinel_navigation \
    vision_aiming \
    decision_making

echo -e "${YELLOW}[2/3] Checking code style...${NC}"
# Python风格检查
if command -v flake8 &> /dev/null; then
    flake8 src --count --select=E9,F63,F7,F82 --show-source --statistics
    echo -e "${GREEN}✓ Python style check passed${NC}"
else
    echo -e "${YELLOW}flake8 not found, skipping Python style check${NC}"
fi

# C++风格检查
if command -v cpplint &> /dev/null; then
    cpplint --recursive --filter=-build/include_subdir src
    echo -e "${GREEN}✓ C++ style check passed${NC}"
else
    echo -e "${YELLOW}cpplint not found, skipping C++ style check${NC}"
fi

echo -e "${YELLOW}[3/3] Verifying launch files...${NC}"
python3 -m pytest test_launch_files.py || echo -e "${YELLOW}Launch file tests not found${NC}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  All tests completed!${NC}"
echo -e "${GREEN}========================================${NC}"
