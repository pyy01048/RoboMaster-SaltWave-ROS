#!/bin/bash
# RoboMaster哨兵机器人导航系统 - 一键编译脚本

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  RoboMaster Sentinel Navigation Build${NC}"
echo -e "${GREEN}========================================${NC}"

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}Error: ROS environment not sourced!${NC}"
    echo "Please run: source /opt/ros/humble/setup.bash"
    exit 1
fi

echo -e "${GREEN}ROS Distro: $ROS_DISTRO${NC}"

# 工作空间路径
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo -e "${GREEN}Workspace: $WORKSPACE_DIR${NC}"

# 安装依赖
echo -e "${YELLOW}[1/4] Installing dependencies...${NC}"
if command -v rosdep &> /dev/null; then
    rosdep update
    rosdep install --from-paths src --ignore-src -y --skip-keys "fastlio sentinal_point_lio"
else
    echo -e "${YELLOW}rosdep not found, skipping dependency installation${NC}"
fi

# 清理旧的编译结果
echo -e "${YELLOW}[2/4] Cleaning old build...${NC}"
rm -rf build install log

# 编译
echo -e "${YELLOW}[3/4] Building workspace...${NC}"
colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers 4

# 检查编译结果
echo -e "${YELLOW}[4/4] Verifying build...${NC}"
if [ -d "install" ]; then
    echo -e "${GREEN}✓ Build successful!${NC}"
    echo -e "${GREEN}✓ Install directory: $WORKSPACE_DIR/install${NC}"
    echo ""
    echo -e "${YELLOW}To use the workspace, run:${NC}"
    echo "  source $WORKSPACE_DIR/install/setup.bash"
    echo ""
    echo -e "${YELLOW}To launch the system, run:${NC}"
    echo "  ros2 launch sentinel_bringup bringup_all.launch.py"
else
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi
