#!/bin/bash
# =========================================================================
# 一键启动巡检: 拉起节点 → 等待就绪 → 触发巡航
# 用法:
#   ./start_cruise.sh              # 默认工位1
#   ./start_cruise.sh 3            # 从工位3开始
#   ./start_cruise.sh 1 --build    # 先编译再启动
# =========================================================================

# 退出时自动清理所有子进程
cleanup() {
    echo ""
    echo "=== 正在清理残留进程 ==="
    pkill -9 -f "launch_ros" 2>/dev/null
    pkill -9 -f "originbot_base" 2>/dev/null
    pkill -9 -f "navigation_node" 2>/dev/null
    pkill -9 -f "vision_node" 2>/dev/null
    pkill -9 -f "vp100_ros2" 2>/dev/null
    pkill -9 -f "nav2" 2>/dev/null
    sleep 1
    echo "=== 清理完成 ==="
}
trap cleanup EXIT INT TERM

set -e

STATION_ID=${1:-1}
DO_BUILD=false
[[ "$2" == "--build" || "$1" == "--build" ]] && DO_BUILD=true

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=========================================="
echo "  一键巡检启动"
echo "  工位: ${STATION_ID}"
echo "=========================================="

# ---- 编译（可选）----
if $DO_BUILD; then
    echo ""
    echo "=== 编译 lab_inspection ==="
    cd "$WS_DIR"
    colcon build --packages-select lab_inspection
fi

# ---- 后台拉起所有节点 ----
echo ""
echo "=== 拉起巡检节点 ==="
ros2 launch lab_inspection lab_inspection.launch.py &
LAUNCH_PID=$!

# ---- 等待服务就绪（轮询，不硬等）----
echo ""
echo "=== 等待 /start_inspection 服务就绪 ==="
MAX_WAIT=120
for ((i=1; i<=MAX_WAIT; i++)); do
    if ros2 service list 2>/dev/null | grep -q "/start_inspection"; then
        echo "✓ /start_inspection 已就绪 (${i}s)"
        break
    fi
    if [ $i -eq $MAX_WAIT ]; then
        echo "✗ 超时 ${MAX_WAIT}s，退出"
        kill $LAUNCH_PID 2>/dev/null
        exit 1
    fi
    sleep 1
done

# ---- 等待 Nav2 生命周期节点完全激活 ----
echo ""
echo "=== 等待导航生命周期节点激活 ==="
NAV2_MAX_WAIT=180
for ((i=1; i<=NAV2_MAX_WAIT; i++)); do
    # 检查 bt_navigator 是否处于 active 状态（生命周期节点激活的最后一步）
    STATE=$(ros2 lifecycle get /bt_navigator 2>/dev/null || echo "")
    if echo "$STATE" | grep -qi "active"; then
        echo "✓ 导航生命周期节点已全部激活 (${i}s)"
        break
    fi
    if [ $i -eq $NAV2_MAX_WAIT ]; then
        echo "✗ 等待导航节点激活超时 ${NAV2_MAX_WAIT}s，退出"
        kill $LAUNCH_PID 2>/dev/null
        exit 1
    fi
    # 每 10 秒打印一次状态，避免刷屏
    if [ $((i % 10)) -eq 0 ]; then
        echo "  等待中... (${i}s) 当前状态: ${STATE}"
    fi
    sleep 1
done

# ---- 触发巡检 ----
echo ""
echo "=== 触发巡检 (工位 ${STATION_ID}) ==="
ros2 service call /start_inspection lab_inspection/srv/TriggerInspection \
    "{station_id: ${STATION_ID}}"

echo ""
echo "=========================================="
echo "  巡检运行中 (PID: ${LAUNCH_PID})"
echo "  Ctrl+C 停止"
echo "=========================================="

wait $LAUNCH_PID
