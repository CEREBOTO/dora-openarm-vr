#!/usr/bin/env bash
# ── OpenArm Unified Launch Script ──
# Auto-detects connected CAN interfaces by *count* and renames them:
#   1 CAN 接口  → can2 (底盘)
#   2 CAN 接口  → can0, can1 (双臂)
#   3 CAN 接口  → can0, can1, can2 (双臂 + 底盘)
#
# Arm followers hardcode can0/right_arm and can1/left_arm (config.yaml),
# so renaming is REQUIRED for arms.  Chassis uses --can-if which we inject.
#
# Usage:  ./config/launch.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
UNIFIED_YAML="$SCRIPT_DIR/dataflow-unified.yaml"
RUNTIME_YAML="/tmp/dataflow-runtime.yaml"

# ── Phase 0: 检查并安全清理幽灵接口 ──
# 上次运行我们把硬件重命名为 can0/can1/can2。
# 设备拔掉后 CAN 接口残留为 DOWN 的"幽灵"，/sys/class/net/$iface/device 不存在。
# 安全规则：只删除没有硬件背书的接口。

for iface in can0 can1 can2; do
    if ip link show "$iface" &>/dev/null; then
        if [ ! -e /sys/class/net/"$iface"/device ]; then
            echo "[launch] 清理幽灵接口: $iface（无硬件设备）"
            sudo ip link delete "$iface" 2>/dev/null || true
        fi
    fi
done

# ── Phase 1: 收集所有 CAN 接口并重命名为临时名字 ──

echo "[launch] 扫描 CAN 接口 ..."

CAN_IFACES=($(ip -o link show type can 2>/dev/null | awk '{print $2}' | sed 's/:$//' || true))

if [ ${#CAN_IFACES[@]} -eq 0 ]; then
    echo "[launch] 未检测到任何 CAN 接口"
fi

# 先把所有接口 down 并重命名为临时名字
TMP_CAN=()
for idx in "${!CAN_IFACES[@]}"; do
    iface="${CAN_IFACES[$idx]}"
    sudo ip link set "$iface" down 2>/dev/null || true
    tmp_name="tmp_can_$idx"
    sudo ip link set "$iface" name "$tmp_name" 2>/dev/null || true
    TMP_CAN+=("$tmp_name")
done

# ── Phase 2: 按数量决定角色并重命名为目标名称 ──

CAN_COUNT=${#TMP_CAN[@]}
HAS_ARMS=false
HAS_CHASSIS=false
HAS_CAN0=false
HAS_CAN1=false
HAS_CAN2=false

case $CAN_COUNT in
    1)
        echo "[launch]   ${TMP_CAN[0]} → can2"
        sudo ip link set "${TMP_CAN[0]}" name can2 2>/dev/null || true
        HAS_CHASSIS=true
        HAS_CAN2=true
        echo "[launch] 检测到 1 个 CAN 接口 → 底盘 (can2)"
        ;;
    2)
        echo "[launch]   ${TMP_CAN[0]} → can0"
        sudo ip link set "${TMP_CAN[0]}" name can0 2>/dev/null || true
        echo "[launch]   ${TMP_CAN[1]} → can1"
        sudo ip link set "${TMP_CAN[1]}" name can1 2>/dev/null || true
        HAS_ARMS=true
        HAS_CAN0=true
        HAS_CAN1=true
        echo "[launch] 检测到 2 个 CAN 接口 → 双臂 (can0 / can1)"
        ;;
    3)
        echo "[launch]   ${TMP_CAN[0]} → can0"
        sudo ip link set "${TMP_CAN[0]}" name can0 2>/dev/null || true
        echo "[launch]   ${TMP_CAN[1]} → can1"
        sudo ip link set "${TMP_CAN[1]}" name can1 2>/dev/null || true
        echo "[launch]   ${TMP_CAN[2]} → can2"
        sudo ip link set "${TMP_CAN[2]}" name can2 2>/dev/null || true
        HAS_ARMS=true
        HAS_CHASSIS=true
        HAS_CAN0=true
        HAS_CAN1=true
        HAS_CAN2=true
        echo "[launch] 检测到 3 个 CAN 接口 → 双臂 (can0 / can1) + 底盘 (can2)"
        ;;
esac

echo ""

# ── Phase 3: 配置 CAN 接口并 UP ──

setup_can() {
    local iface="$1"
    local ctype="$2"

    echo "[launch] $iface 配置中 ($ctype) ..."
    sudo ip link set "$iface" down 2>/dev/null || true

    if [ "$ctype" = "arm" ]; then
        if sudo ip link set "$iface" type can bitrate 1000000 dbitrate 5000000 fd on 2>/dev/null; then
            echo "[launch]   $iface 设为 CAN FD (1M/5M)"
        else
            echo "[launch]   $iface CAN FD 不支持，降级为普通 CAN (1M)"
            sudo ip link set "$iface" type can bitrate 1000000 2>/dev/null || true
        fi
    else
        sudo ip link set "$iface" type can bitrate 500000 2>/dev/null || true
    fi

    sudo ip link set "$iface" up || true
}

if $HAS_CAN0; then setup_can can0 arm;   echo "[launch] can0 就绪"; fi
if $HAS_CAN1; then setup_can can1 arm;   echo "[launch] can1 就绪"; fi
if $HAS_CAN2; then setup_can can2 chassis; echo "[launch] can2 就绪"; fi

echo ""

# ── 升降机串口 ──

if [ -e /dev/ttyACM0 ]; then
    HAS_LIFT=true
    echo "[launch] /dev/ttyACM0 存在 (升降机)"
else
    HAS_LIFT=false
    echo "[launch] /dev/ttyACM0 不存在 (升降机)"
fi

# ── 全部没连则退出 ──

if ! $HAS_ARMS && ! $HAS_CHASSIS && ! $HAS_LIFT; then
    echo "[launch] 错误: 未检测到任何硬件，退出程序"
    exit 1
fi

# ── 安装 openarm-can ──

if $HAS_ARMS; then
    echo "[launch] 安装 openarm-can ..."
    uv pip install openarm-can > /dev/null 2>&1
fi

# ── 状态摘要 ──

_human() { if "$1"; then echo "启动"; else echo "跳过"; fi; }
echo ""
echo "  双臂:   $(_human $HAS_ARMS)"
echo "  底盘:   $(_human $HAS_CHASSIS)"
echo "  升降机: $(_human $HAS_LIFT)"
echo ""

# ── 生成运行时 YAML ──

rm -f "$RUNTIME_YAML"

python3 - "$UNIFIED_YAML" "$RUNTIME_YAML" \
    "$HAS_ARMS" "$HAS_CHASSIS" "$HAS_LIFT" <<'PYEOF'
import sys, yaml

unified_path = sys.argv[1]
runtime_path = sys.argv[2]
has_arms     = sys.argv[3] == "true"
has_chassis  = sys.argv[4] == "true"
has_lift     = sys.argv[5] == "true"

with open(unified_path, "r") as f:
    doc = yaml.safe_load(f)

filtered_nodes = []
for node in doc.get("nodes", []):
    nid = node.get("id", "")

    if nid.startswith("arm-"):
        if not has_arms:
            continue
        # arm-follower-* 使用 config.yaml 中的 can0/can1，已被 launch.sh 配置好
        filtered_nodes.append(node)
    elif nid.startswith("chassis-"):
        if not has_chassis:
            continue
        # chassis-controller 已通过 YAML 中的 --can-if can2 使用重命名后的接口
        filtered_nodes.append(node)
    elif nid.startswith("lift-"):
        if has_lift:
            filtered_nodes.append(node)
    else:
        filtered_nodes.append(node)

doc["nodes"] = filtered_nodes

with open(runtime_path, "w") as f:
    yaml.dump(doc, f, default_flow_style=False, allow_unicode=True)

print(f"[launch] {len(filtered_nodes)} 个节点启动中 ...")
PYEOF

# ── Cleanup ──

cleanup() {
    rm -f "$RUNTIME_YAML"

    if [ -n "${DORA_PID:-}" ]; then
        kill -TERM "$DORA_PID" 2>/dev/null || true
        wait "$DORA_PID" 2>/dev/null || true
    fi
    echo "[launch] 已退出"
}
trap cleanup EXIT INT TERM

# ── Launch dora ──

cd "$PROJECT_DIR"
dora run "$RUNTIME_YAML" &
DORA_PID=$!
wait "$DORA_PID"
