#!/usr/bin/env bash
# ── OpenArm Unified Launch Script ──
# Auto-detects connected CAN interfaces without renaming them:
#   1 CAN 接口  → 底盘
#   2 CAN 接口  → 双臂
#   3 CAN 接口  → 双臂 + 底盘
#
# Actual interface names are injected into a runtime arm config and the
# chassis --can-if argument. This avoids interface-name collisions.
# The defaults can be overridden with RIGHT_ARM_CAN, LEFT_ARM_CAN,
# and CHASSIS_CAN when kernel enumeration is not stable.
#
# Usage:  ./config/launch.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
UNIFIED_YAML="$SCRIPT_DIR/dataflow-unified.yaml"
ARM_CONFIG_SOURCE="$PROJECT_DIR/src/local_openarm_driver/config.yaml"
RUNTIME_YAML="/tmp/dataflow-runtime-${UID}.yaml"
RUNTIME_ARM_CONFIG="/tmp/openarm-config-${UID}.yaml"
if [ -n "${XDG_RUNTIME_DIR:-}" ] && [ -d "$XDG_RUNTIME_DIR" ]; then
    LOCK_FILE="$XDG_RUNTIME_DIR/openarm-launch.lock"
else
    LOCK_FILE="/tmp/openarm-launch-v2-${UID}.lock"
fi
DORA_PID=""
CLEANUP_DONE=false

# ── 单实例锁与清理 ──

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "[launch] 错误: 已有 launch.sh 正在运行，请先停止旧实例"
    exit 1
fi

cleanup() {
    local exit_code=$?
    local attempts

    # EXIT 与信号可能连续触发，确保清理只执行一次。
    if $CLEANUP_DONE; then
        return
    fi
    CLEANUP_DONE=true
    trap - EXIT INT TERM

    if [ -n "$DORA_PID" ]; then
        echo "[launch] 正在停止 Dora 节点 ..."

        # 先通知 dora run 优雅停止，让机械臂等节点处理 STOP 事件。
        kill -INT "$DORA_PID" 2>/dev/null || true
        for attempts in {1..50}; do
            if ! pgrep -s "$DORA_PID" >/dev/null 2>&1; then
                break
            fi
            sleep 0.1
        done

        # Dora 节点可能拥有各自的进程组，但都属于 setsid 创建的 session。
        if pgrep -s "$DORA_PID" >/dev/null 2>&1; then
            echo "[launch] 部分节点未退出，发送 TERM ..."
            pkill -TERM -s "$DORA_PID" 2>/dev/null || true
            for attempts in {1..20}; do
                if ! pgrep -s "$DORA_PID" >/dev/null 2>&1; then
                    break
                fi
                sleep 0.1
            done
        fi

        if pgrep -s "$DORA_PID" >/dev/null 2>&1; then
            echo "[launch] 强制清理残留节点 ..."
            pkill -KILL -s "$DORA_PID" 2>/dev/null || true
        fi

        wait "$DORA_PID" 2>/dev/null || true
        DORA_PID=""
    fi

    rm -f "$RUNTIME_YAML" "$RUNTIME_ARM_CONFIG"
    flock -u 9 2>/dev/null || true
    exec 9>&-
    echo "[launch] 已退出"

    return "$exit_code"
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# ── Phase 1: 收集 CAN 接口 ──

echo "[launch] 扫描 CAN 接口 ..."

mapfile -t CAN_IFACES < <(
    ip -o link show type can 2>/dev/null |
        awk '{sub(/:$/, "", $2); print $2}' |
        sort -V
)

CAN_COUNT=${#CAN_IFACES[@]}
HAS_ARMS=false
HAS_CHASSIS=false
RIGHT_CAN=""
LEFT_CAN=""
CHASSIS_CAN_ACTUAL=""

case $CAN_COUNT in
    0)
        echo "[launch] 未检测到任何 CAN 接口"
        ;;
    1)
        CHASSIS_CAN_ACTUAL="${CHASSIS_CAN:-${CAN_IFACES[0]}}"
        HAS_CHASSIS=true
        echo "[launch] 检测到 1 个 CAN 接口 → 底盘 ($CHASSIS_CAN_ACTUAL)"
        ;;
    2)
        RIGHT_CAN="${RIGHT_ARM_CAN:-${CAN_IFACES[0]}}"
        LEFT_CAN="${LEFT_ARM_CAN:-${CAN_IFACES[1]}}"
        HAS_ARMS=true
        echo "[launch] 检测到 2 个 CAN 接口 → 双臂 ($RIGHT_CAN / $LEFT_CAN)"
        ;;
    3)
        RIGHT_CAN="${RIGHT_ARM_CAN:-${CAN_IFACES[0]}}"
        LEFT_CAN="${LEFT_ARM_CAN:-${CAN_IFACES[1]}}"
        CHASSIS_CAN_ACTUAL="${CHASSIS_CAN:-${CAN_IFACES[2]}}"
        HAS_ARMS=true
        HAS_CHASSIS=true
        echo "[launch] 检测到 3 个 CAN 接口 → 双臂 ($RIGHT_CAN / $LEFT_CAN) + 底盘 ($CHASSIS_CAN_ACTUAL)"
        ;;
    *)
        echo "[launch] 错误: 检测到 $CAN_COUNT 个 CAN 接口，无法自动分配角色"
        echo "[launch] 接口: ${CAN_IFACES[*]}"
        exit 1
        ;;
esac

echo ""

# ── Phase 2: 校验并配置 CAN 接口 ──

validate_can() {
    local iface="$1"
    local role="$2"

    if ! ip link show "$iface" &>/dev/null; then
        echo "[launch] 错误: $role 指定的接口 $iface 不存在"
        exit 1
    fi
}

if $HAS_ARMS; then
    validate_can "$RIGHT_CAN" "右臂"
    validate_can "$LEFT_CAN" "左臂"
    if [ "$RIGHT_CAN" = "$LEFT_CAN" ]; then
        echo "[launch] 错误: 左右臂不能共用接口 $RIGHT_CAN"
        exit 1
    fi
fi

if $HAS_CHASSIS; then
    validate_can "$CHASSIS_CAN_ACTUAL" "底盘"
    if $HAS_ARMS && { [ "$CHASSIS_CAN_ACTUAL" = "$RIGHT_CAN" ] || [ "$CHASSIS_CAN_ACTUAL" = "$LEFT_CAN" ]; }; then
        echo "[launch] 错误: 底盘不能与机械臂共用接口 $CHASSIS_CAN_ACTUAL"
        exit 1
    fi
fi

setup_can() {
    local iface="$1"
    local ctype="$2"

    echo "[launch] $iface 配置中 ($ctype) ..."
    if ! sudo ip link set "$iface" down; then
        echo "[launch] 错误: 无法关闭 $iface，可能仍被旧进程占用"
        return 1
    fi

    if [ "$ctype" = "arm" ]; then
        if sudo ip link set "$iface" type can bitrate 1000000 dbitrate 5000000 fd on 2>/dev/null; then
            echo "[launch]   $iface 设为 CAN FD (1M/5M)"
        else
            echo "[launch]   $iface CAN FD 不支持，降级为普通 CAN (1M)"
            sudo ip link set "$iface" type can bitrate 1000000 fd off
        fi
    else
        sudo ip link set "$iface" type can bitrate 500000 fd off
    fi

    sudo ip link set "$iface" up
    if ! ip -o link show "$iface" | grep -q '<[^>]*UP'; then
        echo "[launch] 错误: $iface 未进入 UP 状态"
        return 1
    fi
}

if $HAS_ARMS; then
    setup_can "$RIGHT_CAN" arm
    echo "[launch] 右臂 $RIGHT_CAN 就绪"
    setup_can "$LEFT_CAN" arm
    echo "[launch] 左臂 $LEFT_CAN 就绪"
fi
if $HAS_CHASSIS; then
    setup_can "$CHASSIS_CAN_ACTUAL" chassis
    echo "[launch] 底盘 $CHASSIS_CAN_ACTUAL 就绪"
fi

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
if $HAS_ARMS; then
    echo "  右臂 CAN: $RIGHT_CAN"
    echo "  左臂 CAN: $LEFT_CAN"
fi
if $HAS_CHASSIS; then
    echo "  底盘 CAN: $CHASSIS_CAN_ACTUAL"
fi
echo ""

# ── 生成运行时 YAML 和机械臂配置 ──

python3 - "$UNIFIED_YAML" "$RUNTIME_YAML" \
    "$ARM_CONFIG_SOURCE" "$RUNTIME_ARM_CONFIG" \
    "$HAS_ARMS" "$HAS_CHASSIS" "$HAS_LIFT" \
    "$RIGHT_CAN" "$LEFT_CAN" "$CHASSIS_CAN_ACTUAL" <<'PYEOF'
import shlex
import sys

import yaml

unified_path = sys.argv[1]
runtime_path = sys.argv[2]
arm_config_source = sys.argv[3]
arm_config_path = sys.argv[4]
has_arms = sys.argv[5] == "true"
has_chassis = sys.argv[6] == "true"
has_lift = sys.argv[7] == "true"
right_can = sys.argv[8]
left_can = sys.argv[9]
chassis_can = sys.argv[10]

with open(unified_path, "r") as f:
    doc = yaml.safe_load(f)

if has_arms:
    with open(arm_config_source, "r") as f:
        arm_config = yaml.safe_load(f)
    arm_config["can_interface"]["right_arm"] = right_can
    arm_config["can_interface"]["left_arm"] = left_can
    with open(arm_config_path, "w") as f:
        yaml.safe_dump(arm_config, f, default_flow_style=False, allow_unicode=True)

filtered_nodes = []
for node in doc.get("nodes", []):
    nid = node.get("id", "")

    if nid.startswith("arm-"):
        if not has_arms:
            continue
        if nid.startswith("arm-follower-"):
            args = shlex.split(node.get("args", ""))
            args.extend(["--config", arm_config_path])
            node["args"] = shlex.join(args)
        filtered_nodes.append(node)
    elif nid.startswith("chassis-"):
        if not has_chassis:
            continue
        args = shlex.split(node.get("args", ""))
        if "--can-if" in args:
            args[args.index("--can-if") + 1] = chassis_can
        else:
            args.extend(["--can-if", chassis_can])
        node["args"] = shlex.join(args)
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

# ── Launch dora ──

cd "$PROJECT_DIR"
if command -v dora &>/dev/null; then
    DORA_CMD="$(command -v dora)"
elif [ -x "$PROJECT_DIR/.venv/bin/dora" ]; then
    DORA_CMD="$PROJECT_DIR/.venv/bin/dora"
else
    echo "[launch] 错误: 找不到 dora 命令"
    exit 1
fi

# 不让 Dora 及其节点继承锁 FD；锁只由 launch.sh 本身持有。
setsid "$DORA_CMD" run "$RUNTIME_YAML" 9>&- &
DORA_PID=$!
wait "$DORA_PID"
