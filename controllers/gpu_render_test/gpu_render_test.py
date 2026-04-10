"""
gpu_render_test.py  —  Webots GPU Rendering Speed Test Controller
=================================================================
純 Webots Python controller（無 ROS2），直接在 Webots 進程內運行。
使用 input_* CSV 軌跡驅動機器人，每 5 秒回報一次模擬速度。

使用位置控制（position control）而非扭矩控制，
可避免 webots_ros2_driver torque API 在 R2025a 的 segfault 問題。

輸出格式（可被 test_gpu_render.sh 解析）:
  GPU_TEST: speed=0.2345x  t_sim=7.23s  t_wall=30.8s
  GPU_TEST: FINAL speed=0.2312x
"""

from controller import Robot
import csv
import os
import sys
import time
import math

# ── 設定 ──────────────────────────────────────────────────────────────────────

# 搜尋 input_*.csv（優先 Vx0.20 版本）
_CSV_DIR = "/root/corgi_ws/corgi_ros2_ws/input_csv"
_candidates = sorted(
    [f for f in os.listdir(_CSV_DIR) if f.startswith("input_") and f.endswith(".csv")]
) if os.path.isdir(_CSV_DIR) else []

# 優先選速度較快的 Vx0.20
CSV_PATH = None
for _f in _candidates:
    if "Vx0.20" in _f:
        CSV_PATH = os.path.join(_CSV_DIR, _f)
        break
if CSV_PATH is None and _candidates:
    CSV_PATH = os.path.join(_CSV_DIR, _candidates[0])

TEST_DURATION_WALL = 35  # 壁時間測試長度（秒）
REPORT_INTERVAL    = 5   # 每幾秒回報一次速度

# ── 腿部 IK 簡化版（與 Controller_TB.py 兼容的近似）──────────────────────────

THETA_MIN = math.radians(17)   # 最小 theta（0.2967 rad）

def simple_ik(theta, beta):
    """
    簡化 IK（不引入 Controller_TB 避免依賴問題）
    回傳 (cmd_L, cmd_R) 近似值，足以驅動位置控制做渲染測試。
    """
    if theta < THETA_MIN:
        theta = THETA_MIN
    # 模仿 Controller_TB 的近似關係
    cmd_L = theta + beta
    cmd_R = theta - beta
    return cmd_L, cmd_R


# ── 主程式 ────────────────────────────────────────────────────────────────────

def main():
    robot    = Robot()
    timestep = int(robot.getBasicTimeStep())   # 通常 1 ms

    print(f"GPU_TEST: Controller started (timestep={timestep}ms)", flush=True)
    print(f"GPU_TEST: CSV source: {CSV_PATH or 'none (static hold)'}", flush=True)

    # ── 初始化馬達（位置控制模式）──────────────────────────────────────────────
    LEGS = {
        'A': ("A_Module_", "A"),
        'B': ("B_Module_", "B"),
        'C': ("C_Module_", "C"),
        'D': ("D_Module_", "D"),
    }
    motors = {}
    for leg, (prefix, abad_prefix) in LEGS.items():
        for side in ("L_Motor", "R_Motor"):
            dev = robot.getDevice(prefix + side)
            if dev:
                motors[f"{leg}_{side}"] = dev
                dev.setPosition(THETA_MIN)           # 安全初始位置
        abad = robot.getDevice(f"{abad_prefix}_ABAD")
        if abad:
            motors[f"{leg}_ABAD"] = abad
            abad.setPosition(0.0)

    print(f"GPU_TEST: Motors found: {len(motors)}", flush=True)

    # G_Joint 初始化（被動追蹤，設為 0）
    for leg, (prefix, _) in LEGS.items():
        g = robot.getDevice(prefix + "G_Joint")
        if g:
            g.setPosition(0.0)

    # ── 讀取 CSV ───────────────────────────────────────────────────────────────
    rows = []
    if CSV_PATH and os.path.exists(CSV_PATH):
        with open(CSV_PATH, 'r') as f:
            for line in f:
                try:
                    row = [float(x) for x in line.strip().split(',')]
                    if len(row) >= 12:
                        rows.append(row)
                except (ValueError, AttributeError):
                    continue
        print(f"GPU_TEST: Loaded {len(rows)} CSV rows", flush=True)
    else:
        print("GPU_TEST: No CSV, holding static position", flush=True)

    csv_len = len(rows)

    # CSV 欄位映射（與 corgi_csv_control.cpp 一致）:
    # cols 0..7  → [A_θ, A_β, B_θ, B_β, C_θ, C_β, D_θ, D_β]
    # cols 8..11 → [A_γ, B_γ, C_γ, D_γ]
    LEG_CSV_MAP = [
        # (leg, θ_col, β_col, γ_col, beta_sign)
        ('A', 0, 1,  8,  -1),
        ('B', 2, 3,  9,   1),
        ('C', 4, 5, 10,  -1),
        ('D', 6, 7, 11,   1),
    ]

    # ── 主迴圈 ─────────────────────────────────────────────────────────────────
    start_wall = time.time()
    last_report = start_wall
    csv_idx    = 0

    # Webots batch 模式下 controller 的 print() 不轉送到父進程 stdout，
    # 改用檔案方式回報結果（test_gpu_render_inner.py 讀取此檔）
    result_file = "/tmp/gpu_render_test_result.txt"
    with open(result_file, 'w') as rf:
        rf.write("GPU_TEST: Controller started\n")

    while robot.step(timestep) != -1:
        sim_time  = robot.getTime()
        wall_time = time.time()
        elapsed   = wall_time - start_wall

        # 套用 CSV 指令（位置控制）
        if csv_len > 0:
            row = rows[csv_idx % csv_len]
            for leg, tc, bc, gc, beta_sign in LEG_CSV_MAP:
                theta = row[tc]
                beta  = row[bc] * beta_sign
                gamma = row[gc]
                cmd_L, cmd_R = simple_ik(theta, beta)
                if f"{leg}_L_Motor" in motors:
                    motors[f"{leg}_L_Motor"].setPosition(cmd_L)
                if f"{leg}_R_Motor" in motors:
                    motors[f"{leg}_R_Motor"].setPosition(cmd_R)
                if f"{leg}_ABAD" in motors:
                    motors[f"{leg}_ABAD"].setPosition(gamma)
            csv_idx += 1

        # 每 REPORT_INTERVAL 秒輸出一次速度
        if wall_time - last_report >= REPORT_INTERVAL and elapsed > 0:
            speed = sim_time / elapsed
            msg = f"GPU_TEST: speed={speed:.4f}x  t_sim={sim_time:.2f}s  t_wall={elapsed:.1f}s"
            print(msg, flush=True)
            with open(result_file, 'a') as rf:
                rf.write(msg + "\n")
            last_report = wall_time

        # 達到測試時間後結束
        if elapsed >= TEST_DURATION_WALL:
            speed = sim_time / elapsed if elapsed > 0 else 0
            final_msg = (f"GPU_TEST: FINAL speed={speed:.4f}x  "
                         f"t_sim={sim_time:.2f}s  t_wall={elapsed:.1f}s")
            print(final_msg, flush=True)
            with open(result_file, 'a') as rf:
                rf.write(final_msg + "\n")
                rf.write("GPU_TEST: Controller exiting\n")
            break

    print("GPU_TEST: Controller exiting", flush=True)
    sys.exit(0)


if __name__ == '__main__':
    main()
