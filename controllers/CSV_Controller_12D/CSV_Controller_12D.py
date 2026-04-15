"""
CSV_Controller_12D.py
Controller that reads a 12-column CSV file without headers.
Columns: [FL_θ, FL_β, FR_θ, FR_β, RR_θ, RR_β, RL_θ, RL_β, FL_γ, FR_γ, RR_γ, RL_γ]
"""

import Controller_TB
from LegModel import LegModel
from controller import Robot
import math
import csv
import sys
import os

# 因為共用函式庫已在同個資料夾下，直接 import 即可

DEBUG = True


class FullLegController:
    def __init__(self, prefix_sagittal="A_Module_", prefix_abad="A", axis_dir=1, robot=None):
        if robot is None:
            self.robot = Robot()
        else:
            self.robot = robot

        self.timestep = int(self.robot.getBasicTimeStep())
        self.prefix_sagittal = prefix_sagittal
        self.prefix_abad = prefix_abad
        self.axis_dir = axis_dir

        # TB Controller for Theta/Beta
        self.controller_tb = Controller_TB.Controller_TB(
            theta_0=math.radians(17))
        self.leg_model = LegModel()
        self.leg_model.forward(math.radians(17), 0.0)

        self.motors = {}

        # Sagittal motors
        for name in ["L_Motor", "R_Motor"]:
            motor = self.robot.getDevice(self.prefix_sagittal + name)
            if motor:
                self.motors[name] = motor
            else:
                if DEBUG:
                    print(f"Warning: 找不到馬達 {self.prefix_sagittal + name}")

        # G_Joint
        try:
            g_joint = self.robot.getDevice(self.prefix_sagittal + "G_Joint")
            if g_joint:
                self.motors["G_Joint"] = g_joint
                self.G_Offset = self.leg_model['ang_OGF']
            else:
                self.motors["G_Joint"] = None
        except:
            self.motors["G_Joint"] = None

        # ABAD motor
        self.motor_abad = self.robot.getDevice(f"{prefix_abad}_ABAD")
        if not self.motor_abad and DEBUG:
            print(f"Warning: 找不到馬達 {prefix_abad}_ABAD")

    def set_sagittal(self, theta, beta):
        if theta < math.radians(17):
            theta = math.radians(17)

        cmd_L, cmd_R = self.controller_tb.IK(theta, beta)
        if "L_Motor" in self.motors and "R_Motor" in self.motors:
            self.motors["L_Motor"].setPosition(cmd_L)
            self.motors["R_Motor"].setPosition(cmd_R)

        if self.motors.get("G_Joint"):
            self.leg_model.theta = theta
            self.leg_model.beta = beta
            self.leg_model.calculate()
            g_joint_target = self.leg_model['ang_OGF'] - self.G_Offset
            self.motors["G_Joint"].setPosition(g_joint_target)

    def set_abad(self, gamma):
        if self.motor_abad:
            # 加入 axis_dir 的方向調整
            self.motor_abad.setPosition(gamma * self.axis_dir)


if __name__ == "__main__":
    Bot = Robot()

    # 建立四腿全控制器
    # FL (A), FR (B), RL (C), RR (D)
    # 根據之前的物理軸設定：A: 1, B: -1, C: 1, D: -1
    legs = {
        'FL': FullLegController("A_Module_", "A", -1, Bot),
        'FR': FullLegController("B_Module_", "B", 1, Bot),
        'RR': FullLegController("C_Module_", "C", -1, Bot),
        'RL': FullLegController("D_Module_", "D", 1, Bot)
    }

    # 讀取 CSV 檔案
    # 預設從 csv 資料夾內讀取軌跡檔，使用者可在此更改檔案名稱
    csv_dir = os.path.join(os.path.dirname(__file__), "csv")
    csv_file_name = "Trot_Vx0.00_Vy0.21_Wz1.43_H0.25_S0.008_P0.2.csv"
    # csv_file_name = "Walk_Vx0.00_Vy0.00_Wz0.35_H0.25_S0.009_P1.0.csv"
    csv_file_path = os.path.join(csv_dir, csv_file_name)
    trajectory_data = []

    print(f"--- 讀取 12D 軌跡檔: {csv_file_path} ---")
    try:
        with open(csv_file_path, 'r') as file:
            reader = csv.reader(file)
            for row in reader:
                try:
                    data_row = [float(val) for val in row]
                    # 必須確保是至少 12 個欄位的資料
                    if len(data_row) >= 12:
                        trajectory_data.append(data_row)
                except ValueError:
                    continue  # 跳過無法轉換為數值的行
        print(f"--- 讀取完成，共 {len(trajectory_data)} 筆資料 ---")
    except FileNotFoundError:
        print(f"Error: 找不到 CSV 檔案 {csv_file_path}，請確認路徑。")
        trajectory_data = []

    timestep = int(Bot.getBasicTimeStep())
    current_index = 0
    max_index = len(trajectory_data)

    print("--- 12D 控制迴圈啟動 (CSV Mode) ---")

    while Bot.step(timestep) != -1:
        if current_index < max_index:
            # Columns: [FL_θ, FL_β, FR_θ, FR_β, RR_θ, RR_β, RL_θ, RL_β, FL_γ, FR_γ, RR_γ, RL_γ]
            row_data = trajectory_data[current_index]

            # FL (A): Index 0, 1, 8
            legs['FL'].set_sagittal(theta=row_data[0], beta=-row_data[1])
            legs['FL'].set_abad(gamma=row_data[8])

            # FR (B): Index 2, 3, 9
            legs['FR'].set_sagittal(theta=row_data[2], beta=row_data[3])
            legs['FR'].set_abad(gamma=row_data[9])

            # RR (C): Index 4, 5, 10
            legs['RR'].set_sagittal(theta=row_data[4], beta=row_data[5])
            legs['RR'].set_abad(gamma=row_data[10])

            # RL (D): Index 6, 7, 11
            legs['RL'].set_sagittal(theta=row_data[6], beta=-row_data[7])
            legs['RL'].set_abad(gamma=row_data[11])

            current_index += 1

        # Debug 資訊: 每 500ms 打印一次當前執行的 FL 第一欄位設定
        time = Bot.getTime()
        if int(time * 1000) % 500 == 0 and current_index < max_index and DEBUG:
            target = trajectory_data[current_index-1]
            print(f"Time: {time:.2f} | Index: {current_index}/{max_index} | "
                  f"FL_θ: {math.degrees(target[0]):.1f}, FL_β: {math.degrees(target[1]):.1f}, FL_γ: {math.degrees(target[8]):.1f}")
