"""
Controller_TB
運動學轉換器 (Kinematics Transformer)
負責將廣義座標 (Theta, Beta) 轉換為馬達命令 (Phi_L, Phi_R)，以及反向轉換
"""

import math

class Controller_TB:
    def __init__(self, theta_0=0.0):
        """
        初始化運動學轉換器
        :param theta_0: 初始安裝角度偏移量 (根據您的機構校正值，單位：弧度)
        """
        self.theta_0 = theta_0

    def IK(self, theta, beta):
        """
        [控制器用] 輸入廣義座標 (Theta, Beta)，計算馬達目標角度 (Phi_L, Phi_R)
        
        :param theta: 機構伸展角度 (夾角)
        :param beta:  機構整體旋轉角度 (相對於 y 軸)
        :return: (phi_L, phi_R) 目標馬達角度，單位：弧度
        """
        # 根據推導出的逆運算公式
        # phi_L = beta + (theta - theta_0)
        # phi_R = beta - (theta - theta_0)
        
        delta_theta = theta - self.theta_0
        
        phi_L = beta + delta_theta
        phi_R = beta - delta_theta
        
        return phi_L, phi_R

    def FK(self, phi_L, phi_R):
        """
        [感測器用] 輸入馬達角度 (Phi_L, Phi_R)，計算目前廣義座標 (Theta, Beta)
        對應圖片中的式 3.3
        
        :param phi_L: 左馬達目前角度
        :param phi_R: 右馬達目前角度
        :return: (theta, beta)
        """
        # 式 3.3: 
        # theta = 0.5 * (phi_L - phi_R) + theta_0
        # beta  = 0.5 * (phi_L + phi_R)
        
        theta = 0.5 * (phi_L - phi_R) + self.theta_0
        beta  = 0.5 * (phi_L + phi_R)
        
        return theta, beta

# =================使用範例=================
if __name__ == "__main__":
    # 假設初始偏移 theta_0 為 0 (如果您的機構一開始不是閉合的，這裡要填入初始夾角)
    kinematics = Controller_TB(theta_0=math.radians(17))

    # 情境 1: 控制器下達命令
    # 我們想要：伸張角度(theta) = 30度, 整體不旋轉(beta) = 0度
    target_theta = math.radians(30)
    target_beta = math.radians(0)

    cmd_L, cmd_R = kinematics.IK(target_theta, target_beta)
    print(f"輸入 Theta: 30°, Beta: 0° -> 馬達命令: L={math.degrees(cmd_L):.2f}°, R={math.degrees(cmd_R):.2f}°")
    # 預期結果: L=30, R=-30 (兩腳張開)

    # 情境 2: 控制器下達命令
    # 我們想要：保持張開(theta) = 0度, 但整體往前擺(beta) = 20度
    target_theta = math.radians(17)
    target_beta = math.radians(20)
    
    cmd_L, cmd_R = kinematics.IK(target_theta, target_beta)
    print(f"輸入 Theta: 17°, Beta: 20° -> 馬達命令: L={math.degrees(cmd_L):.2f}°, R={math.degrees(cmd_R):.2f}°")
    print(f" 反解回 Theta, Beta: {math.degrees(kinematics.FK(cmd_L, cmd_R)[0]):.2f}°, {math.degrees(kinematics.FK(cmd_L, cmd_R)[1]):.2f}°")
    # 預期結果: L=20, R=20 (兩腳同步轉動)