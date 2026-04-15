# motor_config.py
# 定義各腿的關節與馬達方向 (1.0 為正轉, -1.0 為反轉)
# 
# joint_dir: 
#   - theta: 腿部伸展角方向
#   - beta: 腿部旋轉角方向
#   - g_joint_beta: G_Joint 被動追蹤的旋轉角方向
# motor_dir:
#   - L: 左馬達方向
#   - R: 右馬達方向
#   - ABAD: 側擺馬達方向

LEG_CONFIG = {
    'A': {
        'joint_dir': {
            'theta': 1.0,
            'beta': -1.0,
            'g_joint_beta': -1.0,
        },
        'motor_dir': {
            'L': 1.0,
            'R': 1.0,
            'ABAD': -1.0,
        }
    },
    'B': {
        'joint_dir': {
            'theta': 1.0,
            'beta': 1.0,
            'g_joint_beta': -1.0,
        },
        'motor_dir': {
            'L': 1.0,
            'R': 1.0,
            'ABAD': 1.0,
        }
    },
    'C': {
        'joint_dir': {
            'theta': 1.0,
            'beta': 1.0,
            'g_joint_beta': -1.0,
        },
        'motor_dir': {
            'L': 1.0,
            'R': 1.0,
            'ABAD': -1.0,
        }
    },
    'D': {
        'joint_dir': {
            'theta': 1.0,
            'beta': -1.0,
            'g_joint_beta': -1.0,
        },
        'motor_dir': {
            'L': 1.0,
            'R': 1.0,
            'ABAD': 1.0,
        }
    }
}
