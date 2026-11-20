"""《旋涡 / VORTEX》参数（mm）—— 空心漏斗 + 双螺旋护栏，13 圈，珠子重力下滑。"""
import numpy as np

R_TOP = 20.0     # 漏斗顶部半径
R_BOT = 50.0     # 漏斗底部半径
H = 70.0         # 漏斗高
WALL = 2.5       # 壁厚

TURNS = 11       # 螺旋圈数
RAIL_GAP = 10.0  # 两条护栏中心距（内净宽 8）
RAIL_T = 1.5     # 护栏厚
RAIL_H = 3.5     # 护栏高

BEAD_D = 10.0
BEAD_R = 5.0

PLATE_R = 65.0
PLATE_H = 8.0

# 槽中心线半径 = 漏斗内表面半径 - RAIL_GAP/2
def r_cone(v):
    return R_TOP + (R_BOT - R_TOP)*v

def theta_s(v):
    return 2*np.pi*TURNS*v
