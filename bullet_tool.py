import numpy as np
from scipy.spatial.transform import Rotation as R
import pybullet as p

'''
有些在pybullet中用处比较大的自写函数，作为辅佐工具
陆续完善
'''
class DebugAxes(object):
    """
    可视化基于base的坐标系, 红色x轴, 绿色y轴, 蓝色z轴
    常用于检查当前关节pose或者测量关键点的距离
    用法:
    goalPosition1 = DebugAxes()
    goalPosition1.update([0,0.19,0.15
                         ],[0,0,0,1])
    """
    def __init__(self):
        self.uids = [-1, -1, -1]

    def update(self, pos, orn):
        """
        Arguments:
        - pos: len=3, position in world frame
        - orn: len=4, quaternion (x, y, z, w), world frame
        """
        pos = np.asarray(pos).reshape(3)

        rot3x3 = R.from_quat(orn).as_matrix()
        axis_x, axis_y, axis_z = rot3x3.T
        self.uids[0] = p.addUserDebugLine(pos, pos + axis_x * 0.05, [1, 0, 0], replaceItemUniqueId=self.uids[0])
        self.uids[1] = p.addUserDebugLine(pos, pos + axis_y * 0.05, [0, 1, 0], replaceItemUniqueId=self.uids[1])
        self.uids[2] = p.addUserDebugLine(pos, pos + axis_z * 0.05, [0, 0, 1], replaceItemUniqueId=self.uids[2])
