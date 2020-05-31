"""
Parts of the code in this file are from https://github.com/yrouben/Sampling-Based-Path-Planning-Library. Thanks for the coder yrouben.
"""

from __future__ import division
from shapely.geometry import Polygon
from RRTGit.environment import Environment        # 从environment文件中导入Environment
from RRTGit.RRTs import RRTPlanner                # 从文件RRTs文件中导入RRTPlanner
from matplotlib import pyplot as plt


environment = Environment('bugtrap.yaml')     # 导入数据
bounds = (-2, -3, 12, 8)                      # 图的边界
start_pose = (2, 2.5)                         #起始点
goal_region = Polygon([(10,5), (10,6), (11,6), (11,5)])   # 目标范围
object_radius = 0.3             # 对象半径,比如对象是个小车，所以要留半径不可能像线那样理想
steer_distance = 0.3            # 数枝长度
num_iterations = 10000          # 迭代次数
resolution = 3
drawResults = True
runForFullIterations = False




sbpp = RRTPlanner()           # 继承RRTPlann 类
path= sbpp.RRT(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, drawResults, runForFullIterations) # 产生路径
plt.show()                   # 显示路径图

