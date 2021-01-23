"""
Parts of the code in this file are from https://github.com/yrouben/Sampling-Based-Path-Planning-Library. Thanks for the coder yrouben.
"""

from __future__ import division

import csv

from shapely.geometry import Polygon
from RRTPCN.Genvironment import Environment        # 从environment文件中导入Environment
from RRTPCN.GRRTs import RRTPlanner                # 从文件RRTs文件中导入RRTPlanner
from matplotlib import pyplot as plt
import math


environment = Environment('bugtrap4.yaml')
bounds = (0, 0, 10,10)
start_pose = (0.2, 0.2)
goal_region = Polygon([(9.5,9.5), (9.5,10.0), (10.0,10.0), (10.0,9.5)])


object_radius = 0.2                                       # 对象半径,比如对象是个小车，所以要留半径不可能像线那样理想
steer_distance = 0.3                                     # 步长
num_iterations = 100000                                     # 迭代次数
resolution = 3
rand_fre = 0.5                                       # 目标偏向概率阈值rand_fre是偏向目标的概率
num_randpoint = 8                                        # 备选节点池中的点
angel_min = math.pi*1/2                                   # 设置夹角的最小值，现在的问题是如果小于这个值而贸然的去增加节点并不一定能缓和角度。所以这个增加点的坐标要自己去选定
angel_min = 0
node_k = 0.5                                                   #节点过滤中的K值用于衡量
pcn = float('inf')                                               # 节点评价pcn阈值
# pcn = 20
drawResults = True
runForFullIterations = False


# 读取csv文件Dev数据
data = []
with open('sobol.csv', encoding="utf-8") as f:
    reader = csv.reader(f)
    data = list(reader)               # 转成列表
    # print(float(data[1][0]))
    # for line in reader:
    #     # print(line)  # 打印文件每一行的信息
    #     data.append(line)


sbpp = RRTPlanner()           # 继承RRTPlann 类
path= sbpp.RRT(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations,
               resolution, drawResults, runForFullIterations,rand_fre,num_randpoint,angel_min,data,node_k,pcn) # 产生路径
# plt.show()                   # 显示路径图

