from __future__ import division

import csv

from shapely.geometry import Point, LineString
import random
import math
import numpy as np
import time
import os
from RRTGai.Gdrawer import draw_results
from NewRRT.InverseSequence import InverseSquence
from RRTGai.GHalton import GHalton

"""算法实现
    
"""
class RRTPlanner():

    """Plans path using an algorithm from the RRT family.

    Contains methods for simple RRT based search, RRTstar based search and informed RRTstar based search.

    """
    random_i = 0
    ghalton = GHalton()  # 继承类

    def initialise(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations,rand_fre,num_randpoint,angel,data):
        """Initialises the planner with information about the environment and parameters for the rrt path planers

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.    障碍物环境
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world. 可行域边界
            start_pose( (float float) ): Starting x and y coordinates of the object in question.                路径起始点
            goal_region (Polygon): A polygon representing the region that we want our object to go to.          目标点
            object_radius (float): Radius of the object.                                                        对象半径
            steer_distance (float): Limits the length of the branches                                           树枝长度，应该是步长
            num_iterations (int): How many points are sampled for the creationg of the tree                     迭代次数
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            runForFullIterations (bool): If True RRT and RRTStar return the first path found without having to sample all num_iterations points.

        Returns:
            None
        """

        self.env = environment
        self.obstacles = environment.obstacles                    # 赋值环境中的障碍物
        self.bounds = bounds                                      # 边界
        self.minx, self.miny, self.maxx, self.maxy = bounds
        self.start_pose = start_pose
        self.goal_region = goal_region
        self.obj_radius = object_radius
        self.N = num_iterations                                  # 迭代次数
        self.resolution = resolution
        self.steer_distance = steer_distance                    #
        self.V = set()                                         # 建立新的无元素重合的集合
        self.E = set()
        self.child_to_parent_dict = dict()                    # key = child, value = parent 子代到父代
        self.runForFullIterations = runForFullIterations
        self.goal_pose = (goal_region.centroid.coords[0])        # 目标点

        self.rand_free = rand_fre                               # 目标偏置概率阀值
        self.num_randpoint = num_randpoint
        self.angel_min = angel

        self.random_i = 0
        self.data = data








    def RRT(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, drawResults, runForFullIterations,rand_fre,num_randpoint,angel,data, RRT_Flavour= "RRT"):
        """Returns a path from the start_pose to the goal region in the current environment using the specified RRT-variant algorithm.

        Args:
            environment (A yaml environment): Environment where the planner will be run. Includes obstacles.
            bounds( (int int int int) ): min x, min y, max x, and max y coordinates of the bounds of the world.
            start_pose( (float float) ): Starting x and y coordinates of the object in question.
            goal_region (Polygon): A polygon representing the region that we want our object to go to.
            object_radius (float): Radius of the object.
            steer_distance (float): Limits the length of the branches
            num_iterations (int): How many points are sampled for the creationg of the tree
            resolution (int): Number of segments used to approximate a quarter circle around a point.
            runForFullIterations (bool): If True RRT and RRTStar return the first path found without having to sample all num_iterations points.
            RRT_Flavour (str): A string representing what type of algorithm to use.
                               Options are 'RRT', 'RRT*', and 'InformedRRT*'. Anything else returns None,None,None.

        Returns:
            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree
        """
        self.env = environment

        # 初始化参数
        self.initialise(environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution, runForFullIterations,rand_fre,num_randpoint,angel,data)

        # Define start and goal in terms of coordinates. The goal is the centroid of the goal polygon.
        x0, y0 = start_pose
        x1, y1 = goal_region.centroid.coords[0]
        start = (x0, y0)                             # 起始节点
        goal = (x1, y1)                              # 目标节点
        elapsed_time=0                               # 运行时间
        path=[]

        # Handle edge case where where the start is already at the goal
        # 如果起点在目标点附近把起点终点放入path和两个空集中
        if start == goal:                                  # 判断路径从起始点是否到目标点
            path = [start, goal]                           # 路径是由起始点和目标点组成的数组
            self.V.union([start, goal])                    # 将起点终点坐标放入一个空间中
            self.E.union([(start, goal)])
        # There might also be a straight path to goal, consider this case before invoking algorithm
        # 在运行算法之前考虑有一条直的路径从起点到终点,就存入path和空集中
        elif self.isEdgeCollisionFree(start, goal):        # 考虑是否有一条直通目标的直线，在运行算法之前考虑这种情况
            path = [start, goal]
            self.V.union([start, goal])
            self.E.union([(start, goal)])
        # Run the appropriate RRT algorithm according to RRT_Flavour
        # 运行RRT算法
        else:
            if  RRT_Flavour == "RRT":
                start_time = time.time()                      # 计算规划路径时间
                path = self.RRTSearch()                       # 进行路径搜索，现在path包括原始路径和剪枝后的路径
                elapsed_time = time.time() - start_time       # 计算搜索路径的时间

        # 将路径点画出来
        if path and drawResults:              # drawResults是布尔变量，初始值是True
            draw_results("RRT", path, self.V, self.E, environment, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time)

        return path   # 返回路径

    # 使用RRT算法进行路径搜索
    def RRTSearch(self):
        """Returns path using RRT algorithm.

        Builds a tree exploring from the start node until it reaches the goal region. It works by sampling random points in the map and connecting them with
        the tree we build off on each iteration of the algorithm.

        Returns:

            path (list<(int,int)>): A list of tuples/coordinates representing the nodes in a path from start to the goal region #表示从起点到目标区域的路径中的节点的元组/坐标列表
            self.V (set<(int,int)>): A set of Vertices (coordinates) of nodes in the tree  #树中节点的一组顶点(坐标)
            self.E (set<(int,int),(int,int)>): A set of Edges connecting one node to another node in the tree 连接树中一个节点到另一个节点的边
        """

        # Initialize path and tree to be empty.


        path = []
        path_length = float('inf')        # 路径长度赋值为正无穷
        tree_size = 0
        path_size = 0
        self.V.add(self.start_pose)
        goal_centroid = self.get_centroid(self.goal_region)     # 获取目标点形心
        new_point = self.start_pose                             # 设置

        # Iteratively sample N random points in environment to build tree
        # 迭代采样过程中的N个随机点来构建树
        for i in range(self.N):       # N是迭代次数
            if i >= self.N-1:
                self.runForFullIterations == True
            rand_num =  random.random()   # 使用随机数
            # rand_num =float(self.data[i][4])
            """
            加入目标偏置策略
            """
            if rand_num < self.rand_free:            # 若是随机数小于目标偏向概率
                random_point = self.goal_pose
                # nearest_point = self.find_nearest_point(random_point)
            else:
                random_point1 = self.get_collision_free_random_point()       #收集随机点，这里是随机过程
                # The new point to be added to the tree is not the sampled point, but a colinear point with it and the nearest point in the tree.
                # This keeps the branches short
            # 通过建立与目标向量之间的角度值选取前四个
                thea=[]
                thea.clear()
                for randompoint_i in random_point1:                            # 遍历随机点
                    nearest_point = self.find_nearest_point(randompoint_i)
                    dis = self.euclidian_dist(randompoint_i,self.goal_pose)     # 找随机点附近最近点，最近点就是之前分支上所有的点离当前随机点最近的点
                    thea.append((dis,nearest_point,randompoint_i))             # 将计算出的角度值加入列表中
                thea.sort()                                       # 将thea元素按照thea_tem升序排列
                # # self.text_create("theadata",thea)
                for thea_tem in thea:
                    nearest_point = thea_tem[1]
                    random_point = thea_tem[2]
                    new_point = self.steer(nearest_point, random_point)  # 新点，newpoint又与最近点相关，像是RRT*，这里就把新点选出来了
                    # 如果说最近点与采样点连线之间没有障碍物，就把新点加入到树枝了。
                    if self.isEdgeCollisionFree(nearest_point, new_point):
                        self.V.add(new_point)  # V中表示加入new_point点
                        self.E.add((nearest_point, new_point))  # E中包含邻近点和新点，就是图中所显示的所有点
                        self.setParent(nearest_point, new_point)  # 设置父节点,nearest_point是new_point的父节点

                        # 判断新节点与目标点之间是否有一条直通的路径
                        # if self.isEdgeCollisionFree(new_point,self.goal_pose):
                        #     self.V.add(self.goal_pose)  # V中表示加入new_point点
                        #     self.E.add((new_point, self.goal_pose))  # E中包含邻近点和新点，就是图中所显示的所有点
                        #     self.setParent(new_point, self.goal_pose)  # 设置父节点,nearest_point是new
                        #     new_point = self.goal_pose                 # 将目标点赋值给新点

                        # If new point of the tree is at the goal region, we can find a path in the tree from start node to goal.
                        # 如果说树的新点在目标区域内，就可以找到一个从起始点到目标点路径，否则继续迭代寻找

                        break

                if self.isAtGoalRegion(new_point):
                    if not self.runForFullIterations:  # If not running for full iterations, terminate as soon as a path is found.
                        path, tree_size, path_size, path_length = self.find_path(self.start_pose,
                                                                                 new_point)  # findpath应该就是将新增的节点都放到path中，
                        break

                    # 基本不会执行else里的语句
                    else:  # If running for full iterations, we return the shortest path found.
                        tmp_path, tmp_tree_size, tmp_path_size, tmp_path_length = self.find_path(
                            self.start_pose, new_point)
                        if tmp_path_length < path_length:
                            path_length = tmp_path_length
                            path = tmp_path
                            tree_size = tmp_tree_size
                            path_size = tmp_path_size


                # random_tem = int(random.uniform(0,self.num_randpoint))
                # nearest_point = thea[random_tem][1]
                # random_point = thea[random_tem][2]



        num_i = i+1        # 计算算法迭代次数


        uniPruningPath,uniPruningPathAngel=self.uniPruningAndAngel(path)      # 这里是剪枝操作
        """
        这里对剪枝后的
        """
        uniPruningPathAngel1 = list(uniPruningPath)              # 这样是新复制了一个列表，如果直接等于相当于是付给了指针

        for j in range(len(uniPruningPathAngel1)-2):                  # 这个是对路径角度的一个处理
            dir1 = self.direction(uniPruningPathAngel1[j+1],uniPruningPathAngel1[j])
            dir2 = self.direction(uniPruningPathAngel1[j+1],uniPruningPathAngel1[j+2])
            angel_r = self.arangel(dir1,dir2)                   # 获取两个向量的夹角
            if angel_r < self.angel_min:
                path_tem = list(path)
                angel_insert1 = path_tem[uniPruningPathAngel[j+1][1]-1]       # 加入一个点
                # angel_insert2 = path_tem[uniPruningPathAngel[j+1][1]-2]
                uniPruningPathAngel1.insert(j+1,angel_insert1)                # 在第i点后插入一个数
                # uniPruningPathAngel1.insert(j + 2, angel_insert2)
                # uniPruningPathAngel[j+1][0] = angel_insert

            # else:

        # If no path is found, then path would be an empty list.  如果没有路径那么path应该是空列表。
        print(num_i)
        return [path,uniPruningPath,uniPruningPathAngel1,num_i]


    """
    ******************************************************************************************************************************************
    ***************************************************** Helper Functions *******************************************************************
    ******************************************************************************************************************************************
    """




    def arangel(self,pose1,pose2):
        """
        计算两个向量之间的夹角
        :param pose1:
        :param pose2:
        :return:
        """
        zero_pose = [0,0]



        angel = abs(math.acos((pose1[0]*pose2[0]+pose1[1]*pose2[1])/(self.euclidian_dist(zero_pose,pose1)*self.euclidian_dist(zero_pose,pose2))))
        return angel


    def direction(self,starpose,goalpose):
        """
        求两点确定的向量
        :param starpose:
        :param goalpose:
        :return:
        """
        goalpose = list(goalpose)
        starpose = list(starpose)
        dir = [0,0]
        dir[0] = goalpose[0] - starpose[0]
        dir[1] = goalpose[1] - starpose[1]
        return dir


    def sample(self, c_max, c_min, x_center, C):
        if c_max < float('inf'):      # 表示正无穷
            r= [c_max /2.0, math.sqrt(c_max**2 - c_min**2)/2.0, math.sqrt(c_max**2 - c_min**2)/2.0]      # cmax,cmin指的是边界上下界
            L = np.diag(r)         # 提取对角线数组
            x_ball = self.sample_unit_ball()
            random_point = np.dot(np.dot(C,L), x_ball) + x_center      # dot指的是两个数组的点积
            random_point = (random_point[(0,0)], random_point[(1,0)])
        else:
            random_point = self.get_collision_free_random_point()
        return random_point

    def sample_unit_ball(self):
        a = random.random()
        b = random.random()

        if b < a:
            tmp = b
            b = a
            a = tmp
        sample = (b*math.cos(2*math.pi*a/b), b*math.sin(2*math.pi*a/b))
        return np.array([[sample[0]], [sample[1]], [0]])

    def find_min_point(self, nearest_set, nearest_point, new_point):
        min_point = nearest_point
        min_cost = self.cost(nearest_point) + self.linecost(nearest_point, new_point)
        for vertex in nearest_set:
            if self.isEdgeCollisionFree(vertex, new_point):
                temp_cost = self.cost(vertex) + self.linecost(vertex, new_point)
                if temp_cost < min_cost:
                    min_point = vertex
                    min_cost = temp_cost
        return min_point

    def cost(self, vertex):
        path, tree_size, path_size, path_length = self.find_path(self.start_pose, vertex)
        return path_length


    def linecost(self, point1, point2):
        return self.euclidian_dist(point1, point2)
    # 把值放入字典中
    def getParent(self, vertex):
        return self.child_to_parent_dict[vertex]

    def setParent(self, parent, child):
        self.child_to_parent_dict[child] = parent

    # 在边界范围内选择一个随机点
    def get_random_point(self,rand_i,data):
        thea = []
        thea.clear()
        for i in range(self.num_randpoint):
            while True:
                rand_i = rand_i + 1
                # x = self.minx + float(data[rand_i][1+i*2]) * (self.maxx - self.minx)
                # y = self.miny + float(data[rand_i][2+i*2]) * (self.maxy - self.miny)
                x = self.minx + random.random() * (self.maxx - self.minx)
                y = self.miny + random.random() * (self.maxy - self.miny)
                pointtem = [x,y]
                buffered_point = Point(pointtem).buffer(self.obj_radius, self.resolution)  # obj_radius 半径
                if self.isPointCollisionFree(buffered_point):
                    thea.append(pointtem)
                    break

            # x = self.minx + random.random() * (self.maxx - self.minx)
            # y = self.miny + random.random() * (self.maxy - self.miny)
            # pointtem = [x, y]
            thea.append(pointtem)
        return thea



        # ghalton = GHalton()                                          # 继承类
        # x = self.minx + random.random() * (self.maxx - self.minx)
        # y = self.miny + random.random() * (self.maxy - self.miny)

        # x = self.minx + ghalton.halton_reader(rand_i,1) * (self.maxx - self.minx)
        # y = self.miny + ghalton.halton_reader(rand_i,3) * (self.maxy - self.miny)

        return (x, y)

    def get_collision_free_random_point(self):
        # Run until a valid point is found     # 运行直到找到一个有效的点
        point = []
        point.clear()

        # point = self.get_random_point()                 # 选择边界范围内的随机点
        point_tem = self.get_random_point(self.random_i,self.data)  # 选择边界范围内的随机点
        self.random_i = self.random_i + 1
        return point_tem




    # 如果随机点不在障碍物内，就认为该点随机成功
    def isPointCollisionFree(self, point):
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return False
        return True

    # 寻找最近点
    def find_nearest_point(self, random_point):
        closest_point = None
        min_dist = float('inf')
        for vertex in self.V:                                        # self.v包含所有产生的随机点
            euc_dist = self.euclidian_dist(random_point, vertex)     # 计算两点之间的距离，选取与random_point最近的V中的点
            if euc_dist < min_dist:
                min_dist = euc_dist
                closest_point = vertex
        return closest_point

    def isOutOfBounds(self, point):
        if((point[0] - self.obj_radius) < self.minx):
            return True
        if((point[1] - self.obj_radius) < self.miny):
            return True
        if((point[0] + self.obj_radius) > self.maxx):
            return True
        if((point[1] + self.obj_radius) > self.maxy):
            return True
        return False

    # 判断是否有一条直通目标点的路径
    def isEdgeCollisionFree(self, point1, point2):
        if self.isOutOfBounds(point2):                     # 目标点是否已经超出边界
            return False                                   # 返回错误信息
        line = LineString([point1, point2])                # 定义line类
        expanded_line = line.buffer(self.obj_radius, self.resolution)
        for obstacle in self.obstacles:
            if expanded_line.intersects(obstacle):       # 判断两图形是否相交，是返回TRUE否则返回false
                return False
        return True                                      # 图形不想交且在同一条直线上就返回True

    # 产生新节点的规则，这里算法中产生新节点的核心
    """
    这里是产生新节点的法则，遍历随机数的所有节点，找到与随机点最近的点，如果大于步长那么新节点在随机点与最近点的方向上，长度为步长。
    """
    def steer(self, from_point, to_point):
        fromPoint_buffered = Point(from_point).buffer(self.obj_radius, self.resolution)
        toPoint_buffered = Point(to_point).buffer(self.obj_radius, self.resolution)
        if fromPoint_buffered.distance(toPoint_buffered) < self.steer_distance:        # 分支长度（应该是步长），就是不让新点超过步长
            to_x, to_y = to_point
            new_point = (to_x, to_y)
            return new_point
        else:
            from_x, from_y = from_point
            to_x, to_y = to_point
            theta = math.atan2(to_y - from_y, to_x- from_x)     # 两个点连线与水平轴的夹角
            new_point = (from_x + self.steer_distance * math.cos(theta), from_y + self.steer_distance * math.sin(theta))
            return new_point

    def isAtGoalRegion(self, point):
        buffered_point = Point(point).buffer(self.obj_radius, self.resolution)
        intersection = buffered_point.intersection(self.goal_region)
        inGoal = intersection.area / buffered_point.area
        return inGoal >= 0.5            # 原始定为0.5
        # return inGoal >= 0.01

    # 计算两点之间的距离
    def euclidian_dist(self, point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    # end_point加入的新点，寻找路径的过程应该是末端向初始端前进
    def find_path(self, start_point, end_point):
        # Returns a path by backtracking through the tree formed by one of the RRT algorithms starting at the end_point until reaching start_node.
        path = [end_point]                                     # 将末端点加入路径中
        tree_size, path_size, path_length = len(self.V), 1, 0  # tree_size 树的大小
        current_node = end_point                               # 当前节点就是新加入的节点
        previous_node = None
        target_node = start_point                              # 从末端点到起点，起点就是目标点
        while current_node != target_node:
            parent = self.getParent(current_node)              # 寻找当前节点前一个节点（父节点）
            path.append(parent)                                # 插入新点
            previous_node = current_node
            current_node = parent                              # 当前节点变成向目标点进一步的节点
            path_length += self.euclidian_dist(current_node, previous_node)    # 两节点的距离，目的是求得整个路径的长度
            path_size += 1                                     # 计算节点个数
        path.reverse()                                         # 反向列表中的元素，因为是逆序找的
        return path, tree_size, path_size, path_length

     # region是传入的目标多边形区域，这个函数的作用是取该区域的形心
    def get_centroid(self, region):
        centroid = region.centroid.wkt
        filtered_vals = centroid[centroid.find("(")+1:centroid.find(")")]  # 过滤值
        filtered_x = filtered_vals[0:filtered_vals.find(" ")]
        filtered_y = filtered_vals[filtered_vals.find(" ") + 1: -1]
        (x,y) = (float(filtered_x), float(filtered_y))
        return (x,y)

    # 剪枝操作
    def uniPruning(self, path):
        unidirectionalPath=[path[0]]     # 起点
        pointTem=path[0]                 # 起点
        for i in range(3,len(path)):     # 去掉起点，终点
            # pointTem,path[i] 两个点连线没有障碍物+not就是有障碍物
            if not self.isEdgeCollisionFree(pointTem,path[i]):
                pointTem=path[i-1]
                unidirectionalPath.append(pointTem)
        unidirectionalPath.append(path[-1])        # path[-1] 代表path 的最后一个值
        return unidirectionalPath

        # 剪枝操作改进
    def uniPruningAndAngel(self, path):
        unidirectionalPath = [path[0]]           # 起点
        unidirectionalPathAngel = [(path[0],0)]
        pointTem = path[0]                       # 起点
        for i in range(3, len(path)):            # 去掉起点，终点
            # pointTem,path[i] 两个点连线有障碍物
            if not self.isEdgeCollisionFree(pointTem, path[i]):      # 有障碍物
                pointTem = path[i - 1]
                unidirectionalPath.append(pointTem)
                unidirectionalPathAngel.append((pointTem,i-1))
        unidirectionalPath.append(path[-1])  # path[-1] 代表path 的最后一个值
        unidirectionalPathAngel.append((path[-1],i))
        return unidirectionalPath,unidirectionalPathAngel

    def text_create(self,name, msg):
        # desktop_path = "D:\\PythonData\\"  # 新创建的txt文件的存放路径
        desktop_path = os.path.abspath(os.path.dirname(__file__))  # 新创建的txt文件的存放路径
        full_path = desktop_path + '\\' + name + '.txt'  # 也可以创建一个.doc的word文档
        file = open(full_path, 'w')
        file.write(str(msg))                 # msg要写入的信息!
        # file.close()


