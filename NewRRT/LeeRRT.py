"""
RRT算法与Halton序列结合，把节点扩展的随机过程用Halton序列
"""
import math
import random
import time
from NewRRT.LeeDrawer import draw_results

from shapely.geometry import LineString, Point

from NewRRT import InverseSequence
from NewRRT.LeeHalton import LeeHalton
from sobol_seq import sobol_seq


class LeeRRTPlanner():
    def __init__(self):
        super(LeeRRTPlanner,self).__init__()

    def initialise(self,environment,bounds,start_pose,goal_region,object_radius,steer_distance,num_iterations,resolution,runForFullIterations):
        """
        初始化参数
        environment: 障碍物环境
        bounds: 路径边界
        start_pose：起点
        goal_refion: 目标区域
        object_radius: 对象半径
        steer_distance: 步长
        num_iterations: 迭代次数
        resolution:
        :return:
        """
        self.env = environment
        self.obstacles = environment.obstacles
        self.bounds = bounds
        self.minx, self.miny, self.minz, self.maxx, self.maxy, self.maxz = bounds
        self.start_pose = start_pose
        self.goal_region = goal_region
        self.obj_radius = object_radius
        self.N = num_iterations

        self.resolution = resolution
        self.steer_distance = steer_distance

        self.V = set()           # 建立空集存储节点
        self.E = set()           # 建立空集存储随机树

        self.child_to_parent_dict = dict()
        self.runForFullIterations = runForFullIterations
        self.goal_pose = self.Centroid(goal_region)
        # self.goal_pose = (goal_region.centroid.coords[0])  # 目标点

    def RRT(self,environment,bounds,start_pose,goal_region,object_radius,steer_distance,num_iterations,resolution, drawResults, runForFullIterations, RRT_Flavour = "RRT"):
        self.env = environment
        self.random_i = 0

        self.initialise(environment, bounds, start_pose, goal_region,object_radius, steer_distance, num_iterations,resolution, runForFullIterations)

        x0,y0,z0 = start_pose
        x1,y1,z1 = self.Centroid(goal_region)
        start = (x0,y0,z0)              # 起点
        goal = (x1,y1,z1)               # 终点
        elapsed_time = 0                # 运行时间
        path=[]                         # 存储路径

        # 考虑特殊情况
        if start == goal:
            path = [start,goal]
            self.V.union([start,goal])
            self.E.union([(start, goal)])
        # 判断是否有从起点直通终点的路径，且该路径中间是否经过障碍物
        elif self.isEdgeCollisionFree(start,goal):
            path = [start,goal]         # 说明有一条路径从起点直通终点且不经过障碍物
            self.V.union([start, goal])
            self.E.union([(start, goal)])

        # 运行RRT算法
        else:
            if RRT_Flavour == "RRT":
                start_time = time.time()      # 记录该算法的开始时间
                path = self.RRTSearch()       # 开始进行路径搜索
                elapsed_time = time.time()-start_time  # 计算算法运行时间

        # 画出路径
        if path and drawResults:
            draw_results("RRT", path, self.V, self.E, environment, bounds, object_radius, resolution, start_pose,
                         goal_region, elapsed_time)

        return path  # 返回路径





    def RRTSearch(self):
        """
        RRT路径搜索算法
        :return:
        """
        path = []    # 定义路径
        path_length = float('inf')   # 路径默认值为正无穷
        tree_size = 0
        path_size = 0
        self.V.add(self.start_pose)   # start_pose肯定是路径中的一个节点那么将其加入路径中
        # goal_centroid = self.get_centroid(self.goal_region)   # 获取目标图形的形心,该函数z坐标不确定对不对
        goal_centroid = self.Centroid(self.goal_region)

       # 迭代采样过程
        for i in range(self.N):
            if i >= self.N-1:
                self.runForFullIterations == True     # 意思是迭代完了
            if(random.random()>=1.95):                # 这种情况不会存在
                random_point = goal_centroid
            else:
                # 采集随机点
                random_point = self.get_cillision_free_random_point()
            nearest_point = self.find_nearest_point(random_point)
            new_point = self.steer(nearest_point,random_point)
            if self.isEdgeCollisionFree(nearest_point,new_point):
                self.V.add(new_point)
                self.E.add((nearest_point,new_point))
                self.setParent(nearest_point,new_point)          # 设置nearest_point 为 new_point

                if self.isAtGoalRegion(new_point):
                    if not self.runForFullIterations:
                        path,tree_size,path_size,path_length = self.find_path(self.start_pose,new_point)
                        break
                    else:
                        tmp_path, tmp_tree_size, tmp_path_size, tmp_path_length = self.find_path(self.start_pose,
                                                                                                 new_point)
                        if tmp_path_length < path_length:
                            path_length = tmp_path_length
                            path = tmp_path
                            tree_size = tmp_tree_size
                            path_size = tmp_path_size

        uniPruningPath = self.uniPruning(path)  # 这里是剪枝操作
        # If no path is found, then path would be an empty list.  如果没有路径那么path应该是空列表。
        return [path, uniPruningPath]








    def Centroid(self,point_region):
        """
        获取立体图形的中心
        :return:
        """
        x = (point_region[2][0] - point_region[0][0]) / 2 + point_region[0][0]
        y = (point_region[1][1] - point_region[0][1]) / 2 + point_region[0][1]
        z = (point_region[4][2] - point_region[0][2]) / 2 + point_region[0][2]
        return (x, y, z)

    def uniPruning(self,path):
        """
        剪枝操作
        :param path: 路径
        :return:
        """
        unidirectionalPath = [path[0]]  # 起点
        pointTem = path[0]  # 起点
        for i in range(3, len(path)):  # 去掉起点，终点
            # pointTem,path[i] 两个点连线没有障碍物
            if not self.isEdgeCollisionFree(pointTem, path[i]):
                pointTem = path[i - 1]
                unidirectionalPath.append(pointTem)
        unidirectionalPath.append(path[-1])  # path[-1] 代表path 的最后一个值
        return unidirectionalPath


    def find_path(self,start_point,end_point):
        """
        从随机节点中选出路径
        :param start_point:
        :param end_point:
        :return:
        """
        path = [end_point]
        tree_size, path_size, path_length = len(self.V), 1, 0  # tree_size 树的大小
        current_node = end_point  # 当前节点就是新加入的节点
        previous_node = None
        target_node = start_point  # 从末端点到起点，起点就是目标点
        while current_node != target_node:
            parent = self.getParent(current_node)  # 寻找当前节点前一个节点（父节点）
            path.append(parent)  # 插入新点
            previous_node = current_node
            current_node = parent  # 当前节点变成向目标点进一步的节点
            path_length += self.euclidian_dist(current_node, previous_node)  # 两节点的距离，目的是求得整个路径的长度
            path_size += 1  # 计算节点个数
        path.reverse()  # 反向列表中的元素，因为是逆序找的
        return path, tree_size, path_size, path_length

    def isAtGoalRegion(self,point):
         """
         判断新点是否到达目标区域
         :param point:
         :return:
         """
         # buffered_point = Point(point).buffer(self.obj_radius, self.resolution)
         # intersection = buffered_point.intersection(self.goal_region)     # 返回几何图形的交集
         # inGoal = intersection.area / buffered_point.area                 # area几何的无单位面积
         # return inGoal >= 0.5  # 原始定为0.5
         # return inGoal >= 0.01

         return self.euclidian_dist(point,self.goal_pose) <= 3.9    # 这里不能这么大






    def getParent(self, vertex):
        """
        返回父节点
        :param vertex:
        :return:
        """
        return self.child_to_parent_dict[vertex]

    def setParent(self,parent,child):
        """
        设置父节点
        :return:
        """
        self.child_to_parent_dict[child] = parent

    def steer(self,from_point,to_point):
        """
        根据步长确定新增节点
        :param from_point:最近节点（线段起点）
        :param to_point: 随机点（线段终点）
        :return:
        """
        fromPoint_buffered = Point(from_point).buffer(self.obj_radius, self.resolution)
        toPoint_buffered = Point(to_point).buffer(self.obj_radius, self.resolution)
        if fromPoint_buffered.distance(toPoint_buffered) < self.steer_distance: # 两点之间的距离小于步长
            return to_point
        else:    # 以步长为长度扩展节点
            from_x,from_y,from_z = from_point
            to_x,to_y,to_z = to_point
            new_pointx = (to_x-from_x)*self.steer_distance/fromPoint_buffered.distance(toPoint_buffered) + from_x
            new_pointy = (to_y - from_y) * self.steer_distance / fromPoint_buffered.distance(toPoint_buffered) + from_y
            new_pointz = (to_x - from_z) * self.steer_distance / fromPoint_buffered.distance(toPoint_buffered) + from_z
            new_point = (new_pointx,new_pointy,new_pointz)
            return new_point




    def find_nearest_point(self,random_point):
        """
        寻找随机点的最近点

        :return: 最近点
        """
        closest_point = None
        min_dist = float('inf')
        for vertex in self.V:
            # self.V 包含随机树上所有节点
            euc_dist = self.euclidian_dist(random_point, vertex)
            if euc_dist < min_dist:
                min_dist = euc_dist
                closest_point = vertex
        return closest_point


    def get_cillision_free_random_point(self):
        """
        找到一个不在障碍物里的可用随机点
        :return: 可用随机点
        """
        # 运行直到找到一个有效的点
        while True:
            self.random_i = self.random_i +1
            point = self.get_random_point(self.random_i)
            buffered_point = Point(point).buffer(self.obj_radius,self.resolution)
            # 判断随机点是否在障碍物里
            if self.PointCollisionFree(buffered_point):
                return point

    def euclidian_dist(self,point1,point2):
        """
        计算两个点之间的距离
        :param point1:
        :param point2:
        :return:
        """
        return math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2 + (point2[2]-point1[1])**2)

    def PointCollisionFree(self,point):
        """
        判断点是否在障碍物里
        :return:
        """
        for obstacle in self.obstacles:
            if obstacle.contains(point):
                return False
        return True


    def get_random_point(self, rand_i):
        """
        获取随机点的x,y,z值
        :param random_i: 迭代次数
        :return: 坐标值
        """
        x = self.minx + LeeHalton.halton(rand_i,2)*(self.maxx-self.minx)
        y = self.miny + LeeHalton.halton(rand_i,3)*(self.maxy-self.miny)
        z = self.minz + LeeHalton.halton(rand_i,4)*(self.maxz-self.minz)
        # x = self.minx + random.random() * (self.maxx - self.minx)
        # y = self.miny + random.random() * (self.maxy-self.miny)
        # z = self.minz +  random.random() * (self.maxz-self.minz)
        return (x,y,z)

    def get_centroid(self,region):
        """
        功能：获取图形的形心
        :param region:图形
        :return: 图形形心
        """
        centroid = region.centroid.wkt
        filtered_vals = centroid[centroid.find("(")+1:centroid.find(")")]
        filtered_x = filtered_vals[0:filtered_vals.find("")]
        filtered_y = filtered_vals[filtered_vals.find("")+1:-1]
        filtered_z = filtered_vals[filtered_vals.find("")+1:-1]
        (x,y,z) = (float(filtered_x),float(filtered_y),float(filtered_z))
        return (x,y,z)












    def isEdgeCollisionFree(self,point1,point2):
        """
        # 连接两个点的直线中间是否经过障碍物，是否超出界限
        point1,point2 为要判断的两个点
        :param point1:
        :param point2:
        :return:
        """
        # 判断point2是否超出了边界
        if self.isOutofBounds(point2):
            return False
        line = LineString([point1,point2])
        expanded_line = line.buffer(self.obj_radius,self.resolution)
        for obstacle in self.obstacles:
            if expanded_line.intersects(obstacle):
                return False
        return True


    def isOutofBounds(self,point):
        x = point[0]
        y = point[1]
        z = point[2]
        # obj_radius是对象半径，就是走这个路径的对象。这里考虑了实际情况
        if((x-self.obj_radius)<self.minx):
            return True
        if((y-self.obj_radius)<self.miny):
            return True
        if ((z - self.obj_radius) < self.minz):
            return True
        if ((x+self.obj_radius)>self.maxx):
            return True
        if ((y+self.obj_radius)>self.maxy):
            return True
        if ((z+self.obj_radius)>self.maxz):
            return True
        return False











