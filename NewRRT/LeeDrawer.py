from shapely.geometry import Point, Polygon, LineString
from NewRRT.LeeEnvironment import plot_environment, plot_line, plot_poly
from math import sqrt
import matplotlib.pyplot as plt

# 画路径
def draw_results(algo_name, path, V, E, env, bounds, object_radius, resolution, start_pose, goal_region, elapsed_time):
    """
    Plots the path from start node to goal region as well as the graph (or tree) searched with the Sampling Based Algorithms.

    Args:
        algo_name (str): The name of the algorithm used (used as title of the plot)       算法名字
        path (list<(float,float), (float,float)>): The sequence of coordinates traveled to reach goal from start node     从起点到终点的坐标序列
        V (set<(float, float)>): All nodes in the explored graph/tree   图/数中的所有节点
        E (set<(float,float), (float, float)>): The set of all edges considered in the graph/tree   图/树中所有边的集合
        env (yaml environment): 2D yaml environment for the path planning to take place       二维环境下的路径规划
        bounds (int, int int int): min x, min y, max x, max y of the coordinates in the environment.
        object_radius (float): radius of our object.
        resolution (int): Number of segments used to approximate a quarter circle around a point.
        start_pose(float,float): Coordinates of initial point of the path.
        goal_region (Polygon): A polygon object representing the end goal.
        elapsed_time (float): Time it took for the algorithm to run

    Return:
        None

    Action:
        Plots a path using the environment module.
    """
    # originalPath原始曲线，pruningPath修建后的枝
    originalPath,pruningPath=path
    graph_size = len(V)
    path_size = len(originalPath)        # 路径长度，len应该是记录数组大小的
    # Calculate path length
    path_length1 = 0.0
    path_length2 = 0.0
    for i in range(len(originalPath)-1):
        path_length1 += euclidian_dist(originalPath[i], originalPath[i+1])    # 计算两点之间的距离
    for i in range(len(pruningPath)-1):
        path_length2 += euclidian_dist(pruningPath[i], pruningPath[i+1])

    # Create title with descriptive information based on environment, path length, and elapsed_time
    title = algo_name + "\n" + str(graph_size) + " Nodes. " + str(len(env.obstacles)) + " Obstacles. Path Size: " + str(path_size) + "\n Path Length: " + str([path_length1,path_length2]) + "\n Runtime(s)= " + str(elapsed_time)

    # Plot environment
    env_plot = plot_environment(env, bounds)
    # Add title
    env_plot.set_title(title)
    # Plot goal
    # plot_poly(env_plot, goal_region, 'green')
    # Plot start
    # buffered_start_vertex = Point(start_pose).buffer(object_radius, resolution)
    # plot_poly(env_plot, buffered_start_vertex, 'red')

    # Plot Edges explored by ploting lines between each edge
    # 通过在每条边之间绘制线条来绘制边缘
    for edge in E:
        # line = LineString([edge[0], edge[1]])    # egde0代表的是近节点，edge1代表的应该是新节点
        plot_line(env_plot, edge)                # LineString的意思是将坐标点变成一维的一个个数
    # Plot path
    plot_path(env_plot, originalPath, object_radius,'black')       # 原始曲线用黑色表示
    plot_path(env_plot, pruningPath, object_radius,'red')          # 减枝后的曲线用红色表示


def euclidian_dist(point1, point2):
    return sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2 + (point2[2]-point1[2]))

def plot_path(env_plot, path, object_radius,colorset):
    # Plots path by taking an enviroment plot and ploting in red the edges that form part of the path
    # line = LineString(path)
    # x, y, z = line.xyz               # 把x和y和z坐标的数值单独分离出来
    x = [p[0] for p in path]
    y = [p[1] for p in path]
    z = [p[2] for p in path]
    env_plot.plot(x, y,z, color=colorset, linewidth=3, solid_capstyle='round', zorder=1)



