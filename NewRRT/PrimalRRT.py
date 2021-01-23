import numpy as np
import matplotlib.pyplot as plt
from NewRRT.InverseSequence import InverseSquence
import random
import math

# 定义起点、终点位置
start_x=0
start_y=0

goal_x=100
goal_y=-2

max_size=1000
truepath=[]

#初始化起点和终点
px=start_x
py=start_y

foot_length=10   #步长
i=0
x=[0]
y=[0]

#x1,y1不符合要求的点
x1=[0]
y1=[0]

#2.生成随机点，然后判断新生成的路径是否在障碍物上，如果符合条件，则存放在队列中
for j in range(1000):
    num1=random.random()
    num2=random.random()
    # num1 = InverseSquence.RadicalInverse(2,j)
    # num2 = InverseSquence.RadicalInverse(2,j)
    random_x = 120*num1
    random_y = 5*(-0.5+num2)     # 产生随机点

    if random_x>px:           # 产生横坐标大于起点
        new_x=px+(random_x-px)/foot_length    # 产生新节点的规则
        new_y=py+(random_y-py)/foot_length
        # 75<x80  1<y<2
        if new_x > 50 and new_x < 55 and new_y < 2 and new_y > 0.5:           # 判断是不是碰到了障碍物
            flag_1=1                                                 # 说明在右上角障碍物内
        else:
            flag_1=0

        #35<x40 1<y<2
        if new_x>20 and new_x<25 and new_y<2 and new_y>0.5:            #左上角障碍物
            flag_11=1
        else:
            flag_11=0

        #35<x<40 -1<y<0
        if new_x > 20 and new_x < 25 and new_y < -0.5 and new_y>-2:       #左下角障碍物
            flag_111=1
        else:
            flag_111=0

        # 35<x<40 -1<y<0
        if new_x > 50 and new_x < 55 and new_y < -0.5 and new_y > -2:    #右下脚障碍物
            flag_1111 = 1
        else:
            flag_1111 = 0
        distance_c=math.sqrt((goal_x-px)**2+(goal_y-py)**2)          #目标点与起点的距离
        distance_a=math.sqrt((goal_x-new_x)**2+(goal_y-new_y)**2)    #目标点与新节点的距离
        if distance_c>distance_a:            # 这个策略不好不应该是小于目标点与起点的距离，这样有时候会陷入一个局部死解
            flag_2=1
        else:
            flag_2=0

        #新节点与目标点的距离小于起点与目标点的距离且没碰到障碍物则加入这个新的节点
        if flag_1==0 and flag_2==1 and flag_11==0 and flag_111==0 and flag_1111 == 0:
        # if flag_2 == 1:
            x.append(new_x)
            y.append(new_y)
            px=new_x
            py=new_y
        else:
            x1.append(new_x)
            y1.append(new_y)


#3. 绘制图形部分，绘制障碍物以及机器人所要行走的路径
qiang1_x=[]
qiang1_y=[]
qiang2_x=[]
qiang2_y=[]
# 障碍物坐标
qiang3_x=[]
qiang3_y=[]
qiang4_x=[]
qiang4_y=[]

for i in range(100000):
    # 什么意思？制造障碍物墙
    qiang1_x.append(random.randint(50,55))
    qiang1_y.append(0.5+2*random.random())
plt.scatter(qiang1_x,qiang1_y,s=10,color="blue")     #画墙颜色是“蓝色”scatter函数是画点

for i in range(100000):
    qiang2_x.append(random.randint(20,25))           #第二个障碍物
    qiang2_y.append(0.5+2*random.random())
plt.scatter(qiang2_x,qiang2_y,s=10,color="blue")

for i in range(100000):
    qiang3_x.append(random.randint(50,55))
    qiang3_y.append(-0.5-2*random.random())
plt.scatter(qiang3_x,qiang3_y,s=10,color="blue")

for i in range(100000):
    qiang4_x.append(random.randint(20,25))
    qiang4_y.append(-0.5-2*random.random())
plt.scatter(qiang4_x,qiang4_y,s=10,color="blue")

#起点、终点
plt.scatter(start_x,start_y,s=100,color="black")        #scatter应该是画点
plt.scatter(goal_x,goal_y,s=100,color="black")

#RRT路径
plt.plot(x,y,color="red")
# plt.plot(x1,y1,color="yellow")
#plt.plot(x,y) #plot还有很多参数，可以查API修改，如颜色，虚线等
plt.title("");
plt.xlabel("x");
plt.ylabel("y");
plt.show()

