import time

import numpy as np
import math
import matplotlib.pyplot as plt

from numpy import concatenate
from numpy.linalg import norm
from sklearn import preprocessing
# from sklearn.preprocessing import MinMaxScaler
# from sklearn.preprocessing import StandardScaler

#开始

#得到测试集和训练集

#这里产生数组有两种方案，一种是线性800个包含0和1 linspace.一种是线性800个不包含1.arange
# 不包含1的就产生800个数，然后每个数/800
# reshape(-1,1) -1表示不管重组成几行 1表示重组成1列

U1=np.linspace(0,1,800).reshape(-1,1)
Y1=np.linspace(0,0,800).reshape(-1,1)
U2=np.linspace(0,1,200).reshape(-1,1)
Y2=np.linspace(0,0,200).reshape(-1,1)


hL=np.linspace(0,0,800).reshape(-1,1)
hL_opt_L=np.empty((800,1),float)
HL=np.empty((800,0),float)
OutputWeight_opt=np.empty((100,1),float)
T=np.empty((800,0),float)

# testArray=np.arange(0,10,1)
# testArray=np.linspace(0,9,10).reshape(-1,1)
# testArray2=np.linspace(10,19,10).reshape(-1,1)
# concatenate((testArray, testArray2), axis=1) 矩阵拼接
testArray4=np.array([[1],[2]])
testArray5=np.array([[3],[4]])
# testArray6=np.array([[1,2,3]])
# testArray7=np.array([4,5,6])
testArray3=concatenate((testArray4, testArray5), axis=1)
# print(testArray3)

i=0
while i<800:
    Y1[i]=0.2*math.exp(-(10*U1[i]-4)**2)+0.5*math.exp(-(80*U1[i]-40)**2)+0.3*math.exp(-(80*U1[i]-20)**2);
    i=i+1
i=0
while i<200:
    Y2[i]=0.2*math.exp(-(10*U2[i]-4)**2)+0.5*math.exp(-(80*U2[i]-40)**2)+0.3*math.exp(-(80*U2[i]-20)**2);
    i=i+1
TrainingData=np.concatenate((U1,Y1),axis=1)
TestingData=np.concatenate((U2,Y2),axis=1)

# 转换成行向量.transpose(1，0)表示1轴和0轴转置  [:,1] 选取第二列  [:,0]选取第一列
# 一维向量转置用reshape(-1,1) -1表示转换后不管有多少行，1表示转换后只有1列
#             reshape(1,-1) 1表示转换后只有1行，-1表示转换后不管有多少列

# 源代码这里是为了分别转换P、T，但是python不能归一化一维数组，所以无需这几句话
# Pn=TrainingData[:,0].reshape(1,-1)
# Tn=TrainingData[:,1].reshape(1,-1)
# TV_Pn=TestingData[:,0].reshape(1,-1)
# TV_Tn=TestingData[:,1].reshape(1,-1)

# 将测试集 训练集归一化
min_max_scaler = preprocessing.MinMaxScaler(feature_range=(-1,1))
min_max_scaler2 = preprocessing.MinMaxScaler(feature_range=(-1,1))
P = min_max_scaler.fit_transform(TrainingData[:,0:1])
T = min_max_scaler2.fit_transform(TrainingData[:,1:2])
TV_P = min_max_scaler.transform(TestingData[:,0:1])
TV_T = min_max_scaler2.transform(TestingData[:,1:2])

# size()：计算数组和矩阵所有数据的个数
# numpy.size(a, axis=None)
# axis = 0，返回该二维矩阵的行数
# axis = 1，返回该二维矩阵的列数
# shape ():得到矩阵每维的大小

#设置初始值
L = 1
Lmax = 100           # 最大节点数
Epsilon = 1e-2       # 允许误差
Tmax = 10            # 随机网络的最大配置时间
e0 = T
# e00 = 0.0
r = 0.999
Omega = np.empty((1,0),float)
W = np.empty((0,2),float)                # 产生数组
wL_opt = np.empty((0,1),float)
bL_opt = np.empty((0,1),float)
eL_1 = e0
E = np.array([])

# #产生(0,1)开区间的随机数
def npRand():
    npRand = np.random.rand()
    if(npRand==0):
        npRand = np.random.rand()    # 不包括 0
        return npRand
    else:
        return npRand

# mse均方根误差
def mse(x):
    # if(np.size(x,0)==1):
    #     return np.dot(x,(x.reshape(-1,1)))/len(x)
    # if(np.size(x,1)==1):
    #     return np.dot((x.reshape(1, -1)),x) / len(x)
    # if(np.size(x,0)!=1 and np.size(x,1)!=1):
    #     print('转置可能有问题')
        return np.sum(x**2)/np.size(x)

#训练时间计时，开始
start = time.time()
Lambda=150
label=0
while (L<=Lmax and norm(e0)>Epsilon):

    # for Lambda循环开始
    for Lambda in range(150,201,10):
        # for Tmax循环开始
        for i in range(1,11):
            wL=npRand()*2*Lambda-Lambda
            bL=npRand()*2*Lambda-Lambda

            #计算hL开始
            i=0
            while(i<P.size):
                hL[i]=(1/(1+math.exp(-(wL*P[i]+bL))))
                i=i+1
            # hL计算结束

            Mu=(1-r)/(L+1)
            XiL = (np.dot((eL_1.reshape(1,-1)),hL))*(np.dot((eL_1.reshape(1,-1)),hL)) / (np.dot((hL.reshape(1,-1)),hL))-(1-r-Mu)*(np.dot(eL_1.reshape(1,-1),eL_1))

            #XiL判断句开始
            if(XiL>=0):
                W=concatenate((W, [[wL,bL]]), axis=0)
                # W=np.row_stack((W, [wL,bL]))
                Omega=np.append(Omega,XiL)
                continue
            else:
                pass
                # print("XiL<0")
            #XiL>0 判断句

        #for Tmax循环

        # if W为空判断开始
        if(W.size==0):
            Tau=npRand()*(1-r)
            r=r+Tau
            continue
        else:
            OmegaMax=Omega.max()
            loc=np.argmax(Omega)
            wL_opt=concatenate((wL_opt,[[W[loc][0]]]),axis=0)
            bL_opt=concatenate((bL_opt,[[W[loc][1]]]),axis=0)
            # np.append(wL_opt,W[loc][0])
            # np.append(bL_opt, W[loc][1])
            if(wL_opt.size!=0):
                wL_opt_L=wL_opt[wL_opt.size-1]
                bL_opt_L=bL_opt[len(bL_opt)-1]
            else:
                label=label+1
                print('经过',label,'次')
                wL_opt_L=0
                bL_opt_L=0
            i=0
            while(i<np.size(hL_opt_L,0)):
                # 这里有个－后面没加括号 找了两个小时，记录一下 /微笑
                hL_opt_L[i]=1/(1+math.exp(-(wL_opt_L*P[i]+bL_opt_L)))
                i=i+1
            HL=concatenate((HL,hL_opt_L),axis=1)
            # 注意这里的Omega和W置空不能写np.array([0]),这是置零不是置空！
            # 因为这个原因L总是达到Lmax而不能跳出循环
            Omega = np.empty((1,0),float)
            Mu = []
            XiL = []
            W = np.empty((0,2),float)
            break
            # if W为空判断结束

        #for Lambda循环结束

    OutputWeight_opt = np.dot(np.linalg.pinv(HL) , T)
    # 矩阵[0:2] 选取[0,2)行 左闭右开
    # 矩阵[:,0:2] 选取[0,2)列 左闭右开
    # 矩阵[0:2,1:3] 选取[0,2)行 [1,3)列
    eL = T-np.dot(HL[:,0:L],OutputWeight_opt[0:L])
    # 数字用np.append() 矩阵用 concatenate
    E = np.append(E,norm(eL))
    eL_1 = eL
    # python的变量不能随意转换，源代码这里把矩阵e0直接转换成数字了
    e0 = math.sqrt(mse(eL))
    L=L+1


#训练时间计时，结束
end = time.time()
TrainingTime=end-start

TrainingAccuracy=math.sqrt(mse(eL))
E=np.append(norm(T),E)/math.sqrt(np.size(TrainingData,0))

# 测试时间计时 开始
start=time.time()
Q=TV_P.size
BiasMatrix=np.tile(bL_opt,(1,Q))
tempH=np.dot(wL_opt.reshape(-1,1),TV_P.reshape(1,-1))+BiasMatrix
i=0
j=0
H=np.empty((np.size(tempH,0),np.size(tempH,1)),float)
while(i<np.size(tempH,0)):
    while(j<np.size(tempH,1)):
        H[i][j]=1/(1+math.exp(-tempH[i][j]))
        j=j+1
    i=i+1
    j=0
Y=(np.dot(H.transpose(1,0),OutputWeight_opt)).transpose(1,0)
end=time.time()
TestingTime=end-start
# 测试时间计时 结束

TestingAccuracy=math.sqrt(mse(TV_T-Y.transpose(1,0)))

print('TrainingTime=',TrainingTime)
print('TestingTime=',TestingTime)
print('TestingAccuracy',TestingAccuracy)
print('number of L',L)

# 输出反归一化
TY=min_max_scaler2.inverse_transform(Y)

# 画图
plt.figure(1)
X1=np.arange(0,200,1)
plt.plot(X1,TestingData[:,1],'r-',label="Estimation")
plt.plot(X1,TY.reshape(-1,1),'bo',label="Real")
plt.legend(loc='upper right')
plt.xlabel('Sample number')
plt.ylabel('Outputs')
plt.title('SCN')

plt.figure(2)
X2=np.arange(0,L,1)
plt.plot(X2,E,'r-')
plt.xlabel('number of hidden nodes:L')
plt.ylabel('Training RMSE')

plt.show()
# exit(0)