"""
Sobol 序列实现
"""
from matplotlib import pyplot as plt
import random

from scipy.sparse.linalg.isolve.tests.demo_lgmres import M

"""
定义一个可以产生序列的类Halton序列、IntegerRadicalInverse、Sobol序列随机序列
"""
class InverseSquence():
    def __init__(self):
        super(InverseSquence,self).__init__()

    # Radical Inversion实现
    def IntegerRadicalInverse(Base , i):
        base = int(Base)
        i1 = int(i)
        numPoints = 1
        inverse = 0
        while i1 > 0 :
            inverse = int(inverse) * Base + (i1 % Base)
            numPoints = int(numPoints) * Base
            i1 = int(i1 / Base)
        return (inverse / numPoints)   # 将数镜像到小数点右边


    #基于浮点数的Radical Inversion实现
    def RadicalInverse(Base, I):
        base = int(Base)
        i = int(I)
        Digit = Radical = 1.0 / float(Base)
        Inverse = 0.0

        while i>0:
            Inverse = Inverse + Digit * float(i % base)
            Digit = Digit*Radical
            i = i / base
        return Inverse

    # Halton就只是找到互为质数的数字作为Base调用RadicalInverse就可以了
    # Halton 序列
    def HaLton(Dimension,Index):
        dimenson = int(Dimension)
        index = int(Index)
        # 直接用质数作为底数调用RadicalInverse即可
        return InverseSquence.RadicalInverse(dimenson,index)

    # Hammersley则和Halton几乎一样，只是第一维是uniform分布而已
    def Hammersley(self):
        pass


    # Sobol序列的所有维度都是基于2为底数的Radical Inversion。
    # 只不过每个维度有自己不同的生成矩阵（Generator Matrix）[公式]。
    # 因为是以2为底数，将数字从二进制中提取每一位的数字，以及和矩阵[公式]做运算，都可以用位操作（右移，异或等）来完成，非常高效。
    # Sobol序列实现
    def Sobol(I, Dimension):
        i = int(I)
        dimenson = int(Dimension)
        # 将i依次右移，提取二进制里的每一位
        k = 0
        r = 0

        while i > 0:
            i = i>>1
            if (i & 1) > 0:
                r = r ^ C[dimenson][k]
            k = k+1
            return r / float(1 << M)    # M 应该是维度吧

    # 随机序列
    # min与max代表随机数的上下限
    def RandomInverse(min,max):
        randoms = float(random.uniform(min,max))
        return randoms






if __name__ == '__main__':
    b = []
    length = 100
    for t1 in range(1,length+1):
        a = InverseSquence.Sobol(t1,1)
        # 生成0-1的随机实数
        # a = float(random.uniform(0,1))
        b.append(a)
    t2 = range(1,length+1,1)
    plt.plot(t2,b, '*',linewidth=3, solid_capstyle='round', zorder=1)
    plt.show()




