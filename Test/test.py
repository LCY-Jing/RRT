import turtle
import time

# 曲线移动
def curveMove():
    for i in range(200):
        turtle.right(1)
        turtle.forward(1)


def drawHeart():
    turtle.speed(10) # 画笔速度调到最高
    turtle.color('red','pink')
    turtle.begin_fill()
    turtle.left(140) # 逆时针旋转140度
    turtle.forward(111.65) # 向前移动111.65个像素
    curveMove() # 画曲线
    turtle.left(120) # 逆时针旋转120度
    curveMove() # 继续画曲线
    turtle.forward(111.65) # 向前移动111.65个像素
    turtle.end_fill()
    time.sleep(10)

if __name__ == '__main__':
    drawHeart()


