# 数据可视化库：Matplotlib

## 一、Matplotlib开发环境搭建



## 二、基础知识

### ·第一个Matplotlib程序

用Matplotlib来绘制一个一元二次方程曲线，即y=x^2。

计算机通过程序绘图有两种方式：位图和矢量图。位图就是用一个一个像素点绘制的图形；而矢量图是将多个点进行连接的图形。Matplotlib采用矢量图的绘制方式，也可以理解为绘制很多很多点。

```python
import matplotlib.pyplot as plt
#生成200个点的x坐标
X=range(-100,101)
#生成200个点的y坐标
Y=[x**2 for x in X]
#绘制一元二次曲线
plt.plot(X,Y)
#将一元二次曲线保存为result.jpg
plt.savefig('result1.jpg')
#显示绘制曲线
plt.show()
```



### ·绘制正弦曲线和余弦曲线

### ·绘制随机点

使用scatter函数可以绘制随机点，需要点的x y坐标

```python
import random
import matplotlib.pyplot as plt
count=1024
#随机参数1024个随机点的X、Y值
X=[random.random()*100 for i in range(count)]
Y=[random.random()*100 for i in range(count)]
#绘制1024个随机点
plt.scatter(X,Y)
#显示绘制的随机点
plt.show()
```

### ·绘制柱状图

使用bar函数可以绘制柱状图。柱状图需要水平的x坐标值，以及每一个x坐标对应的y坐标值，从而形成柱状的图。

```python
import matplotlib.pyplot as plt
#绘制柱状图
x=[1980 1985 1990 1995]
y=[100 300 400 500]
#x坐标，y坐标，柱宽度（越大越宽）
plt.bar(x,y,width=3)
plt.show()
```



### ·绘制直方图和盒状图

### ·绘制饼图

使用pie函数可以绘制饼图。饼图一般是用来呈现比例的。pie函数用法简单，只要传入比例数据即可。

```python
import matplotlib.pyplot as plt
data = [5,67,23,43,64]
#绘制饼图
plt.pie(data)
#显示饼图
plt.show()
```



## 三、定制颜色和样式

这里会介绍如何通过API定制曲线、离散点、柱状图、饼图的颜色，以及曲线类型和填充类型

### ·定制曲线的颜色

plot函数可以通过color关键字参数指定曲线的颜色。

## 四、注释

