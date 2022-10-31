# 科学计算库：NumPy

## 一、NumPy开发环境搭建

## 二、Numpy数组

### 1、创建多维数组

numpy模块的array函数可以生成多维数组。还可以用shape函数获得多维数组的维数。

例子：

```python
from numpy import *
a = arange(5)
print(a.shape)  #输出数组每一维度的元素个数
```

### 2、获取数组值和数组的分片

```python
from numpy import *

a=array([[1,2,3],[4,5,6],[7,8,9]])
print(a[0,0]) #取第一行第一列的数据

print(a[0:1])	#运行结果[1,2,3]
print(a[0:1][0])#先将3*3数组变为1*3数组，之后获取1*3数组的第一行，结果为[1,2,3]
print(a[0:2])	#将3*3数组变为2*3数组，即取前两行
b = a[0:]
```

### 3、改变数组的长度

通过ravel方法或者flatten方法可以将多维数组变成一维数组。通过reshape方法可以将一维数组变成二维、三维或多维数组。

```python
from numpy import *
b = arange(24).reshape(2,3,4)#将一维数组变成三维数组

b1=b.ravel()   #将b1变为一维数组
b2=b.flatten() #将b2变为一维数组

b.shape = (6,4) #将三维数组b变为二维数组，为6行4列

b3 = b.transpose()	#数组转置

b.resize((2,12))	#将三维数组变为二维数组，2行12列
```

### 4、水平数组组合

现在有两个3*2的数组A和B

A=0  1  2              B = 6  7   8

​	3  4  5                      4  1   5

则使用函数hstack(A ,B)得到

C=[A,B]，即C= 0  1  2  6  7  8 

​                          3  4  5  4  1  5

```python
from numpy import *
a=arange(9).reshape(3,3)
b = a*3
c = a*5

hstack(b,c)
hstack(a,b,c)
```

### 5、垂直数组组合

使用函数vstack函数可以实现数组垂直组合

### 6、水平/垂直分隔数组

使用hsplit函数 和 vsplit函数 ， 用法：hsplit(A,2)，分两份。

### 7、将数组转换为列表

使用tolist方法可以将一维、二维数组转换为列表，用法：l=a.tolist()

## 三、NumPy常用函数

### 1、存取函数

savetxt和loadtxt，将一维数组存储，或将文本中的数据读出，以array数组形式返回。

```python
from numpy import *
a = arange(20)
savetxt("a.txt",a,fmt='%d')		#将数组a以整数格式保存成a.txt文件
savetxt("b.txt",a,fmt='%.2f')		#将数组a以小数格式保存成b.txt文件
reada=loadtxt("a.txt",dtype='int')	#从a.txt文件以整数形式读取文本，返回Numpy数组
readb=loadtxt("b.txt",dtype='float')

x = arange(16).reshape(4,4)
savetxt("x.txt",x,fmt='%d')			#存储二维数组
loadtxt("x.txt",dtype='int')		#装载二维数组
```

### 2、读写CSV文件