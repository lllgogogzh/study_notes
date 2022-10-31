# python语言基础

## ·基础知识

### 一、导入python模块

相当于include

语法：import xxx 或from xxx import fun_name

```python
import math
from math import sqrt
```

### 二、数字及数字的格式化输出

#### 1、普通运算符

```python
print(1/2)   #输出0.5
print(1//2)  #输出0
```

python可以处理很大数据

```python
print(2**630 * 100000)  #输出2^630×100000
```

#### 2、进制转换

四个函数：

①bin() ：将某进制数转换为2进制数

②oct() ：将某进制数转换为8进制数

③int() ：将某进制数转换为10进制数

④hex() ：将某进制数转换为16进制数

用法：bin("456789",10)    **函数名（“要转换的数”，转换之前数的进制）**

### 3、数据的格式化输出

### 三、获取用户输入

```python
name=input("请输入您的名字：")
age=int(input("请输入您的年龄："))
salary=float(input("请输入您的收入："))
```



### 四、函数

### 五、字符串基础

#### 1、单引号字符串和转义符

#### 2、拼接字符串

#### 3、保持字符串原有特性

#### 4、长字符串

### 六、条件语句

#### 1、if语句

```python
if a>10 and a<=20:
    ...
    ...
elif a==0 or a<-1:
    ...
    ...
else:
    ...
    ...

```



#### 2、比较运算符

介绍python中新增的比较运算符：

| 逻辑表达式 |                             描述                             |
| :--------: | :----------------------------------------------------------: |
|   x is y   |                       x和y是同一个对象                       |
| x is not y |                      x和y不是同一个对象                      |
|   x in y   | x是y容器的成员。如：y是列表[1,2,3,4]，那么1是y的成员，12就不是 |
| x not in y |                       x不是y容器的成员                       |

#### 3、断言

断言（assertions）的使用方式类似if，是在不满足条件时，会直接抛出异常。类似于下面的if语句

```python
if not condition:  #如果不满足条件，直接抛出异常，程序中断
	crash program
```

可以用于检测程序在某个地方是否满足条件，进而检测bug。

assert用法：在assert关键词后面指定断言的条件表达式，如果条件表达式的值是False，那么就会抛出异常。且断言后面的语句不会被执行。

```python
name="Bill"
assert name =="Bill"

age=20
assert 0<age<10  , "年龄必须小于10岁"
```



### 七、循环语句

#### 1、while循环

```python
while x<10:
    x=x+1
```



#### 2、for循环

```python
numbers=[1,2,3,4,5]
for number in numbers:
    print(number)

for i in range(10):   #把0~9这10个数字赋给i，并且执行循环
    print(i)
    
range(2,6)  #[2,3,4,5]
range(1,7,2) #[1,3,5]  第三个数字代表步长
```

#### 3、跳出循环

break：跳出当前循环

continue：直接执行下一次循环

### 八、其他常用语句

#### 1、使用exec和eval执行求值字符串