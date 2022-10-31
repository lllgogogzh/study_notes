# 字典

本次学习一个新的数据结构，称为映射（mapping）。字典是Python语言中唯一内建的映射类型。字典中的值并没有特殊的顺序，但都存储在一个特定的键下，可以通过这个键找到与其对应的值。键可以是数字，字符串或者元组。

## 一、引入字典的目的

根据创建字典时指定的关键字查询值，而且查询的速度与字典中的数据量无关，方便查找。

例如，电话簿就是一个非常典型的字典应用。

## 二、创建和使用字典

创建字典：

```python
phoneBook = {"Bill":"1234","Mike":"4321","John":"7536"}
```

可以看到，一个字典是用一对大括号来创建的，键与值之间用冒号分隔，每一对键值之间用逗号分隔。如果大括号中没有任何值，就是一个空的字典。



在字典中，键是唯一的，这样才能通过键唯一定位某一个值。如果键不唯一，则后面的会覆盖先前的。

### 1、dict函数（创建）

可以用dict函数，通过其他映射（如其他的字典）或键值对的序列建立字典。

```python
items = []										#定义一个空的列表
while True:										
    key = input("请输入Key值：")					#输入key
    if key=="end":								#如果key为end，则退出循环，结束输入
        break
    value = input("请输入value值：")				#输入key对应的值
    keyValue=[key,value]						#用key和value创建一个列表
    items.append(keyValue)						#将key-value组成的列表添加到items中

d=dict(items)									#用dict函数将items转换为字典
print(d)										#输出
```



### 2、字典的基本操作（P118）

字典的很多操作与列表类似，下面的一些操作仍适用于字典：

·len(dict)：返回字典dict中元素（键值对）的数量

·dict[key]：返回关联到键key上的值，对于列表，key就是索引

### 3、字典的格式化字符串

### 4、序列与迭代

#### ①获取字典中key的值

在使用字典时，如果想知道字典里有哪些key，可以直接使用for语句对字典进行遍历

```python
dict={"x":"1","y":"2","z":"3"}
for key in dict:
    print(key,end='')

#此代码输出key的值，为x y z
```



#### ②同时获取字典中的key和value列表

如果要同时获取字典中的key和value，除了在上面的代码中使用dict[key]获取值外，还可以使用字典中的items方法同时获取key和value。

```python
dict = {'x':1,'y':2,'z':3}
for key,value in dict.items():
    print(key,value,end='')
    
#同时获取字典中的key和value值
#运行结果：x 1 y 2 z 3
```



#### ③并行迭代

如果想同时迭代两个或多个序列，那么可以使用range函数获取序列索引的范围，然后使用for语句进行迭代。对多个序列进行迭代，一般要求序列中元素个数相同。

```python
names = ["Gzh","Bill","John"]
ages=[20,30,40]

for i in range(len(names)):
    print(name(i),ages[i],end='')
```



#### ④压缩序列

使用zip函数将两个或多个序列对应的元素作为一个元组放到一起。若进行压缩的两个或多个序列的元素个数如果不相同，以元素个数最少的为准。

```python
names= ["Gzh","Bill","John"]
ages=[20,30]
for value in zip(names,ages):   #依次赋值给value，zip代表两个序列压缩后的结果
	print(value,end=" ")

#输出结果：('Gzh',20) ('Bill',30)
```



#### ⑤反转序列迭代

通过reversed函数可以将一个序列反转。

```python
values=[1,2,3]
for value in reversed(values):
    print(value,end=" ")
    
#输出结果：3 2 1
```



## 三、字典方法

### 1、clear方法

clear方法用于清空字典中的所有元素

```python
dict={......}
dict.clear()
```

### 2、copy方法与deepcopy方法

copy方法用于复制一个字典，该方法返回复制后的新字典

```python
dict={"a":1,"b":2}
dict1=dict.copy()
```

copy浅复制只能复制第一层字典的数据，以至于第二层及以下的所有数据，原字典和新字典都指向同一个值（比如第二层元素：序列等），修改一个后原字典和新字典都会改变。

解决这个问题，使用深复制：deepcopy函数，用法与copy相同

### 3、fromkeys方法

fromkeys方法用于根据key建立新的字典（该方法的返回值就是新字典）。

### 4、get方法

get方法用于更宽松的方式从字典中获取key对应的value。当使用dict[key]形式从字典中获取value时，如果key在dict中不存在，那么程序会抛出异常。若不想这种情况发生，就要用get方法。该方法在key不存在时，返回None值。

```python
dict={...}
print(dict.get('x',0))#get方法第二个参数表示：key不存在时的返回值
```



### 5、items方法和keys方法

items方法用于返回字典中所有的key-value对。获得的每一个key-value对用一个元组表示。items方法返回的值是一个被称为字典视图的特殊类型，可以被用于迭代（如使用在for循环中）。

### 6、pop方法和popitem方法

pop方法和popitem方法都用于弹出字典中的元素。

### 7、setdefault方法

### 8、update方法

### 9、values方法

