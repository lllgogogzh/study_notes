# 类和对象

与C++类似。但是需要注意：

1. 与类和实例无绑定关系的function都属于函数（function）；
2. 与类和实例有绑定关系的function都属于方法（method）。

方法与C++中的成员函数十分类似

详细地说明：**在模块中实现的函数只要导入这个模块的这个函数既可以任意使用了，但是在类中声明的必须导入这个类之后然后再通过创建实例或者类名来调用。可以说直接在模块中声明的是更普遍广泛性的函数，而在类中声明的方法一般是专属于一类事物特有的**

## 一、类

### ·创建自己的类

```python
#创建一个Person类
class Person:
    #定义setName方法
    def setName(self,name):
        self.name=name;

#创建person1对象
person1=Person();
#调用person1的setName方法
person1.setName("Gzh")
```

**※程序说明**：

1、每一个方法的第一个参数都是self，这是必须的。这个参数名不一定叫self，但任意一个方法必须至少指定一个self参数，如果方法中包含多个参数，第1个参数将作为self参数使用。在调用方法时，这个参数的值不需要自己传递，系统会将方法所属的对象传入这个参数。在方法内部可以利用这个参数调用对象本身的资源，如属性、方法等等。

2、通过self参数添加的name变量是Person类的属性，可以在外部访问。

3、创建对象：对象名 = 构造方法(参数列表)；

### ·方法和私有化

Python在默认情况下，所有的方法都可以被外部访问。

私有化方法：在Python类的方法名前面加双下划线（__）可以让该方法在外部不可访问。

```python
class Person:
    #定义方法method1，为私有
    def __method1(self):
        print("method2")
    #注意：此处私有并非真正私有，编译器处理遇到method1时，将其改名为_ClassName__methd，所以只要在类外调用_Person__method1即可。
```

### ·类代码块

class语句与for、if语句一样，都是代码块，这就意味着，定义类其实就是执行代码块。

### ·类的继承

类的继承：指一个类从另外一个类中获得了所有的成员。

```python
#父类
class Filter:
    def filter1(self):
        return 20
#子类
class MyFilter(Filter):  #继承，拥有Filter类的所有成员。
    def filter2(self):
        return 30
```



### ·检测继承关系

很多场景中，需要知道一个类A是否从另一个类B继承，这种校验主要是为了调用B类中的成员（方法和属性）。如果B是A的父类，那么创建A类的实例肯定会拥有B类所有的成员，关键是要判断B是否为A的父类。

可以使用issubclass函数，该函数接受两个参数，第一个是子类，第二个是父类。返回值为True或False

### ·多继承

指定多个父类

```python
class MyClass(MyParent1,MyParent2,MyParent3):
	pass #如果类中没有任何代码，需要加一条pass，否则会编译出错
```

注意：若父类中有同名的方法，那么会按照父类书写的顺序继承。写在前面的父类会覆盖写在后面的父类的同名方法。（不可重载）

### ·接口

接口就是一个规范，制定了一个类中都有哪些成员。python中，可以用hasattr函数检测一下方法是否在类中存在。

## 二、构造方法

与C++中的构造函数类似。

构造方法是创建对象的过程中被调用的第一个方法，通常用于初始化对象中需要的资源。

### ·构造方法基础知识

在方法名两侧各加两个下划线，构造方法名为init。所以完整名：______init__

```python
class Person:
    #Person类的构造方法
    def __init__(self,name="Bill"):
        print("...")
        self.name=name

#创建Person类的实例，调用构造方法        
person = Person();
```

### ·重写普通方法和构造方法



### ·使用super函数

## 三、特殊成员方法

### ·自定义序列

除了构造方法，还有可以使用的如下四个特殊方法定义自己的序列类。

1. ______len______(self)：返回序列中元素的个数。使用len函数获取序列对象的长度时会调用该方法。
2. ______getitem______(self,key)：返回与所给键对应的值。key表示键。在使用sequence[key]时会调用改方法。
3. ______setitem______(self,key,value)：设置key对应的值。
4. ______setitem______(self,key)：从序列中删除键为key的key-value对。

### ·从内建列表、字符串和字典继承

## 四、属性

### ·传统的属性

### ·property函数

### ·监控对象中所有的属性

## 五、静态方法和类方法

静态方法调用时根本不需要类的实例（即静态方法不需要self参数）。

#### ·静态方法的定义

定义静态方法需要使用@staticmethod装饰器（decorator），定义类方法需要使用@classmethod装饰器

```python
class Myclass:
    #实例方法
    def instanceMethod(self):
        pass
    #静态方法
    @staticmethod
    def staticMethod():
        pass
    #类方法
    @classmethod
    def classMethod(self):
        pass
```



## 六、迭代器

迭代器（iterator）就是循环的意思，也就是对一个集合中的元素进行循环，从而得到每一个元素。

### ·自定义可迭代的类

迭代器与列表对比。列表看似和迭代器功能类似，但是列表是将所有元素直接存入内存中，数据量小可以快速实现，数据量大则难于实现；而迭代器是读取多少元素，就将多少元素装载到内存中，不读取就不装载。

如果在一个类中定义______iter______方法，那么这个类的实例就是一个迭代器。______iter______方法需要返回一个迭代器，所以返回对象本身（self）即可。

当对象每迭代一次时，就会调用迭代器中的另外一个特殊成员方法______next__。该方法需要返回当前迭代的结果。

```python
class RightTriangle:
    def __init__(self):
        #定义一个变量n，表示当前的行数
        self.n=1
    def __next__(self):
        #通过字符串的乘法获取直角三角形每一行的字符串，每一行字符串长度为2×n-1
        result = '*'*(2*self.n-1)
        #行数加1
        self.n+=1
        return result
    #该方法必须返回一个迭代器
    def __iter__(self):
        return self
rt = RightTriangle()

#对迭代器进行迭代
for e in rt:
    #限制输出长度<20
    if len(e)>20:
        break
    print(e)
```



### ·将迭代器转换为列表

迭代器虽然比列表节省资源，但是不具备某些功能，如：分片。但是我们可以把迭代器转换为列表。

```python
class Fibonacci:
    def __init__(self):
        self.a=0
        self.b=1
    def __next__(self):
        result = self.a
        self.a , self.b = self.b ,self.a + self.b
        if result > 500:
            raise StopIteration
        return result
    def __iter__(self):
        return self
    
fibs1 = Fibonacci()
#将迭代器转换为列表
print(list(fibs1))
fibs2 = Fibonacci()
#使用for循环对迭代器进行迭代
for fib in fibs2:
    print(fib, end =' ')
```

## 七、生成器

如果说迭代器是以类为基础的单值生产器，那么生成器（generator）就是以函数为基础的单值生产器。迭代器和生成器都只能一个值一个值地生产。每迭代一次，只能得到一个值。所不同的是，迭代器需要在类中定义______iter______和______next______方法，在使用时需要创建迭代器的实例。而生成器是通过一个函数展现的，可以直接调用，所以从某种意义上来说，生产器在使用上更简洁。

### ·创建生成器

先定义一个函数，在该函数中对某个集合或迭代器进行迭代，然后使yield语句产生当前要生成的值。

```python
def myGenerator():
    numList = [1,2,3,4,5,6,7,8]
    for num in numList:
        #yield语句会冻结当前函数，并提交当前要生成的值(本例是num)
        yield num
    #对生成器进行迭代
    for num in myGenerator():
        print(num, end = ' ')
```

### ·递归生成器

