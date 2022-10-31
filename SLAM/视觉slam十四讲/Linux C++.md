# Linux C++

## 1.1 Hello SLAM程序

#### 1.1.1 简单介绍，使用g++编译

​	在Linux中，程序是一个具有执行权限的文件，其可以是一个脚本，也可以是一个二进制文件。对于一个可执行程序，只要它有可执行权限，那么当我们在终端输入程序名时，它就会运行。**在C++编程时，我们用编译器，把一个文本文件编译成可执行程序。**

```c++
#include <iostream>
using namespace std; 

int main( int argc, char** argv )
{
    cout<<"Hello SLAM!"<<endl;
    return 0;
}
```

​	我们用编译器g++(一个C++编译器)把它编译成一个可执行文件：

```shell
g++ helloSLAM.cpp
```

​	输入完这条命令，我们发现文件夹中出现了一个 a.out的程序，我们输入下列命令，可以得到如下结果：

```shell
./a.out
Hello SLAM!
```

​	这里a.out是默认的输出，我们也可以指定输出的名字：

```shell
g++ helloSLAM.cpp -o [文件名]
eg:
g++ helloSLAM.cpp -o SLAM
```

#### 1.1.2 使用cmake

​	理论上来说，任意一个C++程序都可以用g++来编译，但当程序规模较大时，一个工程可能有许多个文件夹和里边的源文件，这时输入编译的命令会十分复杂。**通常一个小型c++项目含有十几个类，各类间还存在着复杂的依赖关系。其中一部分要编译成可执行文件，另一部分编译成库文件。**

​	我们用cmake来更加方便地编译。

​	**在一个cmake工程中，我们会用cmake命令生成一个makefile文件。然后，用make命令，根据这个makefile文件的内容，编译整个工程。**

​	新建CMakeList.txt文件，输入：

```cmake
# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( HelloSLAM )

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( helloSLAM helloSLAM.cpp )

# 添加一个库
add_library( hello libHelloSLAM.cpp )
# 共享库
add_library( hello_shared SHARED libHelloSLAM.cpp )

add_executable( useHello useHello.cpp )
# 将库文件链接到可执行程序上
target_link_libraries( useHello hello_shared )
```

​	我们在终端中输入：

```shell
cmake .
```

​	在终端中会输出：

```shell
gzh@gzh-G3-3590:~/slamstudy/slambook/ch2$ cmake .
-- The C compiler identification is GNU 5.4.0
-- The CXX compiler identification is GNU 5.4.0
-- Check for working C compiler: /usr/bin/cc
-- Check for working C compiler: /usr/bin/cc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Detecting C compile features
-- Detecting C compile features - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Detecting CXX compile features
-- Detecting CXX compile features - done
-- Configuring done
-- Generating done
-- Build files have been written to: /home/gzh/slamstudy/slambook/ch2
```

之后，会在文件夹中生成一个MakeFile文件夹，这是自动生成的。现在，我们用make命令对工程进行编译：

```shell
gzh@gzh-G3-3590:~/slamstudy/slambook/ch2$ make
Scanning dependencies of target hello
[ 12%] Building CXX object CMakeFiles/hello.dir/libHelloSLAM.cpp.o
[ 25%] Linking CXX static library libhello.a
[ 25%] Built target hello
Scanning dependencies of target hello_shared
[ 37%] Building CXX object CMakeFiles/hello_shared.dir/libHelloSLAM.cpp.o
[ 50%] Linking CXX shared library libhello_shared.so
[ 50%] Built target hello_shared
Scanning dependencies of target helloSLAM
[ 62%] Building CXX object CMakeFiles/helloSLAM.dir/helloSLAM.cpp.o
[ 75%] Linking CXX executable helloSLAM
[ 75%] Built target helloSLAM
Scanning dependencies of target useHello
[ 87%] Building CXX object CMakeFiles/useHello.dir/useHello.cpp.o
[100%] Linking CXX executable useHello
[100%] Built target useHello
```

我们获得了一个可执行程序helloSLAM，我们执行即可。

```shell
gzh@gzh-G3-3590:~/slamstudy/slambook/ch2$ ./helloSLAM
Hello SLAM!
```

​	**这么做的好处就是：我们不需要输入一大串g++命令，只需要维护CMakeList文档即可。**

#### 1.1.3 使用库

​	**在一个C++工程中，并不是所有代码都会编译成可执行文件。只有带main函数的文件才会生成可执行程序。而另一些代码，我们只想把它们打包成一个东西，供其他程序调用。这个东西叫做库。**

​	一个库往往是许多算法、程序的集合，例如OpenCV视觉算法库，Eigen矩阵计算库。我们要学会如何使用cmake 生成库，并且使用库中的函数。现在我们编写一个libHelloSLAM.cpp文件：

```c++
//这是一个库文件
#include <iostream>
using namespace std;

void printHello()
{
    cout<<"Hello SLAM"<<endl;
}
```

​	这个库提供了一个printHello函数。其中没有main函数，这意味着这个库中没有可执行文件。我们在CMakeLists.txt里加一句：

```cmake
add_library(hello libHelloSLAM.cpp)
```

​	**这条命令告诉cmake，我们想把这个文件编译成一个叫做“hello”的库。然后，和上面一样，使用cmake编译整个工程：**

```shell
mkdir build
cd build
cmake ..
make
```

​	这里，我们把编译生成的文件都放入了build文件夹中，不乱。这时，在build文件夹中生成了一个libhello.a文件，这就是我们得到的库。

​	**在Linux中，库文件分成静态库和共享库两种。**静态库以.a作为后缀名，共享库以.so结尾。所有库都是一些函数打包后的集合，**差别在于静态库每次被调用都会生成一个副本，而共享库则只有一个副本。**

​	我们若想生成共享库：在CMakeList.txt文件中添加：

```cmake
add_library(hello_shared SHARED libHelloSLAM.cpp)
```

​	我们会得到一个 libhello_shared.so文件。

​	库文件是一个压缩包，里面有编译好的二进制函数。不过，仅有.a或.so库文件的话，我们并不知道它里面的函数到底是什么，调用的形式如何。为了让别人或者自己使用这个库，**我们需要提供一个头文件。**对于库的使用者，**只要拿到了头文件和库文件，就可以调用这个库了。**我们编写libhello的头文件。

```c++
#ifndef LIBHELLOSLAM_H_
#define LIBHELLOSLAM_H_
// 上面的宏定义是为了防止重复引用这个头文件而引起的重定义错误

void printHello();

#endif
```

​	之后，我们在CMakeList.txt中添加一个可执行程序的生成命令：

```cmake
add_executable( useHello useHello.cpp )
# 将库文件链接到可执行程序上
target_link_libraries( useHello hello_shared )
```

​	通过这两句话，useHello程序就能顺利使用hello_shared库中的代码了。

#### 1.1.4 cmake总结

​	1、首先，程序代码由头文件和源文件组成。

​	2、带有main函数的源文件编译成可执行程序，其他的编译成库文件。

​	3、如果可执行程序想要调用库文件中的函数，它需要参考该库提供的头文件，以明确调用的格       		  式。同时，要把可执行程序链接到库文件上。