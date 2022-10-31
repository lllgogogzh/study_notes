# CMakelist的写法

## 一、整体介绍

```cmake
#对应第一部分
cmake_minimum_required(VERSION 3.0.2)
project(myslam)

#对应第二部分，第三方库的使用方式
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)#ros的

#未讲解
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES myslam
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

#对应第二部分，第三方库的使用方式(自定义库的使用方式)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}#ros的
)

#对应第三部分，添加可执行程序
add_executable(Test src/Test.cpp)
#对应第二部分，链接库
target_link_libraries(Test ${catkin_LIBRARIES})#链接ros的库
```

### 1、cmake_minimun_required

我们直接用代码展示

```cmake
#指定cmake的最小版本
#指定工程：功能包名myslam
cmake_minimum_required(VERSION 3.0.2)
project(myslam)
```

这部分是必有的

### 2、find_package()

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs  #new
)
#附：这里的catkin REQUIRED COMPONENTS 个人认为是ros库中的内容，因此若想要使用一些消息类型，在下面添加消息类型的即可。例如上面，添加了sensor_msgs消息类型。
```

**find_package()的基本原理**

​	当编译一个需要使用第三方库的软件时，我们需要知道以下内容：

1. 去哪找到头文件(xxx.h)
2. 去哪找到库文件(.so/.dll/.lib/.dylib......)
3. 需要链接的库文件的名字

​	举个例子：比如说，我们需要一个第三方库curl，那么我们的CMakeLists.txt文件需要指定**头文件目录**和**库文件目录**：

```cmake
include_directiories(/usr/include/curl)
target_link_libraries(myslam path/curl.so)
```

​	**若借助于cmake提供的find_package，使用cmake Modules目录下的FindCURL.cmake，相应的CMakeList.txt文件如下**

```cmake
find_package(CURL REQUIRED)
include_directories(${CURL_INCLUDE_DIR})
target_link_libraries(curltest ${CURL_LIBRARIES})
```

附：在include_directories中，如下写，则可以识别到本功能包下自己定义的include文件

```cmake
include_directories(include)
```

### 3、add_executable

添加可执行文件，将.cpp文件编译为可执行文件，如果需要链接库，那么还需要将可执行程序进行target_link_libraries()

```cmake
add_executable(Test src/Test.cpp)
target_link_libraries(Test ${catkin_LIBRARIES})
#target_link_libraries在第二节中讲解了，是与使用第三方软件库相关的，如果不需要第三方软件库，那么就不需要这个了
```

### 4、add_library

将指定的源文件(.cpp .c .cc等等)生成链接库文件，后添加到工程中。一般用于自己创建的库或者第三方库，用法示例：

```cmake
add_library(<name> [STATIC|SHARED|MODULE]
[EXCLUDE_FROM_ALL]
[source1]
[source2]
...
)
```

```cmake
###加OpenCV
find_package(OpenCV 3.1 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
###完

add_library(myslamlib
  src/FeatureExtraction.cpp
  src/Frame.cpp
  src/Test.cpp
  src/VOFrontEnd.cpp
  )
#附：若自己写的库中包含官方的一些库(或其他库)，需要在这些之前进行库的搜索以及路径搜索。
#例如：以上自己编写的源文件中，需要用到OpenCV库，那么我们需要在这之前搞好OpenCV，如上所示
#之后，还需要进行库链接，如下，把需要的库链接到自己写的库上
target_link_libraries(myslamlib ${catkin_LIBRARIES} ${OpenCV_LIBS})
```

### 5、target_link_libraries

将目标文件与库文件进行链接，用法上面代码中包括了一些，比较易懂。

### 6、add_subdirectory

功能概述：**添加一个子目录并构建该子目录**

```cmake
add_subdirectory(source_dir [binary_dir] [EXCLUDE_FROM_ALL])
```

- source_dir是必填参数，该参数指定一个子目录，子目录下也应包含CMakelists.txt文件和代码文件。
- binary_dir可选参数，指定一个目录，用于存放输出文件。若没有指定该参数，则默认的输出目录使用source_dir

举个例子：这是视觉slam十四讲中的Project 0.4中的cmakelists文件

```cmake
cmake_minimum_required( VERSION 2.8 )
project ( myslam )

set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Release" )
#set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3" )
set( CMAKE_CXX_FLAGS "-std=c++14 -O2 ${SSE_FLAGS} -msse4" )

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )

############### dependencies ######################
# Eigen
include_directories( "/usr/include/eigen3" )
# OpenCV
find_package( OpenCV 3.1 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
# Sophus 
find_package( Sophus REQUIRED )
include_directories( ${Sophus_INCLUDE_DIRS} )
# G2O
find_package( G2O REQUIRED )
include_directories( ${G2O_INCLUDE_DIRS} )

set( THIRD_PARTY_LIBS 
    ${OpenCV_LIBS}
    ${Sophus_LIBRARIES}
    g2o_core g2o_stuff g2o_types_sba
)
############### dependencies ######################
include_directories( ${PROJECT_SOURCE_DIR}/include )
add_subdirectory( src )
add_subdirectory( test )
```

主要看最后几行，指定了两个子目录，我们看test中的

```cmake
add_executable( run_vo run_vo.cpp )
target_link_libraries( run_vo myslam )
```

这里，myslam在src目录下的cmakelist文件中已经定义。

### 7、list

**命令格式：**

```cmake
list(subcommand <list> [args...])
```

subcommand为具体列表操作子命令，如：读取、查找、修改、排序等等。< list >为待操作的列表变量，[args ...]为对列表变量操作需要使用的参数表，不同的子命令对应的参数也不一致。

我们举个例子，应用在调用g2o库的例子。

```cmake
#我们想使用第三方库一般步骤为两步
#1、在哪找头文件 include_directories
#2、在哪找库文件 target_link_libraries
#我们一般使用find_package来简化上两部的具体路径，
#find_package利用.camke文件来提供包的头文件与库文件目录(dir)
#但是我们在调用g2o时候，直接这三行命令会报错：提示找不到g2o相关的.cmake文件
#我们就需要自己来在CMAKE_MODULE后面加上这个g2o的.cmake文件

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
#APPEND子命令用于将元素追加到列表
##list(APPEND <list> [<element> ...])
#把${PROJECT_SOURCE_DIR}/cmake_modules这个目录下的.cmake文件添加到CMAKE_MODULE_PATH列表中

#之后我们再三步即可
find_package( G2O REQUIRED )
include_directories(${G2O_INCLUDE_DIRS})
target_link_libraries(xxx g2o_core g2o_sutff g2o_types_sba g2o_csparse_extension)
```

具体的FindG2O.cmake文件这里就不展示了，可以自行详细阅读。

## 二、各种库的调用示例

### 1、opencv

```cmake
#OpenCV
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS)
target_link_libraries(xxx ${OpenCV_LIBS})
```

### 2、Eigen

```cmake
include_directories( "/usr/include/eigen3" )
```

比较特殊，之后讲解

### 3、Sophus

```cmake
#Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
target_link_libraries(xxx ${Sophus_LIBRARIES})
```

### 4、g2o

### 5、Ceres
