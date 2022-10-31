# Ceres与g2o实践

## 5.1 Ceres实践

#### 5.1.1 Ceres简介

​	Ceres库面向通用的最小二乘问题的求解，作为用户，我们需要做的是**定义优化问题，然后设置一些选项，进入Ceres求解即可**。

​	Ceres求解的最小二乘问题一般形式如下（带边界的核函数最小二乘）：
$$
min_x \frac{1}{2}\sum_i{g_i}(||f_i(x_{i_1},...,x_{i_n})||^2)\\
s.t.  \ \ \ l_j <=x_j<=u_j
$$
其中，gi为核函数。fi为代价函数，SLAM中可以理解为误差项。lj和uj为xj的上下限。

#### 5.1.2 安装Ceres

#### 5.1.3 使用Ceres拟合曲线

​	我们用Ceres进行曲线拟合，假设有一条满足一下方程的曲线：
$$
y=e^{ax^2+bx+c}+w
$$
其中，w为高斯噪声。那么我们为了求解参数a，b，c，列写最小二乘问题：
$$
min_{a,b,c}\frac{1}{2}\sum_{i=1}^N||y_i-e^{ax^2+bx+c}||^2
$$
拟合思路，先根据真实abc生成真实函数值，再加上高斯噪声，再拟合对比结果。

C++代码：

```C++
#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

// 代价函数的计算模型
struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
    // 残差的计算
    template <typename T>
    bool operator() (
        const T* const abc,     // 模型参数，有3维
        T* residual ) const     // 残差
    {
        residual[0] = T ( _y ) - ceres::exp ( abc[0]*T ( _x ) *T ( _x ) + abc[1]*T ( _x ) + abc[2] ); // y-exp(ax^2+bx+c)
        return true;
    }
    const double _x, _y;    // x,y数据
};

int main ( int argc, char** argv )
{
    double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0,0,0};            // abc参数的估计值

    vector<double> x_data, y_data;      // 数据

    cout<<"generating data: "<<endl;
    for ( int i=0; i<N; i++ )
    {
        double x = i/100.0;
        x_data.push_back ( x );
        y_data.push_back (
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
        );
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }

    // 构建最小二乘问题
    ceres::Problem problem;
    for ( int i=0; i<N; i++ )
    {
        problem.AddResidualBlock (     // 向问题中添加误差项
        // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3> ( 
                new CURVE_FITTING_COST ( x_data[i], y_data[i] )
            ),
            nullptr,            // 核函数，这里不使用，为空
            abc                 // 待估计参数
        );
    }

    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve ( options, &problem, &summary );  // 开始优化
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;

    // 输出结果
    cout<<summary.BriefReport() <<endl;
    cout<<"estimated a,b,c = ";
    for ( auto a:abc ) cout<<a<<" ";
    cout<<endl;

    return 0;
}

```

CMakeList文件：

```cmake
cmake_minimum_required( VERSION 2.8 )
project( ceres_curve_fitting )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )
include_directories( ${CERES_INCLUDE_DIRS} )

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )

add_executable( curve_fitting main.cpp )
# 与Ceres和OpenCV链接
target_link_libraries( curve_fitting ${CERES_LIBRARIES} ${OpenCV_LIBS} )
```

## 5.2 实践：g2o

#### 5.2.1 图优化理论简介

​	之前的非线性最小二乘，它们由很多个误差项之和组成，但我们不清楚它们的**关联**。比如：**某一个优化变量xj存在于多少个误差项里，对它的优化有意义吗？进一步我们希望能够直观地看到该优化问题长什么样。于是，就说到了图优化。**

​	图优化，是把优化问题表现成**图(Graph)**的一种方式。这里的图是图论意义上的图，一个图由若干个**顶点(Vertex)**，以及连接着这些节点的**边(Edge)**组成。进而，用**顶点**表示**优化变量**，用**边**表示**误差项**。于是，这任意一个上述形式的非线性最小二乘问题，我们可以构建与之对应的一个**图**。如下图所示。

![](/home/gzh/图片/2021-11-02 19-38-44 的屏幕截图.png)

​	上图就是一个简单的图优化例子。我们用三角形表示相机位姿节点，用圆形表示路标点，它们构成了图优化的顶点；同时，蓝色线表示相机的运动模型，红色虚线表示观测模型，它们构成了图优化的边。此时，整个问题的数学模式仍然如下(两个关键帧(x1,x2)之间会存在误差，观测模型中也存在误差(x1,p1))
$$
J(x)=\sum_k{e_{v,k}^TR_k^{-1}e_{v,k}}+\sum_k{\sum_j{e_{y,k,j}^TQ_{k,j}^{-1}e_{y,k,j}}}
$$
但现在我们可以直观地看到问题的结构，**如果我们希望，也可以去掉孤立顶点或优先优化边数较多（度数较大）的顶点**。基本的图优化，是用图模型来表达一个非线性最小二乘的优化问题。**而我们可以利用图模型的某些性质，做更好的优化**。

#### 5.2.2 g2o的编译与安装

#### 5.2.3 g2o拟合曲线

​	为了使用g2o，首先要做的是将曲线拟合问题抽象成图优化。**节点为优化变量，边为误差项**。优化变量为a，b，c，误差项为很多个xi yi带来的，是很多条边。如下图：

![](/home/gzh/图片/2021-11-03 14-24-16 的屏幕截图.png)

那么，作为g2o的用户，我们要做的事情如下：

1. 定义顶点和边的类型
2. 构建图
3. 选择优化算法
4. 调用g2o进行优化，返回结果。

而看整体的g2o图优化器，我们要维护的是一个optimizer，这个优化器需要求解线性方程以及一个迭代策略：如下

1. 选择一个线性方程求解器：从PCG，CSparse，Choldmod中选择。
2. 选择一个BlockSolver
3. 选择一个迭代策略，从GN，LM，Doglog中选择

根据上面几个步骤，详细分析代码：

```c++
/*
定义顶点和边的类型：g2o提供两种顶点类型：
1、李代数位姿：class  VertexSE3Expmap : public BaseVertex<6, SE3Quat> 模板参数可以自定义：
第一个参数int d = 6 ，代表顶点参数维度，为6；第二个参数为顶点类型，SE3Quat 平移+旋转各三维
2、空间点位置：class VertexSBAPointXYZ : public BaseVertex<3, Vector3d> 参数同理也可以自定义，此处使用Eigen::Vector3d，三维度
我们本例子使用第二种顶点类型，因此继承public BaseVertex基类
虚函数：我们要重新定义某些虚函数(它们是基类中的纯虚函数)

*/
// 曲线模型的顶点，模板参数：优化变量维度和数据类型
class CurveFittingVertex: public g2o::BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual void setToOriginImpl() // 重置
    {
        _estimate << 0,0,0;//设置初始值
    }
    
    virtual void oplusImpl( const double* update ) // 更新
    {
        _estimate += Eigen::Vector3d(update);//确定更新方式
        //向量空间我们可以使用“+”法，若为李代数空间则为乘法
        //或其他情况，我们都要定义更新的方式。
    }
    // 存盘和读盘：留空
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
};
/*
g2o提供3种边的形式：
1、Point-Pose 二元边(XYZ-SE3 edge) 即要优化相机位姿，也要优化特征点位置
class  EdgeSE3ProjectXYZ: public  BaseBinaryEdge<2, Vector2d, VertexSBAPointXYZ, VertexSE3Expmap>
2、Pose 一元边(SE3) 仅优化相机位姿
class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>
3、Pose-Pose 二元边(SE3-SE3) 优化两个(相邻)关键帧的位姿
class G2O_TYPES_SBA_API EdgeSE3Expmap : public BaseBinaryEdge<6, SE3Quat, VertexSE3Expmap, VertexSE3Expmap>
本例选择Pose 一元边，因为从上述分析可知，边都是一元边。

我们要编写 函数computeError() 来计算误差。
*/

// 误差模型 模板参数：观测值维度，类型，连接顶点类型
class CurveFittingEdge: public g2o::BaseUnaryEdge<1,double,CurveFittingVertex>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CurveFittingEdge( double x ): BaseUnaryEdge(), _x(x) {}
    // 计算曲线模型误差
    void computeError()
    {
        //强制类型转换 static_cast<new type> (expression) 把expression强制转换为new_type
        const CurveFittingVertex* v = static_cast<const CurveFittingVertex*> (_vertices[0]);
        //获得abc估计值
        const Eigen::Vector3d abc = v->estimate();
        //误差值计算
        _error(0,0) = _measurement - std::exp( abc(0,0)*_x*_x + abc(1,0)*_x + abc(2,0) ) ;
    }
    virtual bool read( istream& in ) {}
    virtual bool write( ostream& out ) const {}
public:
    double _x;  // x 值， y 值为 _measurement
};

```

main部分：

```c++
//本部分生成真实数据点后增加高斯噪声，简单易懂不解释
	double a=1.0, b=2.0, c=1.0;         // 真实参数值
    int N=100;                          // 数据点
    double w_sigma=1.0;                 // 噪声Sigma值
    cv::RNG rng;                        // OpenCV随机数产生器
    double abc[3] = {0,0,0};            // abc参数的估计值

    vector<double> x_data, y_data;      // 数据
    
    cout<<"generating data: "<<endl;
    for ( int i=0; i<N; i++ )
    {
        double x = i/100.0;
        x_data.push_back ( x );
        y_data.push_back (
            exp ( a*x*x + b*x + c ) + rng.gaussian ( w_sigma )
        );
        cout<<x_data[i]<<" "<<y_data[i]<<endl;
    }
```

```c++
//步骤二：构建图优化
//这个构建过程比较固定，可以参考这一部分代码
	typedef g2o::BlockSolver< g2o::BlockSolverTraits<3,1> > Block;// 每个误差项优化变量维度为3，误差值维度为1
	//线性方程求解器
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverDense<Block::PoseMatrixType>() );
    std::unique_ptr<Block> solver_ptr (new Block( std::move(linearSolver) ));
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );

    g2o::SparseOptimizer optimizer; //图模型(优化器)
    optimizer.setAlgorithm( solver );   //设置求解器
    optimizer.setVerbose( true ); //打开调试输出
```

```c++
//步骤三：
    // 往图中增加顶点
    CurveFittingVertex* v = new CurveFittingVertex();//声明一个顶点对象
    v->setEstimate( Eigen::Vector3d(0,0,0) );//设置初始值
    v->setId(0);//设置id编号
    optimizer.addVertex( v );//添加顶点v
    
    // 往图中增加边
    for ( int i=0; i<N; i++ )
    {
        CurveFittingEdge* edge = new CurveFittingEdge( x_data[i] );//声明一个边对象
        edge->setId(i);//设置边id
        edge->setVertex( 0, v );                // 设置连接的顶点
        //若为2元边，则第二个顶点例子：edge->setVertex(1,v1);
        edge->setMeasurement( y_data[i] );      // 观测数值
        edge->setInformation( Eigen::Matrix<double,1,1>::Identity()*1/(w_sigma*w_sigma) ); // 信息矩阵：协方差矩阵之逆
        optimizer.addEdge( edge );//图中加入边
    }
```

```c++
//步骤四：开始优化
    cout<<"start optimization"<<endl;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();//记录开始时间
    optimizer.initializeOptimization();//初始化
    optimizer.optimize(100);//迭代次数
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();//记录结束时间
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );//计算during time
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
    
    // 输出优化值
    Eigen::Vector3d abc_estimate = v->estimate();
    cout<<"estimated model: "<<abc_estimate.transpose()<<endl;
```

