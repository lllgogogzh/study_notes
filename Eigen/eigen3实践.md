# Eigen3实践

## 2.1 Eigen3 基础实践

​	本节将介绍如何使用Eigen来表示矩阵、向量，随后引申至旋转矩阵与变换矩阵的计算。

​	Eigen是一个C++开源线性代数库。它提供了快速的有关矩阵的线性代数运算，还包括解方程等功能。

​	在使用Eigen时，只需引入Eigen的头文件即可，不需要连接它的库文件（没有）。

#### 2.1.1 Eigen3 C++代码部分

```c++
#include <iostream>
using namespace std;
#include <ctime>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 50

/****************************
* 本程序演示了 Eigen 基本类型的使用
****************************/

int main( int argc, char** argv )
{
    // Eigen 中所有向量和矩阵都是Eigen::Matrix，它是一个模板类。它的前三个参数为：数据类型，行，列
    // 声明一个2*3的float矩阵
    Eigen::Matrix<float, 2, 3> matrix_23;

    // 同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是Eigen::Matrix
    // 例如 Vector3d 实质上是 Eigen::Matrix<double, 3, 1>，即三维向量
    Eigen::Vector3d v_3d;
	// 这是一样的
    Eigen::Matrix<float,3,1> vd_3d;

    // Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); //初始化为零
    // 如果不确定矩阵大小，可以使用动态大小的矩阵
    Eigen::Matrix< double, Eigen::Dynamic, Eigen::Dynamic > matrix_dynamic;
    // 更简单的
    Eigen::MatrixXd matrix_x;
    // 这种类型还有很多，我们不一一列举

    // 下面是对Eigen阵的操作
    // 输入数据（初始化）
    matrix_23 << 1, 2, 3, 4, 5, 6;
    // 输出
    cout << matrix_23 << endl;

    // 用()访问矩阵中的元素
    for (int i=0; i<2; i++) {
        for (int j=0; j<3; j++)
            cout<<matrix_23(i,j)<<"\t";
        cout<<endl;
    }

    // 矩阵和向量相乘（实际上仍是矩阵和矩阵）
    v_3d << 3, 2, 1;
    vd_3d << 4,5,6;
    // 但是在Eigen里你不能混合两种不同类型的矩阵，像这样是错的
    // Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
    // 应该显式转换
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;

    Eigen::Matrix<float, 2, 1> result2 = matrix_23 * vd_3d;
    cout << result2 << endl;

    // 同样你不能搞错矩阵的维度
    // 试着取消下面的注释，看看Eigen会报什么错
    // Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix_23.cast<double>() * v_3d;

    // 一些矩阵运算
    // 四则运算就不演示了，直接用+-*/即可。
    matrix_33 = Eigen::Matrix3d::Random();      // 随机数矩阵
    cout << matrix_33 << endl << endl;

    cout << matrix_33.transpose() << endl;      // 转置
    cout << matrix_33.sum() << endl;            // 各元素和
    cout << matrix_33.trace() << endl;          // 迹
    cout << 10*matrix_33 << endl;               // 数乘
    cout << matrix_33.inverse() << endl;        // 逆
    cout << matrix_33.determinant() << endl;    // 行列式

    // 特征值
    // 实对称矩阵可以保证对角化成功
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver ( matrix_33.transpose()*matrix_33 );
    cout << "Eigen values = \n" << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = \n" << eigen_solver.eigenvectors() << endl;

    // 解方程
    // 我们求解 matrix_NN * x = v_Nd 这个方程
    // N的大小在前边的宏里定义，它由随机数生成
    // 直接求逆自然是最直接的，但是求逆运算量大

    Eigen::Matrix< double, MATRIX_SIZE, MATRIX_SIZE > matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random( MATRIX_SIZE, MATRIX_SIZE );
    Eigen::Matrix< double, MATRIX_SIZE,  1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random( MATRIX_SIZE,1 );

    clock_t time_stt = clock(); // 计时
    // 直接求逆
    Eigen::Matrix<double,MATRIX_SIZE,1> x = matrix_NN.inverse()*v_Nd;
    cout <<"time use in normal inverse is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms"<< endl;
    
	// 通常用矩阵分解来求，例如QR分解，速度会快很多
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout <<"time use in Qr decomposition is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;

    return 0;
}
```

#### 2.1.2 CMakeList.txt

​	现在来看一下CMakeList部分。首先，需要编译，在CMakeList文件中指定Eigen的头文件目录：

```cmake
include_directories("/usr/include/eigen3")
```

整体代码：

```cmake
cmake_minimum_required( VERSION 2.8 )
project( useEigen )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-O3" )

# 添加Eigen头文件
include_directories( "/usr/include/eigen3" )

# in osx and brew install
# include_directories( /usr/local/Cellar/eigen/3.3.3/include/eigen3 )

add_executable( eigenMatrix eigenMatrix.cpp )
```

#### 2.1.3 eigen使用注意事项

1. Eigen提供的矩阵和MATLAB很相似，几乎所有数据都当作矩阵来处理。但是，为了实现更好的效率，在Eigen中你需要指定矩阵的大小和类型(同matlab)。
2. Eigen内部矩阵实现比较复杂，一般矩阵元素数据类型为float、double这些内置数据类型。
3. Eigen矩阵不支持自动类型提升（C++中我们可以把float和double做运算，**编译器会自动把数据类型转换为最合适的那种**）。但在Eigen中，**必须显式地对矩阵类型进行转换。**（若不如此做，可能报错：YOU MIXED DIFFERENT NUMERIC TYPES）。
4. 同理，计算过程中必须保证矩阵维数的正确性。

## 2.2 Eigen 几何模块

​	实践演练各种旋转表达方式：四元数、欧拉角和旋转矩阵。

### 2.3 Eigen与cv

```c++
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

void cv::eigen2cv(const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src, Mat& dst)
void cv::cv2eigen(const Mat& src, Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& dst)
```

详见：(include用上述代码的方式，先Eigen后cv)

https://blog.csdn.net/ThorKing01/article/details/97628704
