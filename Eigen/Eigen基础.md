# Eigen基础

## 一、初始化

### 1、矩阵初始化

### 2、向量初始化

#### i. VectorXd 初始化

```c++
int main()
{
    Eigen::VectorXd R;//当然也可以直接 R(3)，这么做是为了测试根据a的值赋值长度
    int a=3;
    R=Eigen::VectorXd(a);
    R(0)=1;
    R(1)=2;
    R(2)=3;
    cout<<R<<endl; 
    return 0;
}
```



### 3、其他