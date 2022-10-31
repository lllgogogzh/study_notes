# g2o实践BA

## 1、用BA实现ICP

​	ICP我们知道其原理，优化如下式子：
$$
e(R,t)=\sum||p_1-(Rp_2+t)||^2
$$
我们先不去优化空间坐标点的坐标，就去优化R和t。因此，在g2o图优化中，我们把其看作一个顶点，而所有边都是一元边。在程序中，我们可以自己定义一个一元边，也可以用g2o带的一元边类：class  EdgeSE3ProjectXYZOnlyPose: public  BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>

​	这个程序中，我们用自己定义的一元边

```c++
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap>//误差维度是3维，类型为Eigen::Vector3d，连接的节点类型为g2o::VertexSE3Expmap(相机位姿节点)
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeProjectXYZRGBDPoseOnly( const Eigen::Vector3d& point ) : _point(point) {}

    virtual void computeError()
    {
        const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
        // measurement is p, point is p'
        _error = _measurement - pose->estimate().map( _point );
    }

    virtual void linearizeOplus()
    {
        g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = -z;
        _jacobianOplusXi(0,2) = y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;

        _jacobianOplusXi(1,0) = z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = -x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;

        _jacobianOplusXi(2,0) = -y;
        _jacobianOplusXi(2,1) = x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }

    bool read ( istream& in ) {}
    bool write ( ostream& out ) const {}
protected:
    Eigen::Vector3d _point;
};
```

之后，我们开始求解

```c++
void EstimationOptimization(vector<cv::Point3f> pts1,vector<cv::Point3f> pts2,cv::Mat &R,cv::Mat &t)
{
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,3>> Block;//优化变量6维，误差3维
    std::unique_ptr<Block::LinearSolverType> linearSolver ( new g2o::LinearSolverEigen<Block::PoseMatrixType>() );
    std::unique_ptr<Block> solver_ptr (new Block( std::move(linearSolver) ));
    g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) );
    g2o::SparseOptimizer optimizer;   //优化器
    optimizer.setAlgorithm( solver );   
    optimizer.setVerbose( true ); 

    //定义节点，设置初始值，Id，并添加进优化器
    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(),Eigen::Vector3d(0,0,0)));
    pose->setId(0);
    optimizer.addVertex(pose);
	//定义边，设置与其连接的节点，Id，设置观测值以及信息矩阵，最后添加进优化器
    for(int i=0;i<pts1.size();i++)
    {
        EdgeProjectXYZRGBDPoseOnly *pPoint = new EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d(pts2[i].x,pts2[i].y,pts2[i].z));
        pPoint->setId(i);
        pPoint->setVertex(0,pose);
        pPoint->setMeasurement(Eigen::Vector3d(pts1[i].x,pts1[i].y,pts1[i].z));
        pPoint->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(pPoint);
    }

    optimizer.initializeOptimization();//初始化
    optimizer.optimize(100);//迭代次数
    cout<<"T="<<endl<<Eigen::Isometry3d( pose->estimate() ).matrix()<<endl;//得到最后的结果就是这个pose->estimate()，这pose是优化中设置的节点
}
```

