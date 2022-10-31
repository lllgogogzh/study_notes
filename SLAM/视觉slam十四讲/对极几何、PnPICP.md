# 对极几何、PnP/ICP

​	提前流程：对两帧(地图点与当前帧)之间进行特征提取、特征点匹配后，得到匹配信息，进行相机位姿的估计。

## 一、2d-2d对极几何实践

### 1、理论基础

​	输入特征点匹配对，可以得到两帧之间的相机位姿，这过程不需要深度信息，因此叫2d-2d。理论讲解在pdf文档中。现在进行实践。

​	**大致流程：利用匹配好的特征点来计算E(本质矩阵)，F(基础矩阵)和H(单应矩阵)，进而分解E得到R和t**

### 2、代码详解

​	单独讲解2d-2d对极几何部分：

```c++
void pose_estimation_2d2d ( std::vector<KeyPoint> keypoints_1,
                            std::vector<KeyPoint> keypoints_2,
                            std::vector< DMatch > matches,
                            Mat& R, Mat& t )
{
    // 相机内参,TUM Freiburg2
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵F
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, CV_FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵E
    Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵H
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
    
}
```

运行结果：

```shell
-- Max dist : 95.000000 
-- Min dist : 4.000000 
一共找到了79组匹配点
fundamental_matrix is 
[4.844484382466111e-06, 0.0001222601840188731, -0.01786737827487386;
 -0.0001174326832719333, 2.122888800459598e-05, -0.01775877156212593;
 0.01799658210895528, 0.008143605989020664, 1]
essential_matrix is 
[-0.02036185505234771, -0.4007110038118444, -0.033240742498241;
 0.3939270778216368, -0.03506401846698084, 0.5857110303721015;
 -0.006788487241438231, -0.5815434272915687, -0.01438258684486259]
homography_matrix is 
[0.9497129583105288, -0.143556453147626, 31.20121878625771;
 0.04154536627445031, 0.9715568969832015, 5.306887618807696;
 -2.81813676978796e-05, 4.353702039810921e-05, 1]
R is 
[0.9985961798781877, -0.05169917220143662, 0.01152671359827862;
 0.05139607508976053, 0.9983603445075083, 0.02520051547522452;
 -0.01281065954813537, -0.02457271064688494, 0.9996159607036126]
t is 
[-0.8220841067933339;
 -0.0326974270640541;
 0.5684264241053518]
t^R=
[-0.02879601157010514, -0.5666909361828475, -0.0470095088643642;
 0.5570970160413605, -0.04958801046730488, 0.8283204827837457;
 -0.009600370724838811, -0.8224266019846685, -0.02034004937801358]
epipolar constraint = [0.002528128704106514]
epipolar constraint = [-0.001663727901710814]
epipolar constraint = [-0.0008009088410885212]
..........
```

计算出了F，E，H，与最后的R，t。理论上来说：
$$
\hat t R=E
$$
根据对比，还是有一定的误差。

​	需要注意的是，我们得到的R和t组成的变换矩阵，是从第一个图到第二个图的坐标变换矩阵：
$$
x_2=T_{21}x_1
$$

## 二、三角测量实践

### 1、基本理论

​	根据之前的对极几何求解的相机位姿，通过三角化求出特征点的空间位置。使用OpenCV提供的triangulation函数进行三角化。

### 2、代码

```c++
void triangulation ( 
    const vector< KeyPoint >& keypoint_1, 
    const vector< KeyPoint >& keypoint_2, 
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t, 
    vector< Point3d >& points )
{
    Mat T1 = (Mat_<float> (3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );		//以第一帧为参考，T2为从1到2的变化。就是谁乘1之后等于2，就是x2=T21x1
    
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );//内参
    vector<Point2f> pts_1, pts_2;
    for ( DMatch m:matches )
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
        pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
    }
    
    Mat pts_4d;
    //T1：第一帧相机在世界坐标系下的位姿
    //T2：第二帧相机在世界坐标系下的位姿
    //pts_1，pts_2：特征点在相机坐标系下的归一化坐标
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );//三角化
    
    // 转换成非齐次坐标
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        Point3d p (
            x.at<float>(0,0), 
            x.at<float>(1,0), 
            x.at<float>(2,0) 
        );
        points.push_back( p );
    }
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
    );
}
```

### 3、结果的验证

```c++
//-- 三角化
vector<Point3d> points;
triangulation( keypoints_1, keypoints_2, matches, R, t, points );

//-- 验证三角化点与特征点的重投影关系
Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
for ( int i=0; i<matches.size(); i++ )
{
    Point2d pt1_cam = pixel2cam( keypoints_1[ matches[i].queryIdx ].pt, K );
    Point2d pt1_cam_3d(
        points[i].x/points[i].z, 
        points[i].y/points[i].z 
    );
        
    cout<<"point in the first camera frame: "<<pt1_cam<<endl;
    cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points[i].z<<endl;
        
    // 第二个图
    Point2f pt2_cam = pixel2cam( keypoints_2[ matches[i].trainIdx ].pt, K );
    Mat pt2_trans = R*( Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z ) + t;
    pt2_trans /= pt2_trans.at<double>(2,0);
    cout<<"point in the second camera frame: "<<pt2_cam<<endl;
    cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl;
    cout<<endl;
}
```

输出：

```shell
point in the first camera frame: [-0.153772, -0.0742802]
point projected from 3D [-0.153832, -0.0754351], d=16.6993
point in the second camera frame: [-0.180649, -0.0589251]
point reprojected from second frame: [-0.1806478467687954, -0.05780866898967527, 1]
```

第一行：是特征点在相机坐标系下的归一化坐标，由于没有深度，我们只能先归一化。

第二行：该特征点通过三角化得到的点，将其深度(第三维)归一后得到的结果。

可以看出，有一点误差，不过误差较小。

### 4、三角测量的矛盾

​	三角测量是由平移得到的，有平移才会有对极几何中的三角形，如下图所示：

![]()

当平移量很小时候，三角测精度不佳；当平移量较大时，三角测量精度较好。但是，平移较大时，特征点匹配就困难了，因此这就是**三角化的矛盾**。

## 三、3D-2D:PnP

​	全局坐标系下，某个空间点P的其次坐标为(X,Y,Z,1)^T，其在相机的投影像素坐标已知。这时，相机位姿未知，我们用此算法可以估计出相机位姿。

### 1、代码讲解

​	我们使用opencv带的solvePnPRansac来计算PnP。另外，我们还是用BA来计算，并对比二这区别。在这里，我们匹配特征点时候不是用ORB描述子匹配，我们采取LK光流法跟踪特征点。

```c++
void useLK(cv::Mat img,cv::Mat lastImg,cv::Mat lastDepthImg)
{
    //detect FAST
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();

    vector<cv::KeyPoint> lastKeyPoints;
    detector->detect(lastImg,lastKeyPoints);

    //use LK
    list<cv::Point2f> keyPoints;//last image KeyPoints in list form
    list<cv::Point2f> lPoints;
    vector<cv::Point2f> Points,lastPoints;
    vector<unsigned char> status;
    vector<float> error;
    //cv::KepPoint -> cv::Point2f
    for(auto k:lastKeyPoints)
        lastPoints.push_back(k.pt);
    //vector -> list
    for(auto k:lastKeyPoints)
        keyPoints.push_back(k.pt);

    //LK
    cv::calcOpticalFlowPyrLK(lastImg,img,lastPoints,Points,status,error);
    //draw
    cv::Mat temp=lastImg.clone();
    cv::Mat temp1 = img.clone();
    for(auto p:lastPoints)
        cv::circle(temp, p, 10, cv::Scalar(0, 240, 0), 1);
    for(auto p:Points)
        cv::circle(temp1, p, 10, cv::Scalar(0, 240, 0), 1);
    cv::imshow("LK",temp);
    cv::imshow("LK1",temp1);
    cv::waitKey(0);
    cout<<"after LK lastPoints"<<lastPoints.size()<<endl;
    cout<<"after LK Points"<<Points.size()<<endl;
    //vector -> list
    for(auto p:Points)
        lPoints.push_back(p);


    //PnP RANSAC
    //cv::solvePnPRansac( pts3d, pts2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers );

    //delete unTracking Points in lastImg
    //list is convinent for delete or insert
    int i=0;
    for(auto iter = keyPoints.begin();iter != keyPoints.end();++i)
    {
        if(status[i]==0)
        {
            //delete
            iter = keyPoints.erase(iter);
            continue;
        }
        ++iter;
    }

    i=0;
    for(auto iter = lPoints.begin();iter != lPoints.end();++i)
    {
        if(status[i]==0)
        {
            //delete
            iter = lPoints.erase(iter);
            continue;
        }
        ++iter;
    }
    cout<<"after delete lPoints"<<lPoints.size()<<endl;
    //turn KeyPoints to 3DPoints
    //first turn KeyPoints to vec
    lastPoints.clear();
    for(auto iter = keyPoints.begin();iter != keyPoints.end();++iter)
    {
        lastPoints.push_back(*iter);
    }
    keyPoints.clear();

    //turn lastPoints to 3Dpoints
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<cv::Point3f> lastMapPoints;
    vector<cv::Point2f> uvPoints; 
    for(int i=0;i<Points.size();i++)
    {
        cv::Point2f tempP;
        tempP = pixel2cam(lastPoints[i],K);
        ushort d1 = lastDepthImg.at<unsigned short>(int(lastPoints[i].y),int ( lastPoints[i].x ));
        if(d1 == 0)
        {
            continue;
        }
        float dd1;
        dd1 = float(d1)/5000.0;
        lastMapPoints.push_back(cv::Point3f(tempP.x*dd1,tempP.y*dd1,dd1));
        uvPoints.push_back(Points[i]);
    }

    cout<<"before PnP lastMapPoints"<<lastMapPoints.size()<<endl;
    cout<<"before PnP Points"<<uvPoints.size()<<endl;

    cv::Mat rvec, tvec, inliers;
    cv::solvePnPRansac(lastMapPoints,uvPoints,K,cv::Mat(),rvec,tvec,false,100,4.0,0.99,inliers,0);


    //rvec -> R
    cv::Mat R;
    cv::Rodrigues(rvec,R);
    cout<<"R:"<<R<<endl;
    cout<<"t:"<<tvec<<endl;
}
```



## 四、3D-3D:ICP

### 1、概述

​	已知两个3d点(相机坐标系下)成匹配关系，求出相机坐标系的变换(T21)，如想求正的变换，取逆即可，R 和 t的取逆有相应公式可以推导。
$$
p=Rp'+t  \ \\\ (p'->p的相机坐标系变换)
$$

### 2、代码讲解

```c++
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iostream>

using namespace std;

void FeatureMatch(const cv::Mat &refImg,const cv::Mat &curImg,vector<cv::KeyPoint> &refKeyPoint,
                                            vector<cv::KeyPoint> &curKeyPoint,vector<cv::DMatch> &GoodMatch);
void EstimationP3P(vector<cv::Point3f> refPoint,vector<cv::Point3f> curPoint,cv::Mat &R , cv::Mat &t);

cv::Point2d pixel2cam ( const cv::Point2d& p, const cv::Mat& K )
{
    //见下面公式即可
    return cv::Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}
int main(int argc , char** argv)
{
    cv::Mat refImg = cv::imread("/home/gzh/slamstudy/slambook/ch7/1.png" , CV_LOAD_IMAGE_COLOR);
    cv::Mat curImg = cv::imread("/home/gzh/slamstudy/slambook/ch7/2.png" , CV_LOAD_IMAGE_COLOR);
    vector<cv::KeyPoint> refKeyPoint;
    vector<cv::KeyPoint> curKeyPoint;
    vector<cv::DMatch> GoodMatch;
    //特征点匹配，得到GoodMatch以及两张图的关键点
    FeatureMatch(refImg,curImg,refKeyPoint,curKeyPoint,GoodMatch);

    cv::Mat refDepthImg = cv::imread("/home/gzh/slamstudy/slambook/ch7/1_depth.png" , CV_LOAD_IMAGE_UNCHANGED);
    cv::Mat curDepthImg = cv::imread("/home/gzh/slamstudy/slambook/ch7/2_depth.png" , CV_LOAD_IMAGE_UNCHANGED);

    vector<cv::Point3f> refPoint,curPoint;
    cv::Mat K = ( cv::Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for ( cv::DMatch m:GoodMatch )
    {
        //读取像素点对应的深度，Point的x与y需要注意，见附录链接讲解（先Point.y，后Point.x）
        //循环条件为每个GoodMatch，queryIdx是参考帧的关键点Index，trainIdx是现有帧的关键点Index
        ushort d1 = refDepthImg.at<unsigned short>(int(refKeyPoint[m.queryIdx].pt.y),int ( refKeyPoint[m.queryIdx].pt.x ));
        ushort d2 = curDepthImg.at<unsigned short>(int(curKeyPoint[m.trainIdx].pt.y),int ( curKeyPoint[m.trainIdx].pt.x ));
        if ( d1==0 || d2==0 )   // bad depth
            continue;
        //像素坐标转换为相机归一化坐标：su = Kp，其中s为深度，由于像素点不是相机坐标系下的点，而相机坐标系下的点我们不知道其具体深度(需要配合深度图)
        //因此，我们将相机坐标系下的点的z坐标归一，那么直接把s除过去即可，具体原理见下面公式(相机模型)，这样，我们通过深度图得到深度后，直接把
        //归一化坐标再乘真实深度即可。
        cv::Point2d p1 = pixel2cam ( refKeyPoint[m.queryIdx].pt, K );
        cv::Point2d p2 = pixel2cam ( curKeyPoint[m.trainIdx].pt, K );
        float dd1 = float ( d1 ) /5000.0;
        float dd2 = float ( d2 ) /5000.0;
        refPoint.push_back ( cv::Point3f ( p1.x*dd1, p1.y*dd1, dd1 ) );
        curPoint.push_back ( cv::Point3f ( p2.x*dd2, p2.y*dd2, dd2 ) );
    }

    cv::Mat R,t;
    //ICP
    EstimationP3P(refPoint,curPoint,R,t);
    //输出验证
    cout<<R<<endl<<t<<endl;
    return 0;
}

void EstimationP3P(vector<cv::Point3f> refPoint,vector<cv::Point3f> curPoint,cv::Mat &R , cv::Mat &t)
{
    //ji suan zhi xin  he  qu zhi xin zuo biao 
    cv::Point3f pm1,pm2;
    int  N = refPoint.size();
    //计算匹配点的质心坐标
    for(int i=0;i<refPoint.size();i++)
    {
        pm1 += refPoint[i];
        pm2 += curPoint[i];
    }
    pm1 = cv::Point3f( cv::Vec3f(pm1) / N);
    pm2 = cv::Point3f( cv::Vec3f(pm2) / N);

    vector<cv::Point3f>     q1 ( N ), q2 ( N ); // remove the center
    //计算每个匹配点的去质心坐标
    for(int i=0;i<N;i++)
    {
        q1[i] =refPoint[i] - pm1;
        q2[i] = curPoint[i]-pm2;
    }
	//计算矩阵W = sum(q*q^T)    
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for(int i=0;i<N;i++)
    {
        W += Eigen::Vector3d(q1[i].x,q1[i].y,q1[i].z) * Eigen::Vector3d(q2[i].x,q2[i].y,q2[i].z).transpose();
    }
    //SVD分解得到U和V
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU|Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    if (U.determinant() * V.determinant() < 0)
	{
        for (int x = 0; x < 3; ++x)
        {
            U(x, 2) *= -1;
        }
	}
    //得到R和t
    Eigen::Matrix3d R_ = U*V.transpose();
    Eigen::Vector3d t_ = Eigen::Vector3d(pm1.x,pm1.y,pm1.z) - R_*Eigen::Vector3d(pm2.x,pm2.y,pm2.z);

    R = ( cv::Mat_<double>(3,3) <<
            R_(0,0), R_(0,1), R_(0,2),
            R_(1,0), R_(1,1), R_(1,2),
            R_(2,0), R_(2,1), R_(2,2)
            );
    t = ( cv::Mat_<double>(3,1) << t_(0,0), t_(1,0), t_(2,0) );
}

void FeatureMatch(const cv::Mat &refImg,const cv::Mat &curImg,vector<cv::KeyPoint> &refKeyPoint,
                                            vector<cv::KeyPoint> &curKeyPoint,vector<cv::DMatch> &GoodMatch)
{
    cv::Mat descriptors_1, descriptors_2;
    // FAST 
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    //BRIEF
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    //Match
    cv::Ptr<cv::DescriptorMatcher> matcher  = cv::DescriptorMatcher::create ( "BruteForce-Hamming" );
    

    detector->detect(refImg,refKeyPoint);
    detector->detect(curImg,curKeyPoint);

    cv::Mat refDescriptions,curDescriptions;
    descriptor->compute(refImg,refKeyPoint,refDescriptions);
    descriptor->compute(curImg,curKeyPoint,curDescriptions);

    vector<cv::DMatch> matches;
    matcher->match(refDescriptions,curDescriptions,matches);

    //matches filters
    double max_dist = 0.0, min_dist = 100000.0;

    for(int i=0;i<matches.size();i++)
    {
        if(matches[i].distance<min_dist)
            min_dist = matches[i].distance;
        if(matches[i].distance>max_dist)
            max_dist = matches[i].distance;
    }

    for(int i=0;i<matches.size();i++)
    {
        if(matches[i].distance<=cv::max(2*min_dist,30.0))
        {
            GoodMatch.push_back(matches[i]);
        }
    }
}
```

附录1：Point 的x y 与图像坐标x y 的区别：https://blog.csdn.net/wwwlyj123321/article/details/86770662

https://blog.csdn.net/wwwlyj123321/article/details/86770662

附录2：归一化坐标的理解：
$$
相机模型：s\begin{bmatrix}
u\\v\\1
\end{bmatrix}
=\begin{bmatrix}
f_x&0&c_x\\0&f_y&c_y\\0&0&1
\end{bmatrix}
\begin{bmatrix}
x\\y\\z
\end{bmatrix}
$$
上述公式，s=z为图像在相机坐标系下的深度，u,v为像素坐标，[x,y,z]^T为点在相机坐标系下的坐标。相机坐标系下的归一化坐标就是左右都除以s(就是z)，得到：
$$
u=f_xx_1+c_x\\
v=f_yy_1+c_y
$$
其中，[x1,y1,1]^T为归一化坐标，即[x/z,y/z,1]

## 五、BA与图优化
