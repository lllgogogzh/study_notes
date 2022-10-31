## LK

​	目前LK光流法已经在基本图片上通过测试。

​	需要数据：关键帧的关联的地图点->投影为像素坐标->在当前图像上跟踪->删掉跟踪失败的点

​	PnPRansac ：关键帧地图点3D->当前帧跟踪到的2D关键点->3D-2D PnP->得到位姿

​	优化：只能关键帧全局优化，或关键帧局部地图优化。

## 关键帧地图点

```c++
// MapPoints associated to keypoints
std::vector<MapPoint*> mvpMapPoints;
//这个是关键帧关联的地图点，需要
```

```c++
// MapPoint observation functions
void AddMapPoint(MapPoint* pMP, const size_t &idx);
void EraseMapPointMatch(const size_t &idx);
void EraseMapPointMatch(MapPoint* pMP);
void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
std::set<MapPoint*> GetMapPoints();
std::vector<MapPoint*> GetMapPointMatches();
int TrackedMapPoints(const int &minObs);
MapPoint* GetMapPoint(const size_t &idx);
//以上为操作观测到的地图点的操作，可以使用
```

```c++
// KeyPoints, stereo coordinate and descriptors (all associated by an index)
const std::vector<cv::KeyPoint> mvKeys;
const std::vector<cv::KeyPoint> mvKeysUn;
const std::vector<float> mvuRight; // negative value for monocular points
const std::vector<float> mvDepth; // negative value for monocular points
const cv::Mat mDescriptors;
//mvKeys为未去畸变的关键点
//mvKeysUn为去畸变的关键点
```

## 画图：

​	MapDrawer.h中的 SetCurrentCamreaPose函数可以实现。但是由于估计位姿不准确(未优化)，画图不行。考虑看一看位姿估计为啥不准确。
