# LoopClosing线程

## 一、概述

​	与LocalMapping线程类似，在System.cc中我们可以找到**初始化并启动闭环检测线程的语句。**

```c++
//Initialize the Loop Closing thread and launch
mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
```

​	因此，这里主要讲解Run()函数及其中的其他函数。

我们这里直接用一个图来解释回环检测

![](/home/gzh/slamstudy/note/orb-slam2代码阅读笔记/img/回环检测-1.png)

*上图中，(0,0)与(0.5,0.5)闭环，相对位姿可认为零，update表示更新当前帧的相邻帧，下标应为gwj，对应图中点(0.4,1.4)。*

​	这里，gcw类似数学符号为sim3变换，7自由度(比旋转平移多了一个尺度s)。如果为RGBD或双目相机，那就是6自由度，但是也需要区分新的Tcw和旧的Tcw。

## 二、类成员及方法



## 三、Run()函数

​	流程比较清晰：当闭环检测队列中有关键帧时，检测回环，并且计算相似变换，最后闭环调整。

```c++
void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        // Loopclosing中的关键帧是LocalMapping发送过来的，LocalMapping是Tracking中发过来的
        // 在LocalMapping中通过InsertKeyFrame将关键帧插入闭环检测队列mlpLoopKeyFrameQueue
        // 闭环检测队列mlpLoopKeyFrameQueue中的关键帧不为空
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }

        ResetIfRequested();

        if(CheckFinish())
            break;

        //usleep(5000);
		std::this_thread::sleep_for(std::chrono::milliseconds(5));

	}

    SetFinish();
}
```

## 四、其他函数

### 1、CheckNewKeyFrames()函数

​	若队列不未空，则代表有新的关键帧。

```c++
/**
 * 查看列表中是否有等待被插入的关键帧
 * @return 如果存在，返回true
 */
bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}
```

### 2、DetectLoop()函数

​	检测闭环的函数，返回值代表是否检测成功。

#### 步骤一：

​	mlpLoopKeyFrameQueue为关键帧队列，mpCurretKF为闭环检测线程的参考关键帧。

```c++
// 步骤一：从队列中取出一个关键帧
unique_lock<mutex> lock(mMutexLoopQueue);
//mlpLoopKeyFrameQueue为关键帧队列
mpCurrentKF = mlpLoopKeyFrameQueue.front();
//弹出
mlpLoopKeyFrameQueue.pop_front();
// Avoid that a keyframe can be erased while it is being process by this thread
mpCurrentKF->SetNotErase();
```

#### 步骤二：

```c++
//If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
// 步骤1：如果距离上次闭环没多久（小于10帧），或者map中关键帧总共还没有10帧，则不进行闭环检测
if(mpCurrentKF->mnId<mLastLoopKFid+10)
{
    mpKeyFrameDB->add(mpCurrentKF);
    mpCurrentKF->SetErase();
    return false;
}
```

#### 步骤三：

```c++
// Compute reference BoW similarity score
// This is the lowest score to a connected keyframe in the covisibility graph
// We will impose loop candidates to have a higher similarity than this
// VIII-A
// 步骤2：遍历所有共视关键帧，计算当前关键帧与每个共视关键的bow相似度得分，并得到最低得分minScore
const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
float minScore = 1;
for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
{
    KeyFrame* pKF = vpConnectedKeyFrames[i];
    if(pKF->isBad())
        continue;
    const DBoW2::BowVector &BowVec = pKF->mBowVec;

    float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

    if(score<minScore)
        minScore = score;
}
```

#### 步骤四：

```c++
// Query the database imposing the minimum score
// 步骤3：在所有关键帧中找出闭环备选帧
vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

// If there are no loop candidates, just add new keyframe and return false
if(vpCandidateKFs.empty())
{
    mpKeyFrameDB->add(mpCurrentKF);
    mvConsistentGroups.clear();
    mpCurrentKF->SetErase();
    return false;
}
```

#### 步骤五：

​	mvpEnoughConsistentCandidates 最终筛选后得到的闭环帧，类型为：vector<KeyFrame* >

```c++
// For each loop candidate check consistency with previous loop candidates
// Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
// A group is consistent with a previous group if they share at least a keyframe
// We must detect a consistent loop in several consecutive keyframes to accept it
// 步骤5：在候选帧中检测具有连续性的候选帧
// 1、每个候选帧将与自己相连的关键帧构成一个“子候选组spCandidateGroup”，vpCandidateKFs-->spCandidateGroup
// 2、检测“子候选组”中每一个关键帧是否存在于“连续组”，如果存在nCurrentConsistency++，则将该“子候选组”放入“当前连续组vCurrentConsistentGroups”
// 3、如果nCurrentConsistency大于等于3，那么该”子候选组“代表的候选帧过关，进入mvpEnoughConsistentCandidates
mvpEnoughConsistentCandidates.clear();// 最终筛选后得到的闭环帧
```

​	定义一些关于连续组、子连续组等概念

**成员：mvConsistentGroups 类型为 vector < ConsistentGroup > ，连续组的vector**

**ConsistentGroup数据类型为pair<set<KeyFrame*>,int>**

```c++
// ConsistentGroup数据类型为pair<set<KeyFrame*>,int>
// ConsistentGroup.first对应每个“连续组”中的关键帧，ConsistentGroup.second为每个“连续组”的序号
vector<ConsistentGroup> vCurrentConsistentGroups;
vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
```

​	对闭环候选帧进行遍历(闭环候选帧：所有关键帧中，与当前关键帧评分 大于 当前关键帧与一级相连的关键帧的评分 的关键帧)

​	**连续组：已经判定为连续的ConsistentGroups集合**

​	**候选组：通过评分得到的所有ConsistentGroups集合**

​	**子候选组：候选关键帧+与之一级相邻的关键帧 的集合**

​	**子连续组：连续组中的 一个关键帧+与之以及相邻的关键帧 的集合**

​	当前子候选组中的任意关键帧 在 子连续组中存在，则“bConsistent = true”， "bConsistentForSomeGroup = true"。

​	当 当前子候选组与子连续组 相连，则将子候选组加入子连续组(为后续匹配)。

​	当 当前子候选组完成 与 n个子连续组的匹配后，将当前子候选组对应的关键帧放入闭环候选帧。

```c++
for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
{
    KeyFrame* pCandidateKF = vpCandidateKFs[i];

    // 将自己以及与自己相连的关键帧构成一个“子候选组”
    set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
    spCandidateGroup.insert(pCandidateKF);

    bool bEnoughConsistent = false;
    bool bConsistentForSomeGroup = false;
    // 遍历之前的“子连续组”
    for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
    {
        // 取出一个之前的子连续组
        set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

        // 遍历每个“子候选组”，检测候选组中每一个关键帧在“子连续组”中是否存在
        // 如果有一帧共同存在于“子候选组”与之前的“子连续组”，那么“子候选组”与该“子连续组”连续
        bool bConsistent = false;
        for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
        {
            if(sPreviousGroup.count(*sit))
            {
                bConsistent=true;// 该“子候选组”与该“子连续组”相连
                bConsistentForSomeGroup=true;// 该“子候选组”至少与一个”子连续组“相连
                break;
            }
        }

        if(bConsistent)
        {
            int nPreviousConsistency = mvConsistentGroups[iG].second;
            int nCurrentConsistency = nPreviousConsistency + 1;
            if(!vbConsistentGroup[iG])
            {
                // 将该“子候选组”的该关键帧打上编号加入到“当前连续组”
                ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                vCurrentConsistentGroups.push_back(cg);
                vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
            }
            if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
            {
                //将当前遍历的候选关键帧加入闭环关键帧的条件为：bEnoughConsistent==false 且 连续数目大于等于给定数目
                //大条件：bConsistent为true
                //而nCurrentConsistency在上一个条件中有所更改
                //其意义为：当前连续组数量+1
                mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                bEnoughConsistent=true; //this avoid to insert the same candidate more than once
            }

            //这里是不是缺一个break来提高效率呢？(wubo???)
        }
    }

    // If the group is not consistent with any previous group insert with consistency counter set to zero
    // 如果该“子候选组”的所有关键帧都不存在于“子连续组”，那么vCurrentConsistentGroups将为空，
    // 于是就把“子候选组”全部拷贝到vCurrentConsistentGroups，并最终用于更新mvConsistentGroups，计数器设为0，重新开始
    if(!bConsistentForSomeGroup)
    {
        // 这个地方是不是最好clear一下vCurrentConsistentGroups呢？(wubo???)

        ConsistentGroup cg = make_pair(spCandidateGroup,0);
        vCurrentConsistentGroups.push_back(cg);
    }
}
```

#### 步骤六：

```c++
// Update Covisibility Consistent Groups
mvConsistentGroups = vCurrentConsistentGroups;


// Add Current Keyframe to database
mpKeyFrameDB->add(mpCurrentKF);

if(mvpEnoughConsistentCandidates.empty())
{
    mpCurrentKF->SetErase();
    return false;
}
else
{
    return true;
}

mpCurrentKF->SetErase();
return false;
```

### 3、ComputeSim3()函数

#### 概述

​	每一个闭环关键帧都要做这个操作，注意RGBD和双目相机尺度信息为1，即解算出T即可。

```c++
/**
 * @brief 计算当前帧与闭环帧的Sim3变换等
 *
 * 1. 通过Bow加速描述子的匹配，利用RANSAC粗略地计算出当前帧与闭环帧的Sim3（当前帧---闭环帧）
 * 2. 根据估计的Sim3，对3D点进行投影找到更多匹配，通过优化的方法计算更精确的Sim3（当前帧---闭环帧）
 * 3. 将闭环帧以及闭环帧相连的关键帧的MapPoints与当前帧的点进行匹配（当前帧---闭环帧+相连关键帧）
 * 
 * 注意以上匹配的结果均都存在成员变量mvpCurrentMatchedPoints中，
 * 实际的更新步骤见CorrectLoop()步骤3：Start Loop Fusion
 */
bool LoopClosing::ComputeSim3();
```

**一些局部变量**

```c++
// For each consistent loop candidate we try to compute a Sim3
// 闭环候选帧数量
const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

// We compute first ORB matches for each candidate
// If enough matches are found, we setup a Sim3Solver
ORBmatcher matcher(0.75,true);

vector<Sim3Solver*> vpSim3Solvers;
vpSim3Solvers.resize(nInitialCandidates);// 每个候选帧都有一个Sim3Solver

vector<vector<MapPoint*> > vvpMapPointMatches;
vvpMapPointMatches.resize(nInitialCandidates);

vector<bool> vbDiscarded;
vbDiscarded.resize(nInitialCandidates);
```



#### 步骤一二：

```c++
for(int i=0; i<nInitialCandidates; i++)
{
    // 步骤1：从筛选的闭环候选帧中取出一帧关键帧pKF
    KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

    // avoid that local mapping erase it while it is being processed in this thread
    // 防止在LocalMapping中KeyFrameCulling函数将此关键帧作为冗余帧剔除
    pKF->SetNotErase();

    if(pKF->isBad())
    {
        vbDiscarded[i] = true;// 直接将该候选帧舍弃
        continue;
    }

    // 步骤2：将当前帧mpCurrentKF与闭环候选关键帧pKF匹配
    // 通过bow加速得到mpCurrentKF与pKF之间的匹配特征点，vvpMapPointMatches是匹配特征点对应的MapPoints
    int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

    // 匹配的特征点数太少，该候选帧剔除
    if(nmatches<20)
    {
        vbDiscarded[i] = true;
        continue;
    }
    else
    {
        // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
        // 构造Sim3求解器
        // 如果mbFixScale为true，则是6DoFf优化（双目 RGBD），如果是false，则是7DoF优化（单目）
        Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
        pSolver->SetRansacParameters(0.99,20,300);// 至少20个inliers 300次迭代
        vpSim3Solvers[i] = pSolver;
    }

    // 参与Sim3计算的候选关键帧数加1
    nCandidates++;
}
```



### 4、CorrectLoop()函数

#### 概述

```c++
/**
 * @brief 闭环
 *
 * 1. 通过求解的Sim3以及相对姿态关系，调整与当前帧相连的关键帧位姿以及这些关键帧观测到的MapPoints的位置（相连关键帧---当前帧）
 * 2. 将闭环帧以及闭环帧相连的关键帧的MapPoints和与当前帧相连的关键帧的点进行匹配（相连关键帧+当前帧---闭环帧+相连关键帧）
 * 3. 通过MapPoints的匹配关系更新这些帧之间的连接关系，即更新covisibility graph
 * 4. 对Essential Graph（Pose Graph）进行优化，MapPoints的位置则根据优化后的位姿做相对应的调整
 * 5. 创建线程进行全局Bundle Adjustment
 */
void LoopClosing::CorrectLoop()
```

#### 步骤一：

```c++
// Ensure current keyframe is updated
// 步骤1：根据共视关系更新当前帧与其它关键帧之间的连接
mpCurrentKF->UpdateConnections();
```

#### 步骤二：

```c++
// Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
// 步骤2：通过位姿传播，得到Sim3优化后，与当前帧相连的关键帧的位姿，以及它们的MapPoints
// 当前帧与世界坐标系之间的Sim变换在ComputeSim3函数中已经确定并优化，
// 通过相对位姿关系，可以确定这些相连的关键帧与世界坐标系之间的Sim3变换
```

​	步骤二我们逐一分析

​	首先，取出与当前关键帧相连的关键帧(共视)，并且包括当前帧，这个vector为**mvpCurrentConnectedKFs**。

```c++
// 取出与当前帧相连的关键帧，包括当前关键帧
mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
mvpCurrentConnectedKFs.push_back(mpCurrentKF);
```

```c++
KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
//typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> > > KeyFrameAndPose;
//最外层map<关键帧，>?????????????????????????????????????????????????????


// 先将mpCurrentKF的Sim3变换存入，固定不动
CorrectedSim3[mpCurrentKF]=mg2oScw;
cv::Mat Twc = mpCurrentKF->GetPoseInverse();
```

​	其次，通过位姿传播，得到当前帧位姿与相邻关键帧位姿相对值。
$$
T_{iw}： \ world到pKFi的位姿变换\\
T_{cw}： \ world到CurrentKF的位姿变换\\
T_{ci}： \ CurrentKF到pKFi的位姿变换
$$


```c++
// 步骤2.1：通过位姿传播，得到Sim3调整后其它与当前帧相连关键帧的位姿（只是得到，还没有修正）
for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
{
    KeyFrame* pKFi = *vit;

    cv::Mat Tiw = pKFi->GetPose();

    // currentKF在前面已经添加
    if(pKFi!=mpCurrentKF)
    {
        // 得到当前帧到pKFi帧的相对变换
        cv::Mat Tic = Tiw*Twc;
        cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
        cv::Mat tic = Tic.rowRange(0,3).col(3);
        g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
        // 当前帧的位姿固定不动，其它的关键帧根据相对关系得到Sim3调整的位姿
        g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
        // Pose corrected with the Sim3 of the loop closure
        // 得到闭环g2o优化后各个关键帧的位姿
        CorrectedSim3[pKFi]=g2oCorrectedSiw;
    }

    cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
    cv::Mat tiw = Tiw.rowRange(0,3).col(3);
    g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
    // Pose without correction
    // 当前帧相连关键帧，没有进行闭环g2o优化的位姿
    NonCorrectedSim3[pKFi]=g2oSiw;
}
```

























