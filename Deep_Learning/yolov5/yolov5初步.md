# Conda中使用yolov5 (用D455相机)

## 一、Conda安装以及使用

https://blog.csdn.net/qingnianxiaoji/article/details/122278712

## 二、yolov5配置

### 2.1 conda新建环境

```sh
conda create --name yolov5_test python=3.8 -y
conda activate yolov5_test
```

### 2.2 配置yolov5

```sh
git clone git@github.com:ultralytics/yolov5.git
pip3 install -r requirements.txt #安装所需依赖库
```

### 2.3 初步测试

```sh
python3 detect.py --source 1.png --weights yolov5s.pt --img 640
```

如果成功，会输出以下内容

```sh
detect: weights=['yolov5s.pt'], source=1.png, data=data/coco128.yaml, imgsz=[640, 640], conf_thres=0.25, iou_thres=0.45, max_det=1000, device=, view_img=False, save_txt=False, save_conf=False, save_crop=False, nosave=False, classes=None, agnostic_nms=False, augment=False, visualize=False, update=False, project=runs/detect, name=exp, exist_ok=False, line_thickness=3, hide_labels=False, hide_conf=False, half=False, dnn=False, vid_stride=1
YOLOv5 🚀 v6.2-224-g82a5585 Python-3.8.13 torch-1.13.0+cu117 CUDA:0 (NVIDIA GeForce GTX 1660 Ti with Max-Q Design, 5945MiB)

Fusing layers... 
YOLOv5s summary: 213 layers, 7225885 parameters, 0 gradients, 16.4 GFLOPs
image 1/1 /home/gzh/deep_learning_study/yolov5/yolov5/1.png: 448x640 4 persons, 1 laptop, 10.3ms
Speed: 0.5ms pre-process, 10.3ms inference, 1.3ms NMS per image at shape (1, 3, 640, 640)
Results saved to runs/detect/exp3
```

我们找到输出图片：

![img](../yolov5/runs/detect/exp3/1.png)

#### 遇到的问题

1、 torch安装出错误：即torch不能正常使用。

```sh
pip3 uninstall torch #删除torch
pip3 install torch #重新安装
```

## 三、实时目标检测

```sh
mkdir ros_yolo_ws
cd ros_yolov_ws
mkdir src
cd src
git clone git@github.com:qianmin/yolov5_ROS.git
cd ..
catkin_make
```

打开D455相机

```sh
roslaunch realsense2_camera rs_camera.launch
```

将文件final_yolo.py中的话题改为相机话题

最后，在虚拟环境中运行yolo

```sh
source devel/setup.bash#在工作空间source一下
conda activate yolov5_test
rosrun ros_yolo final_yolo.py
```

*注意：需要移除在环境变量中需要移除ros自带的python2.7*

#### 遇到的问题：

https://blog.csdn.net/qq_40280673/article/details/125095353

按照上面的博客改即可

最终可以实现目标检测。



























