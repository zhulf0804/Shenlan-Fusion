# 深蓝学院2022 - 多传感器融合作业

## 第一次作业 - 传感器标定

#### 1. 内参标定

**数据集:** euroc 提供的标定数据集
- http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/calibration_datasets/cam_checkerboard/cam_checkerboard.zip
- http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/calibration_datasets/cam_april/cam_april.zip

**代码运行**
```
cd calib/camera_calibration
mkdir build 
cd build
cmake ..
make

## 棋盘格标定
./camera_calib -i /home/lifa/code/calib/cam_checkerboard/mav0/cam0/data

## 二维码标定
./camera_calib -i /home/lifa/code/calib/cam_april/mav0/cam0/data -a
```
**结果**
棋盘格
![](./figures/1-1-intrinsic-calib-1.png)
二维码
![](./figures/1-1-intrinsic-calib-2.png)


