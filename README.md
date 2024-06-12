# **IFTD: Image Feature Triangle Descriptor for Loop Detection in Driving Scenes**
# **1. Introduction**
**IFTD_detector** In this work, we propose a fast and robust Image Feature Triangle Descriptor (IFTD) based on the STD method, aimed at improving the efficiency and accuracy of place recognition in driving scenarios. We extract keypoints from BEV projection image of point cloud and construct these keypoints into triangle descriptors. By matching these feature triangles, we achieved precise place recognition and calculated the 4-DOF pose estimation between two keyframes. Furthermore, we employ image similarity inspection to perform the final place recognition. Experimental results on three public datasets demonstrate that our IFTD can achieve greater robustness and accuracy than state-of-the-art methods with low computational overhead.

  

## **1.1. Developers:**
The codes of this repo are contributed by:
[Fengtian Lang](), [Ruiye Ming](), [Zikang Yuan]().


## **1.2. Related paper**
Our preprint version is now available on **arxiv**:  
[xxxxxx](xxxxx)

# **2. Prerequisites**

### 2.1 Requirements

> GCC >= 9.4.0
>
> Cmake >= 3.11.0
> 
> [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page) >= 3.3.4
>
> [Ceres](http://ceres-solver.org/installation.html) >= 2.1
> 
> [OpenCV](https://github.com/opencv/opencv) >= 3.4
>
> [PCL](https://pointclouds.org/downloads/) == 1.8 for Ubuntu 18.04, and == 1.10 for Ubuntu 20.04
>
> [ROS](http://wiki.ros.org/ROS/Installation)

##### Have Tested On:

| OS    | GCC  | Cmake | Eigen3 | OpenCV | PCL | 
|:-:|:-:|:-:|:-:|:-:|:-:|
| Ubuntu 18.04 | 9.4.0  | 3.11.2 | 3.3.4 | 3.4.16 | 1.8.0 |

### 2.2 Create ROS workspace

```bash
mkdir -p ~/IFTD/src
cd IFTD/src
```

### 2.3 Clone the directory and build

```bash
git clone https://github.com/EinsTian1/iftd.git
cd ..
catkin_make
```

## **2.4 Prepare for the data**
Since this repo does not implement any method (i.e., LOAM, LIO, etc) for solving the pose for registering the LiDAR scan. So, you have to prepare two set of data for reproducing our results, include: **1) the LiDAR point cloud data. 2) the point cloud registration pose.**

### **2.5.1. LiDAR Point cloud data**
- For the ***Kitti dataset*** (i.e., our Example-1), we read the raw scan data with suffix *".bin"*. These raw LiDAR scan data can be downloaded from the [Kitti Odometry benchmark website](https://www.cvlibs.net/datasets/kitti/eval_odometry.php).
- For the ***Mulran dataset*** (i.e., our Example-1), we read the raw scan data with suffix *".bin"*. These raw LiDAR scan data can be downloaded from the [Mulran Dataset website](https://sites.google.com/view/mulran-pr/home).
- For the ***Nclt dataset*** (i.e., our Example-1), we read the raw scan data with suffix *".bin"*. These raw LiDAR scan data can be downloaded from the [NCLT Dataset website](http://robots.engin.umich.edu/nclt/).
- 
### **2.5.2. Point cloud registration pose**
In the poses file, the poses for LiDAR point cloud registration are given in the following data format:
```
Timestamp pos_x pos_y pos_z quat_x quat_y quat_z quat_w
```
where, ``Timestamp`` is the correspond sampling time stamp of a LiDAR scan, ``pose_{x,y,z}`` and ``quad_{x,y,z,w}`` are the translation and rotation (expressed used quaternion) of pose. 

# **3. Run on Public Datasets**
Noted:

**Please create a folder named "output" in "xxxx" folder before running.** When **IFTD** is running, the result of loop closure is recorded in real time in the **loop.txt** located in the **output folder**.


##  **3.1 Run on [*KITTI*](https://www.cvlibs.net/datasets/kitti/eval_odometry.php)**

First, you should modify the **demo_kitti.launch** file
- Set the **lidar_path** to your local path
- Set the **pose_path** to your local path

Then go to the workspace of **IFTD** and open a terminals, type the following command in terminal :
```
bash
cd IFTD
source devel/setup.bash
roslaunch iftd_detection demo_kitti.launch
```
##  **3.2 Run on [*Mulran*](https://sites.google.com/view/mulran-pr/home)**

First, you should modify the **demo_mulran.launch** file
- Set the **lidar_path** to your local path
- Set the **pose_path** to your local path

Then go to the workspace of **IFTD** and open a terminals, type the following command in terminal :
```
bash
cd IFTD
source devel/setup.bash
roslaunch iftd_detection demo_mulran.launch
```
##  **3.3 Run on [*NCLT*](http://robots.engin.umich.edu/nclt/)**

First, you should modify the **demo_nclt.launch** file
- Set the **lidar_path** to your local path
- Set the **pose_path** to your local path

Then go to the workspace of **IFTD** and open a terminals, type the following command in terminal :
```bash
cd IFTD
source devel/setup.bash
roslaunch iftd_detection demo_nclt.launch
```

## Citation

If you use our work in your research project, please consider citing:

```
xxxxxx
```

## Acknowledgments

Thanks for [STD](https://github.com/hku-mars/STD).
