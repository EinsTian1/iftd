# **IFTD: Image Feature Triangle Descriptor for Loop Detection in Driving Scenes**
# **1. Introduction**
**iftd_detector** In this work, we propose a fast and robust Image Feature Triangle Descriptor (IFTD) based on the STD method, aimed at improving the efficiency and accuracy of place recognition in driving scenarios. We extract keypoints from BEV projection image of point cloud and construct these keypoints into triangle descriptors. By matching these feature triangles, we achieved precise place recognition and calculated the 4-DOF pose estimation between two keyframes. Furthermore, we employ image similarity inspection to perform the final place recognition. Experimental results on three public datasets demonstrate that our IFTD can achieve greater robustness and accuracy than stateof-the-art methods with low computational overhead.

  

## **1.1. Developers:**
The codes of this repo are contributed by:
[Fengtian Lang](), [Ruiye Ming](), [Zikang Yuan]().


## **1.2. Related paper**
Our preprint version is now available on **arxiv**:  
[xxxxxx](xxxxx)

## Installation

### 1. Requirements

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

### 2. Create ROS workspace

```bash
mkdir -p ~/IFTD/src
cd IFTD/src
```

### 3. Clone the directory and build

```bash
git clone https://github.com/ZikangYuan/dynamic_lio.git
cd ..
catkin_make
```

## Run on Public Datasets
