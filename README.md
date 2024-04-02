# KF-GINS-DEMO

## Introduction

KF-GINS的实践项目，软件采用C++语言编写，采用CMake管理项目。

Referring：https://github.com/i2Nav-WHU/KF-GINS

使用的硬件平台：

- MCU（RK3308）：基于RK3308的RockPiS，行业内RK3308主要是用来做蓝牙音箱的，以前在RockPiS里面跑过10Hz的RTKLIB，对于基于ESKF的组合导航算法其算力是够的。

- LC29H（AG3335A）：LC29H DA是移远基于AG3335A（L1+L5）的1Hz RTK模块。AG3335A内置一个MCU，主频530MHz、配备4MB flash、664KB SYSRAM以及一些基础的外设，很多初创公司或初设RTK业务的公司都会采用AG3335A作为RTK的研发起点，避免了自己开模做芯片的复杂过程，同时又能将产品体积做得相对较小。

- Realsense D435i（BMI055）：去年年底买的Realsense D435i，主要用作视觉里程计，用以研究SLAM相关的应用，不过我这里主要只是使用里面内置的IMU（BMI055），在低成本IMU里面BMI055、BMI088在机器人应用上使用非常广泛。

## 1 Program Compilation and Execution

### 1.1 Compilation environment

KF-GINS项目使用CMake管理，支持在Linux环境，MacOS环境和Windows环境下编译。我们建议优先选择Linux环境进行编译。KF-GINS编译成功后需要使用配置文件作为参数。程序调试时也需要添加命令行参数。

### 1.2 Dependency libraries
除了基本的C++标准库之外，KF-GINS依赖两个库，分别为Eigen3，yaml-cpp. 这两个库已经作为三方库加到工程源代码中，不需要使用者单独安装。

### 1.3 Compile under Linux

We recommend you use g++ compiler of Ubuntu18.04 or Ubuntu20.04 to compile KF-GINS. You should install the build-essential libraries following the commonds:
```shell
sudo apt-get install cmake
sudo apt-get install build-essential
```

配置好自己的编译环境之后，将仓库克隆到本地后并按照如下操作编译项目：

```shell
# Clone the repository
git clone git@github.com:salmoshu/KF-GINS-DEMO.git ~/

# Build KF-GINS
cd ~/KF-GINS-DEMO
mkdir build && cd build

cmake ../ -DCMAKE_BUILD_TYPE=Release 
make -j8

# Run demo dataset
cd ~/KF-GINS-DEMO
./bin/KF-GINS ./dataset/basic.yaml

# Wait until the program finish
```


## 2 关键工作

### 2.1 minmea

Referring：https://github.com/kosma/minmea

-GBS (Satellite Fault Detection)
- GGA (Fix Data)
- GLL (Geographic Position: Latitude/Longitude)
- GSA (DOP and active satellites)
- GST (Pseudorange Noise Statistics)
- GSV (Satellites in view)
- RMC (Recommended Minimum: position, velocity, time)
- VTG (Track made good and Ground speed)
- ZDA (Time & Date - UTC, day, month, year and local time zone)

Adding support for more sentences is trivial; see minmea.cpp source. 

### 2.2 rtklib

Referring：https://github.com/rtklibexplorer/RTKLIB

KF-GINS要求的GNSS数据使用了GPS时间（具体采用了周数+周内秒的形式），因此我需要增加UTC至GPS时间的转换函数，这里可以参考RTKLIB。

### 2.3 sensync

Referring：https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/README.md

IMU线性插值做法是在pipe.start(cfg, callback)的回调函数中增加插值函数FillImuData_LinearInterpolation，它主要对加速度计的数据进行了插值或外推从而跟陀螺仪数据对齐，其中比较核心的一个步骤是：

template <typename T> T lerp(const T &a, const T &b, const double t) {
    return a * (1.0 - t) + b * t;
}
这段代码是一个线性插值函数，用于在两个值a和b之间进行插值。参数t表示插值的比例，取值范围为[0, 1]，它由加速度计和陀螺仪数据之间的相对时间间隔得到。