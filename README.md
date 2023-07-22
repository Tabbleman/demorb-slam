# Toy-slam
一个用来学习的简易slam系统
[[中文](./README.md)|[English](./README-EN.md)]
## 本项目是本人学习高翔老师的《slam14讲》，而进行的拙劣的模仿，为了更好的学习slam
[传送门](https://github.com/gaoxiang12/slambook2/tree/master/ch13)
## 如何使用本项目？

### Prerequisties
此项目实在 *Ubuntu 20.04*下面进行书写的,使用的是c++11的标准,所以你需要支持c++11标准的g++编译器(为什么是g++,因为我在`common_include.h`里面偷懒了,加了个<bits/stdc++.h>的文件头)
除此之外其他的依赖库：
1. eigen3
2. Sophus
3. g2o
4. gtest
5. gflag
6. glog


## 如何构建本项目？
```bash
chmod a+x build.sh
./build.sh
```

### Kitti 数据集
todo 

### ros noetic 
todo 

### ros2 example 
todo 