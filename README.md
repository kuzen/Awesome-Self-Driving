The Awesome Self-Driving list

- [1. Self-Driving Software](#1-self-driving-software)
- [2. Planning](#2-planning)
- [3. HD Map](#3-hd-map)
  - [3.1. Lanelet2](#31-lanelet2)
  - [3.2. OSM](#32-osm)
  - [3.3. OpenDrive](#33-opendrive)
  - [3.4. VectorMap](#34-vectormap)
  - [3.5. Converter](#35-converter)
- [4. Semantic](#4-semantic)
- [5. Calibration](#5-calibration)
- [6. Detection](#6-detection)
- [7. Control](#7-control)
- [8. SLAM](#8-slam)
  - [8.1. Lidar-based](#81-lidar-based)
  - [8.2. Camera-based](#82-camera-based)
  - [8.3. Lidar-Camera-based](#83-lidar-camera-based)
  - [8.4. Odometry](#84-odometry)
  - [8.5. Registration](#85-registration)
  - [8.6. Optimization](#86-optimization)
  - [8.7. Loop Closing](#87-loop-closing)
  - [8.8. Other](#88-other)
- [9. GPS, IMU](#9-gps-imu)
- [10. Dataset](#10-dataset)
- [11. Web](#11-web)
- [12. Study](#12-study)
- [13. Other Tool](#13-other-tool)
- [14. Other Awesome](#14-other-awesome)
- [15. For Chinese](#15-for-chinese)


# 1. Self-Driving Software
- [apollo](https://github.com/ApolloAuto/apollo) - An open autonomous driving platform
- [Autoware.ai](https://github.com/Autoware-AI/autoware.ai) - Open-source software for self-driving vehicles
- [AutowareArchitectureProposal.proj](https://github.com/tier4/AutowareArchitectureProposal.proj) - This is the source code of the feasibility study for Autoware architecture proposal.
- [Aslan](https://github.com/project-aslan/Aslan) - Open source self-driving software for low speed environments
- [AutoC2X-AW](https://github.com/esakilab/AutoC2X-AW) - AutoC2X is cooperative awareness driving software, extension for Autoware and OpenC2X. 
- [carla](https://github.com/carla-simulator/carla) - Open-source simulator for autonomous driving research.

# 2. Planning
- [PathPlanning](https://github.com/zhm-real/PathPlanning) - Common used path planning algorithms with animations.
- [mesh_navigation](https://github.com/uos/mesh_navigation) - The Mesh Navigation bundle provides software to perform efficient robot navigation on 2D-manifolds in 3D represented as triangular meshes.
- [path_optimizer](https://github.com/LiJiangnanBit/path_optimizer) - Real-time path planning for vehicles.
- [SDC_ND_T3P1_Path_Planning](https://github.com/jkpld/SDC_ND_T3P1_Path_Planning) - Create a path planner to safely drive a car on a 3 lane highway and pass slow cars


# 3. HD Map
- [fmm](https://github.com/cyang-kth/fmm) - Fast map matching, an open source framework in C++
## 3.1. Lanelet2
- [MapToolbox](https://github.com/autocore-ai/MapToolbox) - Plugins to make Lanelet2/vector_map in Unity
- [AssureMappingTools](https://github.com/hatem-darweesh/assuremappingtools) - Desktop based tool for viewing, editing and saving road network maps for autonomous vehicle platforms such as Autoware.
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) - Map handling framework for automated driving
- [lanelet_rviz_plugin_ros](https://github.com/coincar-sim/lanelet_rviz_plugin_ros) - Rviz Plugin for displaying a lanelet2 map.

## 3.2. OSM
- [JOSM](https://github.com/openstreetmap/josm) - the Java OpenStreetMap Editor
- [GMapCatcher](https://github.com/heldersepu/GMapCatcher) - An offline map viewer

## 3.3. OpenDrive
- [opendriveparser](https://github.com/liuyf5231/opendriveparser) - OpenDRIVE Map parser
- [OpenDriveOnlineEditor](https://github.com/Olodus/OpenDriveOnlineEditor) - Web based viewer of OpenDrive (xodr) files (editing not implemented yet)

## 3.4. VectorMap
- [rubimapper](https://github.com/likewatchk/rubimapper) - Autoware vector_map maker

## 3.5. Converter
- [opendrive2lanelets-converter](https://github.com/wenlong-dev/opendrive2lanelets-converter) - This is an unofficial copy of https://commonroad.in.tum.de/tools/opendrive2lanelet
- [opendrive2lanelet](https://github.com/itabrl/opendrive2lanelet) - opendrive to lanelet2
- [osm2xodr](https://github.com/JHMeusener/osm2xodr) - converter for OpenStreetMaps to OpenDrive roads - for use with Carla or other things
- [xodr-OSM-Converter](https://github.com/tiev-tongji/xodr-OSM-Converter) - OpenDrive to osm
- [osm2opendrive](https://github.com/CWGran/osm2opendrive) - A tool for generating OpenDRIVE maps from OpenStreetMap data
- [VectorMap2Opendrive](https://github.com/monkeykane/VectorMap2Opendrive) - Convert Autoware vector map to Opendrive format

# 4. Semantic 
- [lidar-bonnetal](https://github.com/PRBonn/lidar-bonnetal) - Semantic and Instance Segmentation of LiDAR point clouds for autonomous driving
- [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) - Real-Time 3D Semantic Reconstruction from 2D data
- [semantic_suma](https://github.com/PRBonn/semantic_suma) - SuMa++: Efficient LiDAR-based Semantic SLAM (Chen et al IROS 2019)

# 5. Calibration
- [camera_calibration](https://github.com/puzzlepaint/camera_calibration) - Accurate geometric camera calibration with generic camera models
- [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration) - Automatic Extrinsic Calibration Method for LiDAR and Camera Sensor Setups. ROS Package.
- [lidar_camera_calibration](https://github.com/heethesh/lidar_camera_calibration) - Light-weight camera LiDAR calibration package for ROS using OpenCV and PCL (PnP + LM optimization)
- [atom](https://github.com/lardemua/atom) - Calibration tools for multi-sensor, multi-modal, robotic systems
- [FastCertRelPose](https://github.com/mergarsal/FastCertRelPose) - Fast and robust certifiable relative pose estimation
- [ACSC](https://github.com/HViktorTsoi/ACSC) - Automatic Calibration for Non-repetitive Scanning Solid-State LiDAR and Camera Systems
- [kalibr](https://github.com/ethz-asl/kalibr) - The Kalibr visual-inertial calibration toolbox
- [online_calibration](https://github.com/plumewind/online_calibration) - This is an online calibration system between multiple sensors (camera, lidar, IMU). It is being created. . . . . .
- [SensorCalibration](https://github.com/FENGChenxi0823/SensorCalibration) - IMU-Lidar Extrinsic Calibration Package
- [lidar_IMU_calib](https://github.com/APRIL-ZJU/lidar_IMU_calib) - Targetless Calibration of LiDAR-IMU System Based on Continuous-time Batch Estimation
- [lidar_align](https://github.com/ethz-asl/lidar_align) - A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor
- [extrinsic_lidar_camera_calibration](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration) - This is a package for extrinsic calibration between a 3D LiDAR and a camera, described in paper: Improvements to Target-Based 3D LiDAR to Camera Calibration.
- [mrcal](https://github.com/dkogan/mrcal) - mrcal is a generic toolkit to solve calibration and SFM-like problems originating at NASA/JPL. 

# 6. Detection
- [JRMOT_ROS](https://github.com/StanfordVL/JRMOT_ROS) -  A Real-Time 3D Multi-Object Tracker
- [bbox](https://github.com/varunagrawal/bbox) - Python library for 2D/3D bounding boxes
- [CLOCs](https://github.com/pangsu0613/CLOCs) - CLOCs: Camera-LiDAR Object Candidates Fusion for 3D Object Detection
- [CenterFusion](https://github.com/mrnabati/CenterFusion) - CenterFusion: Center-based Radar and Camera Fusion for 3D Object Detection
- [CenterPoint](https://github.com/tianweiy/CenterPoint) - Center-based 3D Object Detection and Tracking
- [OpenPCDet](https://github.com/open-mmlab/OpenPCDet) - OpenPCDet Toolbox for LiDAR-based 3D Object Detection.
- [LaneDetection_End2End](https://github.com/wvangansbeke/LaneDetection_End2End) - End-to-end Lane Detection for Self-Driving Cars (ICCV 2019 Workshop)
- [VoxelNetRos](https://github.com/AbangLZU/VoxelNetRos) - implement the VoxelNet with ROS, using Kitti data to test

# 7. Control
- [mpc_local_planner](https://github.com/rst-tu-dortmund/mpc_local_planner) - It provides a generic and versatile model predictive control implementation with minimum-time and quadratic-form receding-horizon configurations.


# 8. SLAM

## 8.1. Lidar-based
- [r2live](https://github.com/hku-mars/r2live) - R2LIVE is a robust, real-time tightly-coupled multi-sensor fusion framework, which fuses the measurement from the LiDAR, inertial sensor, visual camera to achieve robust, accurate state estimation.
- [LaMa](https://github.com/iris-ua/iris_lama) -  A Localization and Mapping library. Low computational effort and low memory usage whenever possible. 
- [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) - Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
- [SSL_SLAM2](https://github.com/wh200720041/ssl_slam2) - Lightweight 3-D Localization and Mapping for Solid-State LiDAR (mapping and localization separated) ICRA 2021
- [iscloam](https://github.com/wh200720041/iscloam) - Intensity Scan Context based full SLAM implementation for autonomous driving. ICRA 2020
- [loam_livox](https://github.com/hku-mars/loam_livox) - A robust LiDAR Odometry and Mapping (LOAM) package for Livox-LiDAR
- [decentralized_loam](https://github.com/hku-mars/decentralized_loam) - A decentralized framework for simultaneous calibration, localization and mapping with multiple LiDARs
- [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM) - LiDAR-inertial SLAM: Scan Context + LIO-SAM
- [MULLS](https://github.com/YuePanEdward/MULLS) - MULLS: Versatile LiDAR SLAM via Multi-metric Linear Least Square
- [lili-om](https://github.com/KIT-ISAS/lili-om) - LiLi-OM is a tightly-coupled, keyframe-based LiDAR-inertial odometry and mapping system for both solid-state-LiDAR and conventional LiDARs.
- [PyICP-SLAM](https://github.com/gisbi-kim/PyICP-SLAM) - Full-python LiDAR SLAM using ICP and Scan Context
- [SC-LeGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM) - LiDAR SLAM: Scan Context + LeGO-LOAM
- [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) - Advanced implementation of LOAM
- [floam](https://github.com/wh200720041/floam) - Fast LOAM: Fast and Optimized Lidar Odometry And Mapping   for indoor/outdoor localization (Lidar SLAM)
- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) - 3D LIDAR-based Graph SLAM
- [LINS---LiDAR-inertial-SLAM](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) - A Lidar-Inertial State Estimator for Robust and Efficient Navigation based on iterated error-state Kalman filter
- [LOAM-multi-thread](https://github.com/tiger20/LOAM-multi-thread) - Adjust the original LOAM to a multi-threaded version which doesn't require ROS.
- [lio-mapping](https://github.com/hyye/lio-mapping) - Implementation of Tightly Coupled 3D Lidar Inertial Odometry and Mapping (LIO-mapping)
- [lidar_localization](https://github.com/Little-Potato-1990/lidar_localization) - A framework of lidar mapping and localization with strong extensibility
- [StaticMapping](https://github.com/EdwardLiuyc/StaticMapping) - Use LiDAR to map the static world
- [M-LOAM](https://github.com/gogojjh/M-LOAM) - Robust Odometry and Mapping for Multi-LiDAR Systems with Online Extrinsic Calibration


## 8.2. Camera-based
- [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) - ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM

## 8.3. Lidar-Camera-based
- [CamVox](https://github.com/ISEE-Technology/CamVox) - A low-cost SLAM system based on camera and Livox lidar.

## 8.4. Odometry
- [FAST_LIO](https://github.com/hku-mars/FAST_LIO) - A computationally efficient and robust LiDAR-inertial odometry (LIO) package
- [ESVO](https://github.com/HKUST-Aerial-Robotics/ESVO) - This repository maintains the implementation of "Event-based Stereo Visual Odometry".
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) - LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping
- [LOL](https://github.com/RozDavid/LOL) - LOL: Lidar-only Odometry and Localization in 3D point cloud maps

## 8.5. Registration
- [scancontext](https://github.com/irapkaist/scancontext) - Global LiDAR descriptor for place recognition and long-term localization
- [3d-icp-cov](https://github.com/CAOR-MINES-ParisTech/3d-icp-cov) - Code for the paper "A New Approach to 3D ICP Covariance Estimation for Mobile Robotics"
- [TEASER-plusplus](https://github.com/MIT-SPARK/TEASER-plusplus) - A fast and robust point cloud registration library
- [FS3R](https://github.com/zarathustr/FS3R) - A Fast Symbolic 3D Registration Solution from HKUST RAM-LAB
- [dynamic_robot_localization](https://github.com/carlosmccosta/dynamic_robot_localization) - Point cloud registration pipeline for robot localization and 3D perception

## 8.6. Optimization
- [voxgraph](https://github.com/ethz-asl/voxgraph) - Voxblox-based Pose graph optimization
- [Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO) - Robust Pose Graph Optimization

## 8.7. Loop Closing
- [OverlapNet](https://github.com/PRBonn/OverlapNet) - OverlapNet - Loop Closing for 3D LiDAR-based SLAM (chen2020rss)

## 8.8. Other
- [interactive_slam](https://github.com/SMRT-AIST/interactive_slam) - Interactive Map Correction for 3D Graph SLAM
- [Vehicle-Markov-Localization](https://github.com/omerwase/Vehicle-Markov-Localization) - Markov localization using map landmarks and LIDAR sensor data
- [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) - Slam Toolbox for lifelong mapping and localization in potentially massive maps with ROS
- [slam_docker_collection](https://github.com/koide3/slam_docker_collection) - A collection of docker environments for 3D SLAM packages
- [evo](https://github.com/MichaelGrupp/evo) - Python package for the evaluation of odometry and SLAM



# 9. GPS, IMU

- [imu_tools](https://github.com/ccny-ros-pkg/imu_tools) - ROS tools for IMU devices
- [PPPLib](https://github.com/heiwa0519/PPPLib) - Precise Point Positioning Library
- [RTKLIB](https://github.com/tomojitakasu/RTKLIB) - RTKLIB is an open source program package for standard and precise positioning
with GNSS (global navigation satellite system).
- [rtklib_ros_bridge](https://github.com/MapIV/rtklib_ros_bridge) - rtklib_ros_bridge is a package that outputs the latitude and longitude, satellite reception status, altitude, ecef xyz, ecef velocity vector, and Time of Week (GPS Time) calculated by RTKLIB as ROS messages.
- [ntpd_driver](https://github.com/vooon/ntpd_driver) - This node convert ROS TimeReference message to ntpd-shm format.
- [eagleye](https://github.com/MapIV/eagleye) - Precise localization based on GNSS and IMU.
- [GPS_IMU_Kalman_Filter](https://github.com/karanchawla/GPS_IMU_Kalman_Filter) - Fusing GPS, IMU and Encoder sensors for accurate state estimation.
- [imu_utils](https://github.com/gaowenliang/imu_utils) - A ROS package tool to analyze the IMU performance.

# 10. Dataset

- [UrbanLoco](https://github.com/weisongwen/UrbanLoco) - UrbanLoco: A Full Sensor Suite Dataset for Mapping and Localization in Urban Scenes
- [UrbanNavDataset](https://github.com/weisongwen/UrbanNavDataset) - UrbanNav: an Open-Sourcing Localization Data Collected in Asian Urban Canyons, Including Tokyo and Hong Kong

# 11. Web
- [webrtc_ros](https://github.com/RobotWebTools/webrtc_ros) - Streaming of ROS Image Topics using WebRTC
- [AutomanTools](https://github.com/tier4/AutomanTools) - AutomanTools is an open-source software for self-driving AI.
- [Rosbag-Analyser](https://github.com/agrija9/Rosbag-Analyser) - Interactive Post-mortem/Live Analysis of ROS Bag Files
- [webviz](https://github.com/cruise-automation/webviz) - web-based visualization libraries


# 12. Study

- [C-Plus-Plus](https://github.com/TheAlgorithms/C-Plus-Plus) - Collection of various algorithms in mathematics, machine learning, computer science and physics implemented in C++ for educational purposes.
- [modern-cpp-features](https://github.com/AnthonyCalandra/modern-cpp-features) - A cheatsheet of modern C++ language and library features.
- [RosHowTwo](https://github.com/rydb/RosHowTwo) - A place to find and share ROS2 learning resources!
- [Eigen-Cheatsheet](https://github.com/zxl19/Eigen-Cheatsheet) - A cheatsheet of Eigen, the C++ linear algebra library.
- [slambook2](https://github.com/gaoxiang12/slambook2) - edition 2 of the slambook
- [CppRobotics](https://github.com/onlytailei/CppRobotics) - cpp implementation of robotics algorithms including localization, mapping, SLAM, path planning and control
- [robotics-toolbox-python](https://github.com/petercorke/robotics-toolbox-python) - Robotics Toolbox for Python
- [self-driving-car](https://github.com/udacity/self-driving-car) - The Udacity open source self-driving car project
- [Dig-into-Apollo](https://github.com/daohu527/Dig-into-Apollo) - Apollo notes - Apollo learning notes for beginners.
- [Autoware_tutorial](https://github.com/pixmoving-moveit/Autoware_tutorial) - Autoware Simple Tutorial for Self-driving Car

# 13. Other Tool

- [Foxglove Studio](https://github.com/foxglove/studio) - Foxglove Studio (foxglove.dev) is an integrated visualization and diagnosis tool for robotics.
- [RoboStack for ROS Noetic](https://github.com/RoboStack/ros-noetic) - RoboStack, which tightly couples ROS with Conda, a cross-platform, language-agnostic package manager.
- [dart](https://github.com/dartsim/dart) - Dynamic Animation and Robotics Toolkit
- [modelzoo](https://github.com/autowarefoundation/modelzoo) - A collection of machine-learned models for use in autonomous driving applications.
- [ROSIntegration](https://github.com/code-iai/ROSIntegration) - Unreal Engine Plugin to enable ROS Support
- [Fastor](https://github.com/romeric/Fastor) - A lightweight high performance tensor algebra framework for modern C++
- [pointcloud_evaluation_tool](https://github.com/AIS-Bonn/pointcloud_evaluation_tool) - This tool computes the Mean Map Entropy and the Mean Plane Variance of a point cloud.
- [jupyter-ros](https://github.com/RoboStack/jupyter-ros) - Jupyter widget helpers for ROS, the Robot Operating System
- [jupyterlab-ros](https://github.com/RoboStack/jupyterlab-ros) - Helper extensions for running ROS from within JupyterLab
- [bagedit](https://github.com/MHarbi/bagedit) - Catkin version of https://bitbucket.org/daniel_dube/bagedit, which are scripts to manage ROS bag files.
- [rosbag_editor](https://github.com/facontidavide/rosbag_editor) - Create a rosbag from a given one, using a simple GUI
- [laser_line_extraction](https://github.com/kam3k/laser_line_extraction) - A ROS package that extracts line segments from LaserScan messages.
- [rviz_satellite](https://github.com/nobleo/rviz_satellite) - Display internet satellite imagery in RViz
- [ign-rviz](https://github.com/ignitionrobotics/ign-rviz) - ROS 2 visualization using Ignition Libraries
- [python-pcl](https://github.com/strawlab/python-pcl) - Python bindings to the pointcloud library (pcl)
- [pyrobot](https://github.com/facebookresearch/pyrobot) - PyRobot: An Open Source Robotics Research Platform
- [ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning) - Deep learning inference nodes for ROS with support for NVIDIA Jetson TX1/TX2/Xavier and TensorRT
- [imgui_ros](https://github.com/lucasw/imgui_ros) - View ros images, visualize in 2D and 3D, and interact with nodes through topics and services using https://github.com/ocornut/imgui
- [mapviz](https://github.com/swri-robotics/mapviz) - Modular ROS visualization tool for 2D data.
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler) - The Time Series Visualization Tool that you deserve.
- [ROS-Mobile-Android](https://github.com/ROS-Mobile/ROS-Mobile-Android) - Visualization and controlling application for Android
- [ros-integrate](https://github.com/Noam-Dori/ros-integrate) - Extends IntelliJ-Based IDEs with ROS specific development tools
- [PPTK](https://github.com/heremaps/pptk) - The Point Processing Toolkit (pptk) is a Python package for visualizing and processing 2-d/3-d point clouds.


# 14. Other Awesome
- [Lidar_For_AD_references](https://github.com/beedotkiran/Lidar_For_AD_references) - A list of references on lidar point cloud processing for autonomous driving
- [Recent_SLAM_Research](https://github.com/YiChenCityU/Recent_SLAM_Research) - Track Advancement of SLAM 跟踪SLAM前沿动态【2020 version】
- [Awesome-3D-Detectors](https://github.com/Hub-Tian/Awesome-3D-Detectors) - Paperlist of awesome 3D detection methods
- [awesome-pointcloud-processing](https://github.com/Tom-Hardy-3D-Vision-Workshop/awesome-pointcloud-processing) - awesome PointCloud processing algorithm
- [awesome-robotic-tooling](https://github.com/protontypes/awesome-robotic-tooling) - Tooling for professional robotic development in C++ and Python with a touch of ROS, autonomous driving and aerospace: https://freerobotics.tools/
- [awesome-data-labeling](https://github.com/heartexlabs/awesome-data-labeling) - A curated list of awesome data labeling tools
- [Awesome-Autonomous-Driving](https://github.com/autonomousdrivingkr/Awesome-Autonomous-Driving) - 
- [awesome-autonomous-vehicles](https://github.com/manfreddiaz/awesome-autonomous-vehicles) - Curated List of Self-Driving Cars and Autonomous Vehicles Resources
- [awesome-point-cloud-processing](https://github.com/mmolero/awesome-point-cloud-processing) - A curated list of awesome Point Cloud Processing Resources, Libraries, Software
- [awesome-point-cloud-analysis](https://github.com/Yochengliu/awesome-point-cloud-analysis) - A list of papers and datasets about point cloud analysis (processing)

# 15. For Chinese
- [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED) - loam code noted in Chinese（loam中文注解版）
- [LeGO-LOAM_NOTED](https://github.com/wykxwyc/LeGO-LOAM_NOTED) - LeGO-LOAM代码注释与学习
- [loam_leanring](https://github.com/shoufei403/loam_leanring) - Include loam_velodyne code and A-LOAM code with chinese explaintion.I also put some papers related to loam in it.
- [UpdatingHDmapByMonoCamera](https://github.com/JokerJohn/UpdatingHDmapByMonoCamera) - a hdmap  updating project by mono camera and gps
- [awesome-autonomous-vehicle](https://github.com/DeepTecher/awesome-autonomous-vehicle) - 无人驾驶的资源列表中文版
- [python-parallel-programming-cookbook-cn](https://github.com/laixintao/python-parallel-programming-cookbook-cn) - 📖《Python Parallel Programming Cookbook》中文版
- [SLAM-BOOK](https://github.com/yanyan-li/SLAM-BOOK) - 这是一本关于SLAM的书稿，希望能清楚的介绍SLAM系统中的使用的几何方法和深度学习方法。书稿最后应该会达到400页左右，书稿每章对应的代码也会被整理出来。
- [opensource_slam_noted](https://github.com/JokerJohn/opensource_slam_noted) - open source slam system  notes
- [Statistical-Learning-Method_Code](https://github.com/Dod-o/Statistical-Learning-Method_Code) - 手写实现李航《统计学习方法》书中全部算法
- [DeepLearning](https://github.com/MingchaoZhu/DeepLearning) - Python for《Deep Learning》，该书为《深度学习》(花书) 数学推导、原理剖析与源码级别代码实现
- [localization_in_auto_driving](https://github.com/Little-Potato-1990/localization_in_auto_driving) - 从零开始做自动驾驶定位
- [Effective-Modern-Cpp-Zh](https://github.com/Ricardo666666/Effective-Modern-Cpp-Zh) - 42 SPECIFIC WAYS TO IMPROVE YOUR USE OF C++11 AND C++14 中文版

