# The Awesome Self-Driving List [![Awesome](https://awesome.re/badge-flat.svg)](https://awesome.re)

## Contents

- [Contents](#contents)
- [1. Self-Driving Software](#1-self-driving-software)
- [2. Planning](#2-planning)
- [3. HD Map](#3-hd-map)
  - [3.1. Lanelet2](#31-lanelet2)
  - [3.2. OSM](#32-osm)
  - [3.3. OpenDrive](#33-opendrive)
  - [3.4. Others](#34-others)
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
- [11. Simulation](#11-simulation)
- [12. Visualization](#12-visualization)
- [13. Web](#13-web)
- [14. Study](#14-study)
- [15. Other Tool](#15-other-tool)
- [16. Other Awesome](#16-other-awesome)
- [17. For Chinese](#17-for-chinese)


## 1. Self-Driving Software

- [apollo](https://github.com/ApolloAuto/apollo) - An open autonomous driving platform.
- [Autoware.ai](https://github.com/Autoware-AI/autoware.ai) - Open-source software for self-driving vehicles.
- [AutowareArchitectureProposal.proj](https://github.com/tier4/AutowareArchitectureProposal.proj) - This is the source code of the feasibility study for Autoware architecture proposal.
- [self-driving-ish_computer_vision_system](https://github.com/iwatake2222/self-driving-ish_computer_vision_system) - This project generates images you've probably seen in autonomous driving demo.
- [Aslan](https://github.com/project-aslan/Aslan) - Open source self-driving software for low speed environments.
- [AutoC2X-AW](https://github.com/esakilab/AutoC2X-AW) - AutoC2X is cooperative awareness driving software, extension for Autoware and OpenC2X.

## 2. Planning

- [ompl](https://github.com/ompl/ompl) - The Open Motion Planning Library (OMPL).
- [omg-tools](https://github.com/meco-group/omg-tools) - Optimal Motion Generation-tools: motion planning made easy.
- [gbplanner_ros](https://github.com/ntnu-arl/gbplanner_ros) - Graph-based Exploration Planner for Subterranean Environments.
- [path_planner](https://github.com/karlkurzer/path_planner) - Hybrid A* Path Planner for the KTH Research Concept Vehicle.
- [PathPlanning](https://github.com/zhm-real/PathPlanning) - Common used path planning algorithms with animations.
- [MotionPlanning](https://github.com/zhm-real/MotionPlanning) - Motion planning algorithms commonly used on autonomous vehicles. (path planning + path tracking).
- [se2_navigation](https://github.com/leggedrobotics/se2_navigation) - Pure pursuit controller and Reeds-Shepp sampling based planner for navigation in SE(2) space.
- [far_planner](https://github.com/MichaelFYang/far_planner) - Fast, Attemptable Route Planner for Navigation in Known and Unknown Environments
- [mesh_navigation](https://github.com/uos/mesh_navigation) - The Mesh Navigation bundle provides software to perform efficient robot navigation on 2D-manifolds in 3D represented as triangular meshes.
- [path_optimizer](https://github.com/LiJiangnanBit/path_optimizer) - Real-time path planning for vehicles.
- [full_coverage_path_planner](https://github.com/nobleo/full_coverage_path_planner) - Full coverage path planning provides a move_base_flex plugin that can plan a path that will fully cover a given area.
- [conformal_lattice_planner](https://github.com/KumarRobotics/conformal_lattice_planner) - conformal lattice planner C++ implementations for autonomous driving tasks. The software is build upon Carla and ROS.

## 3. HD Map

- [fmm](https://github.com/cyang-kth/fmm) - Fast map matching, an open source framework in C++.
- [Tier4 Tool](https://tools.tier4.jp/) - Vector Map Builder is a tool that helps to create a vector map from point cloud data.
  
### 3.1. Lanelet2

- [MapToolbox](https://github.com/autocore-ai/MapToolbox) - Plugins to make Lanelet2/vector_map in Unity.
- [AssureMappingTools](https://github.com/hatem-darweesh/assuremappingtools) - Desktop based tool for viewing, editing and saving road network maps for autonomous vehicle platforms such as Autoware.
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2) - Map handling framework for automated driving.
- [lanelet_rviz_plugin_ros](https://github.com/coincar-sim/lanelet_rviz_plugin_ros) - Rviz Plugin for displaying a lanelet2 map.

### 3.2. OSM

- [JOSM](https://github.com/openstreetmap/josm) - The Java OpenStreetMap Editor.
- [prettymaps](https://github.com/marceloprates/prettymaps) - A small set of Python functions to draw pretty maps from OpenStreetMap data. Based on osmnx, matplotlib and shapely libraries.
- [GMapCatcher](https://github.com/heldersepu/GMapCatcher) - An offline map viewer.

### 3.3. OpenDrive

- [libOpenDRIVE](https://github.com/grepthat/libOpenDRIVE) - Small, lightweight C++ library for handling OpenDRIVE filess.
- [opendriveparser](https://github.com/liuyf5231/opendriveparser) - OpenDRIVE Map parser.
- [OpenDriveOnlineEditor](https://github.com/Olodus/OpenDriveOnlineEditor) - Web based viewer of OpenDrive (xodr) files (editing not implemented yet).

### 3.4. Others

- [QGIS](https://github.com/qgis/QGIS) - QGIS is a free, open source, cross platform (lin/win/mac) geographical information system (GIS).
- [OpenHDMap](https://github.com/Flycars/OpenHDMap) - An open HD map production process for autonomous car simulation.
- [opendrive2lanelets-converter](https://github.com/wenlong-dev/opendrive2lanelets-converter) - This is an unofficial copy of <https://commonroad.in.tum.de/tools/opendrive2lanelet>.
- [opendrive2lanelet](https://github.com/itabrl/opendrive2lanelet) - Opendrive to lanelet2.
- [osm2xodr](https://github.com/JHMeusener/osm2xodr) - Converter for OpenStreetMaps to OpenDrive roads - for use with Carla or other things.
- [xodr-OSM-Converter](https://github.com/tiev-tongji/xodr-OSM-Converter) - OpenDrive to osm.
- [osm2opendrive](https://github.com/CWGran/osm2opendrive) - A tool for generating OpenDRIVE maps from OpenStreetMap data.
- [VectorMap2Opendrive](https://github.com/monkeykane/VectorMap2Opendrive) - Convert Autoware vector map to Opendrive format.

## 4. Semantic

- [lift-splat-shoot](https://github.com/nv-tlabs/lift-splat-shoot) - Lift, Splat, Shoot: Encoding Images from Arbitrary Camera Rigs by Implicitly Unprojecting to 3D (ECCV 2020).
- [lidar-bonnetal](https://github.com/PRBonn/lidar-bonnetal) - Semantic and Instance Segmentation of LiDAR point clouds for autonomous driving.
- [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) - Real-Time 3D Semantic Reconstruction from 2D data.
- [semantic_suma](https://github.com/PRBonn/semantic_suma) - SuMa++: Efficient LiDAR-based Semantic SLAM (Chen et al IROS 2019).
- [mmsegmentation](https://github.com/open-mmlab/mmsegmentation) - OpenMMLab Semantic Segmentation Toolbox and Benchmark.

## 5. Calibration

- [multiple-cameras-and-3D-LiDARs-extrinsic-calibration](https://github.com/alibaba/multiple-cameras-and-3D-LiDARs-extrinsic-calibration) - This repo contains source code of our paper presented in IROS2021 "Single-Shot is Enough: Panoramic Infrastructure Based Calibration of Multiple Cameras and 3D LiDARs"
- [SensorsCalibration](https://github.com/PJLab-ADG/SensorsCalibration) - OpenCalib: A Multi-sensor Calibration Toolbox for Autonomous Driving.
- [multi_sensor_calibration](https://github.com/tudelft-iv/multi_sensor_calibration) - A calibration tool to calibrate a sensor setup consisting of lidars, radars and cameras.
- [camera_calibration](https://github.com/puzzlepaint/camera_calibration) - Accurate geometric camera calibration with generic camera models.
- [image_projection](https://github.com/tu-darmstadt-ros-pkg/image_projection) - Image_projection is a ROS package to create various projections from multiple calibrated cameras.
- [velo2cam_calibration](https://github.com/beltransen/velo2cam_calibration) - Automatic Extrinsic Calibration Method for LiDAR and Camera Sensor Setups. ROS Package.
- [lidar_camera_calibration](https://github.com/heethesh/lidar_camera_calibration) - Light-weight camera LiDAR calibration package for ROS using OpenCV and PCL (PnP + LM optimization).
- [atom](https://github.com/lardemua/atom) - Calibration tools for multi-sensor, multi-modal, robotic systems.
- [FastCertRelPose](https://github.com/mergarsal/FastCertRelPose) - Fast and robust certifiable relative pose estimation.
- [ACSC](https://github.com/HViktorTsoi/ACSC) - Automatic Calibration for Non-repetitive Scanning Solid-State LiDAR and Camera Systems.
- [kalibr](https://github.com/ethz-asl/kalibr) - The Kalibr visual-inertial calibration toolbox.
- [SensorCalibration](https://github.com/FENGChenxi0823/SensorCalibration) - IMU-Lidar Extrinsic Calibration Package.
- [lidar_IMU_calib](https://github.com/APRIL-ZJU/lidar_IMU_calib) - Targetless Calibration of LiDAR-IMU System Based on Continuous-time Batch Estimation.
- [lidar_align](https://github.com/ethz-asl/lidar_align) - A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor.
- [extrinsic_lidar_camera_calibration](https://github.com/UMich-BipedLab/extrinsic_lidar_camera_calibration) - This is a package for extrinsic calibration between a 3D LiDAR and a camera, described in paper: Improvements to Target-Based 3D LiDAR to Camera Calibration.
- [mrcal](https://github.com/dkogan/mrcal) - MRCAL is a generic toolkit to solve calibration and SFM-like problems originating at NASA/JPL.

## 6. Detection

- [MapTR](https://github.com/hustvl/MapTR) - [ICLR'23 Spotlight] MapTR: Structured Modeling and Learning for Online Vectorized HD Map Construction
- [patchwork++](https://github.com/url-kaist/patchwork-plusplus) - Patchwork++: Fast and robust ground segmentation method for 3D point cloud. @ IROS'22.
- [urban_road_filter](https://github.com/jkk-research/urban_road_filter) - Real-time LIDAR-based Urban Road and Sidewalk detection for Autonomous Vehicles.
- [conditional-lane-detection](https://github.com/aliyun/conditional-lane-detection) - "CondLaneNet: a Top-to-down Lane Detection Framework Based on ConditionalConvolution".
- [LaneATT](https://github.com/lucastabelini/LaneATT) - Code for the paper entitled "Keep your Eyes on the Lane: Real-time Attention-guided Lane Detection" (CVPR 2021).
- [Pseudo-LiDAR](https://github.com/mileyan/pseudo_lidar) - (CVPR 2019) Pseudo-LiDAR from Visual Depth Estimation: Bridging the Gap in 3D Object Detection for Autonomous Driving.
- [depth_clustering](https://github.com/PRBonn/depth_clustering) - Fast and robust clustering of point clouds generated with a Velodyne sensor.
- [AB3DMOT](https://github.com/xinshuoweng/AB3DMOT) - Official Python Implementation for "3D Multi-Object Tracking: A Baseline and New Evaluation Metrics".
- [JRMOT_ROS](https://github.com/StanfordVL/JRMOT_ROS) -  A Real-Time 3D Multi-Object Tracker.
- [bbox](https://github.com/varunagrawal/bbox) - Python library for 2D/3D bounding boxes.
- [CLOCs](https://github.com/pangsu0613/CLOCs) - CLOCs: Camera-LiDAR Object Candidates Fusion for 3D Object Detection
- [CenterFusion](https://github.com/mrnabati/CenterFusion) - CenterFusion: Center-based Radar and Camera Fusion for 3D Object Detection.
- [CenterPoint](https://github.com/tianweiy/CenterPoint) - Center-based 3D Object Detection and Tracking.
- [OpenPCDet](https://github.com/open-mmlab/OpenPCDet) - OpenPCDet Toolbox for LiDAR-based 3D Object Detection.
- [LaneDetection_End2End](https://github.com/wvangansbeke/LaneDetection_End2End) - End-to-end Lane Detection for Self-Driving Cars (ICCV 2019 Workshop).
- [VoxelNetRos](https://github.com/AbangLZU/VoxelNetRos) - Implement the VoxelNet with ROS, using Kitti data to test.
- [PLARD](https://github.com/zhechen/PLARD) - Progressive LiDAR Adaptation for Road Detection.
- [RoadMarkingExtraction](https://github.com/YuePanEdward/RoadMarkingExtraction) - A C++ Program for automatically extraction of road markings from MLS or ALS point cloud [ISPRS-A' 19].

## 7. Control

- [MPCC](https://github.com/alexliniger/MPCC) - Model Predictive Contouring Controller (MPCC) for Autonomous Racing.
- [mpc_local_planner](https://github.com/rst-tu-dortmund/mpc_local_planner) - It provides a generic and versatile model predictive control implementation with minimum-time and quadratic-form receding-horizon configurations.


## 8. SLAM

### 8.1. Lidar-based

- [LT-mapper](https://github.com/gisbi-kim/lt-mapper) - A Modular Framework for LiDAR-based Lifelong Mapping.
- [clins](https://github.com/APRIL-ZJU/clins) - CLINS: Continuous-Time Trajectory Estimation for LiDAR-Inertial System.
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) - LIO-SAM: Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping.
- [LINS---LiDAR-inertial-SLAM](https://github.com/ChaoqinRobotics/LINS---LiDAR-inertial-SLAM) - A Lidar-Inertial State Estimator for Robust and Efficient Navigation based on iterated error-state Kalman filter.
- [LaMa](https://github.com/iris-ua/iris_lama) -  A Localization and Mapping library. Low computational effort and low memory usage whenever possible.
- [LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM) - Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain.
- [loam_livox](https://github.com/hku-mars/loam_livox) - A robust LiDAR Odometry and Mapping (LOAM) package for Livox-LiDAR.
- [decentralized_loam](https://github.com/hku-mars/decentralized_loam) - A decentralized framework for simultaneous calibration, localization and mapping with multiple LiDARs
- [SC-LIO-SAM](https://github.com/gisbi-kim/SC-LIO-SAM) - LiDAR-inertial SLAM: Scan Context + LIO-SAM.
- [MULLS](https://github.com/YuePanEdward/MULLS) - MULLS: Versatile LiDAR SLAM via Multi-metric Linear Least Square.
- [lili-om](https://github.com/KIT-ISAS/lili-om) - LiLi-OM is a tightly-coupled, keyframe-based LiDAR-inertial odometry and mapping system for both solid-state-LiDAR and conventional LiDARs.
- [PyICP-SLAM](https://github.com/gisbi-kim/PyICP-SLAM) - Full-python LiDAR SLAM using ICP and Scan Context.
- [SC-LeGO-LOAM](https://github.com/irapkaist/SC-LeGO-LOAM) - LiDAR SLAM: Scan Context + LeGO-LOAM.
- [A-LOAM](https://github.com/HKUST-Aerial-Robotics/A-LOAM) - Advanced implementation of LOAM.
- [floam](https://github.com/wh200720041/floam) - Fast LOAM: Fast and Optimized Lidar Odometry And Mapping   for indoor/outdoor localization (Lidar SLAM).
- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam) - 3D LIDAR-based Graph SLAM.
- [lio-mapping](https://github.com/hyye/lio-mapping) - Implementation of Tightly Coupled 3D Lidar Inertial Odometry and Mapping (LIO-mapping).
- [M-LOAM](https://github.com/gogojjh/M-LOAM) - Robust Odometry and Mapping for Multi-LiDAR Systems with Online Extrinsic Calibration.

### 8.2. Camera-based

- [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion) - An optimization-based multi-sensor state estimator
- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) - A Robust and Versatile Monocular Visual-Inertial State Estimator.
- [ov2slam](https://github.com/ov2slam/ov2slam) - OV²SLAM is a Fully Online and Versatile Visual SLAM for Real-Time Applications.
- [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) - ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM.

### 8.3. Lidar-Camera-based

- [r3live](https://github.com/hku-mars/r3live) - A Robust, Real-time, RGB-colored, LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package.
- [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) - LVI-SAM: Tightly-coupled Lidar-Visual-Inertial Odometry via Smoothing and Mapping.
- [CamVox](https://github.com/ISEE-Technology/CamVox) - A low-cost SLAM system based on camera and Livox lidar.

### 8.4. Odometry

- [Faster-LIO](https://github.com/gaoxiang12/faster-lio) - Faster-LIO: Lightweight Tightly Coupled Lidar-inertial Odometry using Parallel Sparse Incremental Voxels.
- [FAST_LIO](https://github.com/hku-mars/FAST_LIO) - A computationally efficient and robust LiDAR-inertial odometry (LIO) package.
- [ESVO](https://github.com/HKUST-Aerial-Robotics/ESVO) - This repository maintains the implementation of "Event-based Stereo Visual Odometry".
- [LOL](https://github.com/RozDavid/LOL) - LOL: Lidar-only Odometry and Localization in 3D point cloud maps.

### 8.5. Registration

- [Quatro](https://github.com/url-kaist/quatro) - Fast and robust global registration for terrestrial robots @ ICRA2022.
- [UnsupervisedR&R](https://github.com/mbanani/unsupervisedrr) - Unsupervised Pointcloud Registration via Differentiable Rendering.
- [3d-icp-cov](https://github.com/CAOR-MINES-ParisTech/3d-icp-cov) - Code for the paper "A New Approach to 3D ICP Covariance Estimation for Mobile Robotics".
- [TEASER-plusplus](https://github.com/MIT-SPARK/TEASER-plusplus) - A fast and robust point cloud registration library.
- [FS3R](https://github.com/zarathustr/FS3R) - A Fast Symbolic 3D Registration Solution from HKUST RAM-LAB.

### 8.6. Optimization

- [voxgraph](https://github.com/ethz-asl/voxgraph) - Voxblox-based Pose graph optimization.
- [Kimera-RPGO](https://github.com/MIT-SPARK/Kimera-RPGO) - Robust Pose Graph Optimization.
- [MOLA](https://github.com/MOLAorg/mola) - A Modular Optimization framework for Localization and mapping.

### 8.7. Loop Closing

- [scancontext](https://github.com/irapkaist/scancontext) - Global LiDAR descriptor for place recognition and long-term localization.
- [OverlapNet](https://github.com/PRBonn/OverlapNet) - OverlapNet - Loop Closing for 3D LiDAR-based SLAM (chen2020rss).

### 8.8. Other

- [evo](https://github.com/MichaelGrupp/evo) - Python package for the evaluation of odometry and SLAM.
- [libRSF](https://github.com/TUC-ProAut/libRSF) - A robust sensor fusion library for online localization.
- [interactive_slam](https://github.com/SMRT-AIST/interactive_slam) - Interactive Map Correction for 3D Graph SLAM.
- [slam_docker_collection](https://github.com/koide3/slam_docker_collection) - A collection of docker environments for 3D SLAM packages.
- [removert](https://github.com/irapkaist/removert) - Remove then revert (IROS 2020).



## 9. GPS, IMU

- [imu_tools](https://github.com/ccny-ros-pkg/imu_tools) - ROS tools for IMU devices.
- [PPPLib](https://github.com/heiwa0519/PPPLib) - Precise Point Positioning Library.
- [RTKLIB](https://github.com/tomojitakasu/RTKLIB) - RTKLIB is an open source program package for standard and precise positioning.
with GNSS (global navigation satellite system).
- [gnss-ins-sim](https://github.com/Aceinna/gnss-ins-sim) - Open-source GNSS + inertial navigation, sensor fusion simulator. Motion trajectory generator, sensor models, and navigation.
- [rtklib_ros_bridge](https://github.com/MapIV/rtklib_ros_bridge) - Rtklib_ros_bridge is a package that outputs the latitude and longitude, satellite reception status, altitude, ecef xyz, ecef velocity vector, and Time of Week (GPS Time) calculated by RTKLIB as ROS messages.
- [ntpd_driver](https://github.com/vooon/ntpd_driver) - This node convert ROS TimeReference message to ntpd-shm format.
- [eagleye](https://github.com/MapIV/eagleye) - Precise localization based on GNSS and IMU.
- [GPS_IMU_Kalman_Filter](https://github.com/karanchawla/GPS_IMU_Kalman_Filter) - Fusing GPS, IMU and Encoder sensors for accurate state estimation.
- [imu_utils](https://github.com/gaowenliang/imu_utils) - A ROS package tool to analyze the IMU performance.

## 10. Dataset

- [Argoverse 2 dataset](https://github.com/argoai/av2-api) - Official GitHub repository for the Argoverse 2 family of datasets.
- [kapture](https://github.com/naver/kapture) - Kapture is a file format as well as a set of tools for manipulating datasets, and in particular Visual Localization and Structure from Motion data.
- [UrbanLoco](https://github.com/weisongwen/UrbanLoco) - UrbanLoco: A Full Sensor Suite Dataset for Mapping and Localization in Urban Scenes.
- [UrbanNavDataset](https://github.com/weisongwen/UrbanNavDataset) - UrbanNav: an Open-Sourcing Localization Data Collected in Asian Urban Canyons, Including Tokyo and Hong Kong.
- [mapping_dataset_styria](https://github.com/ARTI-Robots/mapping_dataset_styria) - Open mapping dataset of styria in Austria.

## 11. Simulation

- [AWSIM](https://github.com/tier4/AWSIM) - Open source simulator for self-driving vehicles.
- [carla](https://github.com/carla-simulator/carla) - Open-source simulator for autonomous driving research.
- [autocore_sim](https://github.com/autowarefoundation/autocore_sim) - A ROS1/ROS2 Multi-robot Simulator for Autoware.
- [highway-env](https://github.com/eleurent/highway-env) - A minimalist environment for decision-making in autonomous driving.
- [LGSVL](https://github.com/lgsvl/simulator) - A ROS/ROS2 Multi-robot Simulator for Autonomous Vehicles.

## 12. Visualization

- [rosshow](https://github.com/dheera/rosshow) - Visualize ROS topics inside a terminal with Unicode/ASCII art.
- [rosbag_fancy](https://github.com/xqms/rosbag_fancy) - Fancy terminal UI for rosbag.
- [zethus](https://github.com/rapyuta-robotics/zethus) - Realtime robot data visualization in the browser.
- [webviz](https://github.com/cruise-automation/webviz) - Web-based visualization libraries.
- [Foxglove Studio](https://github.com/foxglove/studio) - Foxglove Studio (foxglove.dev) is an integrated visualization and diagnosis tool for robotics.
- [3D-Detection-Tracking-Viewer](https://github.com/hailanyi/3D-Detection-Tracking-Viewer) - 3D detection and tracking viewer (visualization) for kitti & waymo dataset.
- [rviz_satellite](https://github.com/nobleo/rviz_satellite) - Display internet satellite imagery in RViz.
- [mapviz](https://github.com/swri-robotics/mapviz) - Modular ROS visualization tool for 2D data.
- [streetscape.gl](https://github.com/uber/streetscape.gl) - Visualization framework for autonomy and robotics data encoded in XVIZ.
- [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools) - C++ API wrapper for displaying shapes and meshes in Rviz.
- [PlotJuggler](https://github.com/facontidavide/PlotJuggler) - The Time Series Visualization Tool that you deserve.

## 13. Web

- [rosboard](https://github.com/dheera/rosboard) - ROS node that turns your robot into a web server to visualize ROS topics.
- [webrtc_ros](https://github.com/RobotWebTools/webrtc_ros) - Streaming of ROS Image Topics using WebRTC.
- [AutomanTools](https://github.com/tier4/AutomanTools) - AutomanTools is an open-source software for self-driving AI.
- [Rosbag-Analyser](https://github.com/agrija9/Rosbag-Analyser) - Interactive Post-mortem/Live Analysis of ROS Bag Files.


## 14. Study

- [visual-slam-roadmap](https://github.com/changh95/visual-slam-roadmap) - Roadmap to becoming a Visual-SLAM developer in 2021.
- [RosHowTwo](https://github.com/rydb/RosHowTwo) - A place to find and share ROS2 learning resources!
- [Eigen-Cheatsheet](https://github.com/zxl19/Eigen-Cheatsheet) - A cheatsheet of Eigen, the C++ linear algebra library.
- [slambook2](https://github.com/gaoxiang12/slambook2) - Edition 2 of the slambook.
- [CppRobotics](https://github.com/onlytailei/CppRobotics) - Cpp implementation of robotics algorithms including localization, mapping, SLAM, path planning and control.
- [self-driving-car](https://github.com/udacity/self-driving-car) - The Udacity open source self-driving car project.
- [Dig-into-Apollo](https://github.com/daohu527/Dig-into-Apollo) - Apollo notes - Apollo learning notes for beginners.
- [Autoware_tutorial](https://github.com/pixmoving-moveit/Autoware_tutorial) - Autoware Simple Tutorial for Self-driving Car.

## 15. Other Tool

- [PoseLib](https://github.com/vlarsson/PoseLib) - Minimal solvers for calibrated camera pose estimation
- [ros2-for-unity](https://github.com/RobotecAI/ros2-for-unity) - High-performance ROS2 solution for Unity3D.
- [semantic-segmentation-editor](https://github.com/Hitachi-Automotive-And-Industry-Lab/semantic-segmentation-editor) - Web labeling tool for bitmap images and point clouds.
- [MegBA](https://github.com/MegviiRobot/MegBA) - MegBA: A Distributed High-Performance Library for Large-Scale Bundle Adjustment with GPUs.
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree) - This repository provides implementation of an incremental k-d tree for robotic applications.
- [ros_msft_onnx](https://github.com/ms-iot/ros_msft_onnx) - ONNX Runtime for the Robot Operating System (ROS), works on ROS1 and ROS2.
- [catkin_virtualenv](https://github.com/locusrobotics/catkin_virtualenv) - Bundle python requirements in a catkin package via virtualenv.
- [pal_statistics](https://github.com/pal-robotics/pal_statistics) - The PAL Statistics Framework provides a way of gathering, aggregating, storing and visualizing statistics from arbitrary sources in a flexible and real-time safe way in ROS.
- [CARLA Autonomous Driving Leaderboard](https://leaderboard.carla.org/) - The main goal of the CARLA Autonomous Driving Leaderboard is to evaluate the driving proficiency of autonomous agents in realistic traffic situations.
- [PUMA](https://github.com/PRBonn/puma) - Poisson Surface Reconstruction for LiDAR Odometry and Mapping.
- [RoboStack for ROS Noetic](https://github.com/RoboStack/ros-noetic) - RoboStack, which tightly couples ROS with Conda, a cross-platform, language-agnostic package manager.
- [modelzoo](https://github.com/autowarefoundation/modelzoo) - A collection of machine-learned models for use in autonomous driving applications.
- [ROSIntegration](https://github.com/code-iai/ROSIntegration) - Unreal Engine Plugin to enable ROS Support.
- [jupyter-ros](https://github.com/RoboStack/jupyter-ros) - Jupyter widget helpers for ROS, the Robot Operating System.
- [jupyterlab-ros](https://github.com/RoboStack/jupyterlab-ros) - Helper extensions for running ROS from within JupyterLab.
- [rosbag_editor](https://github.com/facontidavide/rosbag_editor) - Create a rosbag from a given one, using a simple GUI.
- [laser_line_extraction](https://github.com/kam3k/laser_line_extraction) - A ROS package that extracts line segments from LaserScan messages.
- [python-pcl](https://github.com/strawlab/python-pcl) - Python bindings to the pointcloud library (pcl).
- [ros_deep_learning](https://github.com/dusty-nv/ros_deep_learning) - Deep learning inference nodes for ROS with support for NVIDIA Jetson TX1/TX2/Xavier and TensorRT.
- [imgui_ros](https://github.com/lucasw/imgui_ros) - View ros images, visualize in 2D and 3D, and interact with nodes through topics and services using ocornut/imgui.
- [ROS-Mobile-Android](https://github.com/ROS-Mobile/ROS-Mobile-Android) - Visualization and controlling application for Android.
- [PPTK](https://github.com/heremaps/pptk) - The Point Processing Toolkit (pptk) is a Python package for visualizing and processing 2-d/3-d point clouds.


## 16. Other Awesome

- [/awesome-slam-datasets](https://github.com/youngguncho/awesome-slam-datasets) - A curated list of awesome datasets for SLAM.
- [ICRA2021-SLAM-paper-list](https://github.com/MichaelWang1028/ICRA2021-SLAM-paper-list) - ICRA2021-SLAM-paper-list.
- [Lidar_For_AD_references](https://github.com/beedotkiran/Lidar_For_AD_references) - A list of references on lidar point cloud processing for autonomous driving.
- [Recent_SLAM_Research](https://github.com/YiChenCityU/Recent_SLAM_Research) - Track Advancement of SLAM 跟踪SLAM前沿动态【2020 version】.
- [Awesome-3D-Detectors](https://github.com/Hub-Tian/Awesome-3D-Detectors) - Paperlist of awesome 3D detection methods.
- [awesome-pointcloud-processing](https://github.com/Tom-Hardy-3D-Vision-Workshop/awesome-pointcloud-processing) - Awesome PointCloud processing algorithm.
- [awesome-robotic-tooling](https://github.com/protontypes/awesome-robotic-tooling) - Tooling for professional robotic development in C++ and Python with a touch of ROS, autonomous driving and aerospace.
- [awesome-data-labeling](https://github.com/heartexlabs/awesome-data-labeling) - A curated list of awesome data labeling tools.
- [Awesome-Autonomous-Driving](https://github.com/autonomousdrivingkr/Awesome-Autonomous-Driving) - Survey for Autonomous Driving Papers.
- [awesome-autonomous-vehicles](https://github.com/manfreddiaz/awesome-autonomous-vehicles) - Curated List of Self-Driving Cars and Autonomous Vehicles Resources.
- [awesome-point-cloud-processing](https://github.com/mmolero/awesome-point-cloud-processing) - A curated list of awesome Point Cloud Processing Resources, Libraries, Software.
- [awesome-point-cloud-analysis](https://github.com/Yochengliu/awesome-point-cloud-analysis) - A list of papers and datasets about point cloud analysis (processing).

## 17. For Chinese

- [slam_in_autonomous_driving](https://github.com/gaoxiang12/slam_in_autonomous_driving) - 《自动驾驶中的SLAM技术》对应开源代码.
- [LOAM_NOTED](https://github.com/cuitaixiang/LOAM_NOTED) - loam code noted in Chinese（loam中文注解版）.
- [LeGO-LOAM_NOTED](https://github.com/wykxwyc/LeGO-LOAM_NOTED) - LeGO-LOAM代码注释与学习.
- [loam_leanring](https://github.com/shoufei403/loam_leanring) - Include loam_velodyne code and A-LOAM code with chinese explaintion.I also put some papers related to loam in it.
- [UpdatingHDmapByMonoCamera](https://github.com/JokerJohn/UpdatingHDmapByMonoCamera) - A hdmap  updating project by mono camera and gps.
- [awesome-autonomous-vehicle](https://github.com/DeepTecher/awesome-autonomous-vehicle) - 无人驾驶的资源列表中文版.
- [Apollo_learning](https://github.com/lucianzhong/Apollo_learning) - Apollo 学习笔记.
- [python-parallel-programming-cookbook-cn](https://github.com/laixintao/python-parallel-programming-cookbook-cn) - 📖《Python Parallel Programming Cookbook》中文版.
- [SLAM-BOOK](https://github.com/yanyan-li/SLAM-BOOK) - 这是一本关于SLAM的书稿，希望能清楚的介绍SLAM系统中的使用的几何方法和深度学习方法。书稿最后应该会达到400页左右，书稿每章对应的代码也会被整理出来.
- [surround-view-system-introduction](https://github.com/neozhaoliang/surround-view-system-introduction) - 一份简单的环视系统制作实现，包含完整的标定、投影、拼接和实时运行流程.
- [opensource_slam_noted](https://github.com/JokerJohn/opensource_slam_noted) - Open source slam system  notes.
- [Statistical-Learning-Method_Code](https://github.com/Dod-o/Statistical-Learning-Method_Code) - 手写实现李航《统计学习方法》书中全部算法.
- [DeepLearning](https://github.com/MingchaoZhu/DeepLearning) - Python for《Deep Learning》，该书为《深度学习》(花书) 数学推导、原理剖析与源码级别代码实现.
- [localization_in_auto_driving](https://github.com/Little-Potato-1990/localization_in_auto_driving) - 从零开始做自动驾驶定位.
- [Effective-Modern-Cpp-Zh](https://github.com/Ricardo666666/Effective-Modern-Cpp-Zh) - 42 SPECIFIC WAYS TO IMPROVE YOUR USE OF C++11 AND C++14 中文版.
- [PCL-Notes](https://github.com/MNewBie/PCL-Notes) - Pcl learning notes.
