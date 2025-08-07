# Colmap-PCD 详细使用文档

## 1. 项目概述

### 1.1 项目简介
Colmap-PCD 是一个开源的精细图像到点云配准工具。该软件可以提取和匹配图像对之间以及图像到点云之间的特征。匹配的特征被表述为因子图优化问题中的约束，该问题求解相机姿态以及三维重建特征。处理开始时需要一个大致已知的初始图像相机姿态。

### 1.2 核心功能
- **特征提取与匹配**：提取图像特征并进行图像间和图像到点云的特征匹配
- **因子图优化**：使用图优化方法同时优化相机姿态和三维特征点
- **点云配准**：精细的图像到点云配准，支持激光雷达点云数据
- **多视图几何**：基于多视图几何的三维重建
- **GUI界面**：提供用户友好的图形界面

### 1.3 技术特点
- 基于原始 [Colmap](https://colmap.github.io) 图像三维重建软件开发
- 属于 [CMU-Recon System](https://www.cmu-reconstruction.com) 项目的一部分
- 支持激光雷达点云数据处理
- 支持多种相机模型（包括OpenCV模型）
- 提供命令行和GUI两种操作方式

## 2. 系统要求

### 2.1 操作系统
- Ubuntu 20.04（推荐）
- Ubuntu 22.04

### 2.2 硬件要求
- CPU：多核处理器（推荐）
- GPU：支持CUDA的Nvidia GPU（强烈推荐，可显著提升性能）
- 内存：8GB以上（取决于数据集大小）
- 存储：足够存储图像、点云和重建结果的空间

## 3. 安装配置

### 3.1 依赖项安装

```bash
sudo apt-get install \
    git \
    cmake \
    build-essential \
    libboost-program-options-dev \
    libboost-filesystem-dev \
    libboost-graph-dev \
    libboost-system-dev \
    libboost-test-dev \
    libeigen3-dev \
    libsuitesparse-dev \
    libfreeimage-dev \
    libmetis-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libglew-dev \
    qtbase5-dev \
    libqt5opengl5-dev \
    libcgal-dev \
    libcgal-qt5-dev \
    libatlas-base-dev \
    libsuitesparse-dev
```

### 3.2 Ceres Solver 安装
安装最新稳定版本的 [Ceres Solver](http://ceres-solver.org)。下载并编译 'ceres-solver-2.1.0.tar.gz'：

```bash
# 下载Ceres Solver
wget http://ceres-solver.org/ceres-solver-2.1.0.tar.gz
tar -xzf ceres-solver-2.1.0.tar.gz
cd ceres-solver-2.1.0

# 编译安装
mkdir build
cd build
cmake ..
make -j
sudo make install
```

### 3.3 CUDA 安装（可选但强烈推荐）
对于配备Nvidia GPU的计算机，强烈推荐安装 [CUDA](https://docs.nvidia.com/cuda)。

### 3.4 编译Colmap-PCD

```bash
# 克隆代码库
git clone https://github.com/XiaoBaiiiiii/colmap-pcd.git
cd colmap-pcd

# 创建构建目录
mkdir build
cd build

# 配置和编译
cmake ..
make -j

# 安装
sudo make install
```

## 4. 快速开始

### 4.1 启动GUI界面

```bash
colmap gui
```

### 4.2 下载示例数据集
下载 [Smith Hall Outdoor Dataset](https://drive.google.com/drive/folders/1P1J9cEWSSFCL_XmHSVYfuWgtmcgAWprB)。
对于快速测试，只需要：
- `25_images.zip`
- `intrinsics.txt`
- `pointcloud_with_norm.ply`

对于完整测试，还需要：
- `450_images.zip`

### 4.3 数据准备
1. 解压 `25_images.zip`
2. 确保点云文件 `pointcloud_with_norm.ply` 可用
3. 查看 [指导视频](https://youtu.be/TuX8tCmJCC8) 了解如何加载数据集到Colmap-PCD

### 4.4 处理流程
1. 启动Colmap-PCD GUI
2. 加载图像数据集
3. 加载点云数据（PLY格式）
4. 设置相机内参
5. 开始处理
6. 查看结果

## 5. 高级用法

### 5.1 点云准备
**推荐使用 [CloudCompare](https://www.danielgm.net/cc) 准备点云：**
- 将点云下采样到3-5cm分辨率
- 使用10-20cm半径计算法向量
- 保存为PLY格式（二进制或ASCII）
- 确保坐标系统为：x-前，y-左，z-上

### 5.2 设置初始相机姿态
默认情况下，初始图像的相机姿态设置为点云原点，无旋转。如需自定义：

1. 点击 `Reconstruction->Reconstruction options`
2. 在 `Init` 标签页中设置：
   - `init_image_x [m]`：X坐标
   - `init_image_y [m]`：Y坐标
   - `init_image_z [m]`：Z坐标
   - `init_image_roll [deg]`：滚转角
   - `init_image_pitch [deg]`：俯仰角
   - `init_image_yaw [deg]`：偏航角

**注意**：相机姿态遵循与点云相同的坐标约定（x-前，y-左，z-上）

### 5.3 保存相机姿态
处理完成后保存相机姿态：

1. 点击 `Reconstruction->Reconstruction options`
2. 在 `Lidar` 标签页中设置 `save_image_pose_folder`
3. 点击保存按钮生成 `pose.ply` 文件

### 5.4 加载相机姿态先验
使用已有的相机姿态作为先验：

1. 处理开始前，点击 `Reconstruction->Reconstruction options`
2. 在 `Lidar` 标签页中：
   - 勾选 `if_import_image_pose_prior`
   - 设置 `image_pose_prior_path` 指向姿态文件

### 5.5 参数调优
在 `Reconstruction->Reconstruction options` 的 `Lidar` 标签页中调整参数：

**约束权重参数：**
- `depth_proj_constraint_weight`：初始关联中图像到点云约束的权重
- `icp_nonground_constraint_weight`：非地面点最终关联权重
- `icp_ground_constraint_weight`：地面点最终关联权重

**距离阈值参数：**
- `min_depth_proj_dist` / `max_depth_proj_dist`：初始关联的最小/最大距离
- `kdtree_max_search_dist` / `kdtree_min_search_dist`：最终关联的起始/结束距离阈值
- `kdtree_search_dist_drop`：距离阈值下降间隔

### 5.6 精细配准
处理完成后可选择进行批量优化以精细化配准：

1. 点击 `Reconstruction->Bundle adjustment`
2. 点击 `Run` 按钮
3. 可选择同时优化相机内参：
   - 勾选 `refine_focal_length`
   - 勾选 `refine_prinpical_point`
   - 勾选 `refine_extra_params`

## 6. 命令行使用

### 6.1 基本命令结构
```bash
colmap [command] [options]
```

### 6.2 主要命令
- `colmap gui`：启动图形界面
- `colmap feature_extractor`：特征提取
- `colmap exhaustive_matcher`：特征匹配
- `colmap mapper`：三维重建
- `colmap automatic_reconstructor`：自动重建

### 6.3 自动重建示例
```bash
# 设置数据集路径
DATASET_PATH=/path/to/project

# 自动重建
colmap automatic_reconstructor \
    --workspace_path $DATASET_PATH \
    --image_path $DATASET_PATH/images
```

### 6.4 手动流程示例
```bash
# 设置数据集路径
DATASET_PATH=/path/to/dataset

# 特征提取
colmap feature_extractor \
   --database_path $DATASET_PATH/database.db \
   --image_path $DATASET_PATH/images

# 特征匹配
colmap exhaustive_matcher \
   --database_path $DATASET_PATH/database.db

# 创建输出目录
mkdir $DATASET_PATH/sparse

# 三维重建
colmap mapper \
    --database_path $DATASET_PATH/database.db \
    --image_path $DATASET_PATH/images \
    --output_path $DATASET_PATH/sparse
```

## 7. 数据集

### 7.1 可用数据集

**NSH Atrium Indoor Dataset**
- [下载链接](https://drive.google.com/drive/folders/1Dmpv91aFBUZfIa1Sh23T0a_Z4PKvsfHN)
- 450张室内图像配准

**NSH Patio Outdoor Dataset**
- [下载链接](https://drive.google.com/drive/folders/1mUs4eRK1aGXgNui4wRt0KaLBX7qXGD-q)
- 450张室外图像配准
- 提高初始化鲁棒性：设置 `init_image_id2 = 2`

**Smith Hall Outdoor Dataset**
- [下载链接](https://drive.google.com/drive/folders/1P1J9cEWSSFCL_XmHSVYfuWgtmcgAWprB)
- 25张图像（快速测试）
- 450张图像（完整测试）

### 7.2 数据格式要求

**图像要求：**
- 支持常见图像格式（JPG、PNG等）
- 推荐有重叠的多视角图像
- 良好的纹理和光照条件

**点云要求：**
- PLY格式（二进制或ASCII）
- 包含法向量信息
- 坐标系：x-前，y-左，z-上
- 推荐分辨率：3-5cm

## 8. 故障排除

### 8.1 常见问题

**Q1：编译时找不到依赖项**
A1：确保所有依赖项正确安装，特别是Ceres Solver

**Q2：处理失败或结果不佳**
A2：检查以下因素：
- 初始相机姿态设置是否正确
- 点云质量和坐标系是否正确
- 图像质量和重叠度是否足够

**Q3：GPU加速不工作**
A3：确保CUDA正确安装且GPU支持

### 8.2 性能优化建议
- 使用GPU加速（需要CUDA）
- 适当调整图像分辨率
- 根据数据集大小调整处理参数

## 9. 引用

如果您使用此代码或数据集，请引用我们的[论文](https://arxiv.org/abs/2310.05504)。

## 10. 许可证

本项目基于BSD许可证开源。

## 11. 作者

- [Chunge Bai](https://github.com/XiaoBaiiiiii)
- [Ruijie Fu](https://ruijiefu.com)
- [Ji Zhang](https://frc.ri.cmu.edu/~zhangji)

## 12. 致谢

本项目基于原始的 [Colmap](https://colmap.github.io) 开发。