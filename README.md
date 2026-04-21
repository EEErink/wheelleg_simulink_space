# Wheel-Leg Robot Dynamics & LQR Control

轮腿机器人的完整动力学推导、线性化分析及LQR控制器设计

[English](#english) | [中文](#中文)

---

## 中文

### 📋 项目概述

本项目完成了轮腿机器人的完整动力学推导、线性化分析以及LQR控制器设计。包含详细的数学推导、MATLAB实现和C代码集成示例。

**机器人模型**：
- 结构：双轮差速驱动 + 双腿（左右独立）
- 自由度：5个广义坐标
- 控制输入：4个力矩（左右轮电机 + 左右髋关节电机）

### 🗂️ 项目结构

```
wheel_leg2026/
├── README.md                                    # 本文件
├── wheelleg_config.txt                          # 机器人配置参数
│
├── HerKules 2026 轮腿建模&LQR推导/              # ⭐ 主要工作目录
│   ├── compute_lqr.m                            # ★ LQR控制器计算（主脚本）
│   ├── simplify_dynamics_v2.m                   # 动力学方程推导与化简
│   ├── apply_kinematics_v2.m                    # 代入运动学约束
│   ├── linearize_system_v2.m                    # 系统线性化
│   ├── get_K_from_leg_lengths.m                 # 根据腿长获取K矩阵
│   └── *.mat                                    # 计算结果文件
│
├── HerKules 2024/                               # 2024年版本（旧方案）
│
├── MIRAI仿真/                                   # MIRAI平台仿真脚本
│   ├── MIRAI_K_cal_v1.m                         # K矩阵计算
│   └── get_k_from_mirai.m                       # 从仿真提取K矩阵
│
├── MIRAI轮腿K矩阵计算/                          # MIRAI平台K矩阵计算
│   ├── MIRAI_K_cal_v1.m                         # K矩阵计算脚本
│   ├── get_A_length.m                           # 腿长相关计算
│   └── Func_Cal_ABCD_Array.m                    # A、B、C、D矩阵计算函数
│
└── 51U_K矩阵计算/                               # 51U平台K矩阵计算
    ├── k_cal_51.m                               # 51U平台K矩阵计算
    └── LQR_K_WBR.m                              # LQR增益计算
```

### 🔧 工作流程

#### Step 1: 动力学方程推导
基于牛顿-欧拉方法建立各刚体的动力学方程，得到5个最终动力学方程。

#### Step 2: 代入运动学约束
将运动学约束代入动力学方程，整理为矩阵形式，提取质量矩阵M、控制矩阵B和重力项g。

#### Step 3: 线性化
在平衡点（直立静止）处线性化，得到线性状态空间模型。

**状态向量**（10维）：机体位置、速度、偏航角、角速度、左腿角度、角速度、右腿角度、角速度、俯仰角、角速度

#### Step 4: LQR控制器设计
计算LQR增益矩阵K，通过加权状态和控制输入来最小化系统能耗，得到最优控制律。

### 🚀 快速开始

#### 单腿长LQR计算

```bash
cd "HerKules 2026 轮腿建模&LQR推导"
matlab -batch "run('compute_lqr.m')"
```

1. 打开 `compute_lqr.m`
2. 修改物理参数（Step 2区域）
3. 修改Q、R权重矩阵（Step 3区域）
4. 确保 `enable_fitting = false`
5. 运行脚本

**输出**：
- 控制台打印K矩阵（可直接复制到C代码）
- `lqr_results.mat` 保存完整结果

#### 变腿长拟合

1. 设置 `enable_fitting = true`
2. 修改 `Leg_data` 数据集（不同腿长对应的参数）
3. 运行脚本

**输出**：
- `lqr_fitting_results.mat` 包含拟合系数
- 拟合公式：$K_{ij}(l_l, l_r) = p_{00} + p_{10} l_l + p_{01} l_r + p_{20} l_l^2 + p_{11} l_l l_r + p_{02} l_r^2$

### 📊 关键参数

| 参数 | 符号 | 单位 | 示例值 |
|:---|:---:|:---:|:---:|
| 轮子半径 | $R$ | m | 0.055 |
| 轮距/2 | $R_w$ | m | 0.225 |
| 机体质量 | $m_b$ | kg | 15.564 |
| 机体转动惯量 | $I_b$ | kg·m² | 0.212 |
| 腿长 | $l_{leg}$ | m | 0.13~0.40 |
| 腿质量 | $m_{leg}$ | kg | 1.205 |

### 💻 C代码集成

#### 固定腿长

```c
// LQR增益矩阵 K[4][10]
// 控制律: u = -K * x
float K[4][10] = {
    {-0.278925f, -1.03799f, -0.0997239f, -0.129885f, -0.11638f, -0.057609f, 0.253744f, -0.0369642f, -4.76675f, -0.39819f},
    {-0.278925f, -1.03799f, 0.0997239f, 0.129885f, 0.253744f, -0.0369642f, -0.11638f, -0.057609f, -4.76675f, -0.39819f},
    {-1.05359f, -3.93452f, -0.0525088f, -0.0672831f, -7.58634f, 0.403738f, -27.1721f, -0.81588f, 73.3285f, 6.43686f},
    {-1.05359f, -3.93452f, 0.0525088f, 0.0672831f, -27.1721f, -0.81588f, -7.58634f, 0.403738f, 73.3285f, 6.43686f}
};

// 状态向量
float x[10] = {X, dX, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b};

// 计算控制力矩
float u[4];
for (int i = 0; i < 4; i++) {
    u[i] = 0;
    for (int j = 0; j < 10; j++) {
        u[i] -= K[i][j] * x[j];
    }
}
```

#### 变腿长（使用拟合）

```c
// 根据腿长计算K矩阵
float K[4][10];
for (int n = 0; n < 40; n++) {
    int row = n / 10;
    int col = n % 10;
    K[row][col] = K_coef[n][0] 
                + K_coef[n][1] * l_l 
                + K_coef[n][2] * l_r
                + K_coef[n][3] * l_l * l_l 
                + K_coef[n][4] * l_l * l_r 
                + K_coef[n][5] * l_r * l_r;
}
```

### ⚠️ 注意事项

1. **可控性**: 系统可控性矩阵秩为8（而非10），因为位置和yaw角是可积分状态。这是正常的，LQR仍能稳定系统。

2. **平衡点**: 线性化假设在平衡点附近（$\theta_l = \theta_r = \theta_b = 0$），大角度偏离时可能需要增益调度。

3. **参数一致性**: 确保MATLAB中的参数与实际机器人一致，特别是腿部惯量和质心位置随腿长的变化。

### 📚 参考资源

- [2023上交轮腿开源](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22756)
- [哈工程轮腿开源](https://zhuanlan.zhihu.com/p/563048952)

### 📝 更新日志

- **2026/01/09**: 完成新动力学模型推导，实现LQR控制器计算和腿长拟合功能

---

## English

### 📋 Overview

This project provides a complete dynamics derivation, linearization analysis, and LQR controller design for a wheel-leg robot. It includes detailed mathematical derivations, MATLAB implementations, and C code integration examples.

**Robot Model**:
- Structure: Differential drive wheels + dual legs (left/right independent)
- DOF: 5 generalized coordinates
- Control inputs: 4 torques (left/right wheel motors + left/right hip motors)

### 🗂️ Directory Structure

```
wheel_leg2026/
├── README.md                                    # This file
├── wheelleg_config.txt                          # Robot configuration parameters
│
├── HerKules 2026 轮腿建模&LQR推导/              # ⭐ Main working directory
│   ├── compute_lqr.m                            # ★ LQR controller computation (main script)
│   ├── simplify_dynamics_v2.m                   # Dynamics equation derivation & simplification
│   ├── apply_kinematics_v2.m                    # Apply kinematics constraints
│   ├── linearize_system_v2.m                    # System linearization
│   ├── get_K_from_leg_lengths.m                 # Get K matrix from leg lengths
│   └── *.mat                                    # Computation result files
│
├── HerKules 2024/                               # 2024 version (legacy approach)
│
├── MIRAI仿真/                                   # MIRAI platform simulation scripts
│   ├── MIRAI_K_cal_v1.m                         # K matrix computation
│   └── get_k_from_mirai.m                       # Extract K matrix from simulation
│
├── MIRAI轮腿K矩阵计算/                          # MIRAI platform K matrix computation
│   ├── MIRAI_K_cal_v1.m                         # K matrix calculation script
│   ├── get_A_length.m                           # Leg length related calculations
│   └── Func_Cal_ABCD_Array.m                    # A, B, C, D matrix calculation functions
│
└── 51U_K矩阵计算/                               # 51U platform K matrix computation
    ├── k_cal_51.m                               # 51U platform K matrix calculation
    └── LQR_K_WBR.m                              # LQR gain calculation
```

### 🔧 Workflow

#### Step 1: Dynamics Equation Derivation
Establish dynamics equations for each rigid body using Newton-Euler method, resulting in 5 final dynamics equations.

#### Step 2: Apply Kinematics Constraints
Substitute kinematics constraints into dynamics equations and organize into matrix form to extract mass matrix M, control matrix B, and gravity term g.

#### Step 3: Linearization
Linearize at equilibrium point (upright and stationary) to obtain linear state-space model.

**State vector** (10-dimensional): body position, velocity, yaw angle, angular velocity, left leg angle, angular velocity, right leg angle, angular velocity, pitch angle, angular velocity

#### Step 4: LQR Controller Design
Compute LQR gain matrix K by weighting states and control inputs to minimize system energy consumption and obtain optimal control law.

### 🚀 Quick Start

#### Single Leg Length LQR Computation

```bash
cd "HerKules 2026 轮腿建模&LQR推导"
matlab -batch "run('compute_lqr.m')"
```

1. Open `compute_lqr.m`
2. Modify physical parameters (Step 2 section)
3. Modify Q, R weight matrices (Step 3 section)
4. Ensure `enable_fitting = false`
5. Run the script

**Output**:
- K matrix printed to console (can be directly copied to C code)
- `lqr_results.mat` saves complete results

#### Variable Leg Length Fitting

1. Set `enable_fitting = true`
2. Modify `Leg_data` dataset (parameters for different leg lengths)
3. Run the script

**Output**:
- `lqr_fitting_results.mat` contains fitting coefficients
- Fitting formula: $K_{ij}(l_l, l_r) = p_{00} + p_{10} l_l + p_{01} l_r + p_{20} l_l^2 + p_{11} l_l l_r + p_{02} l_r^2$

### 📊 Key Parameters

| Parameter | Symbol | Unit | Example |
|:---|:---:|:---:|:---:|
| Wheel radius | $R$ | m | 0.055 |
| Half track width | $R_w$ | m | 0.225 |
| Body mass | $m_b$ | kg | 15.564 |
| Body inertia | $I_b$ | kg·m² | 0.212 |
| Leg length | $l_{leg}$ | m | 0.13~0.40 |
| Leg mass | $m_{leg}$ | kg | 1.205 |

### 💻 C Code Integration

#### Fixed Leg Length

```c
// LQR gain matrix K[4][10]
// Control law: u = -K * x
float K[4][10] = {
    {-0.278925f, -1.03799f, -0.0997239f, -0.129885f, -0.11638f, -0.057609f, 0.253744f, -0.0369642f, -4.76675f, -0.39819f},
    // ... more rows
};

// State vector
float x[10] = {X, dX, phi, dphi, theta_l, dtheta_l, theta_r, dtheta_r, theta_b, dtheta_b};

// Compute control torques
float u[4];
for (int i = 0; i < 4; i++) {
    u[i] = 0;
    for (int j = 0; j < 10; j++) {
        u[i] -= K[i][j] * x[j];
    }
}
```

### ⚠️ Important Notes

1. **Controllability**: System controllability matrix rank is 8 (not 10) because position and yaw angle are integrable states. This is normal; LQR can still stabilize the system.

2. **Equilibrium Point**: Linearization assumes operation near equilibrium point ($\theta_l = \theta_r = \theta_b = 0$). Large angle deviations may require gain scheduling.

3. **Parameter Consistency**: Ensure MATLAB parameters match actual robot, especially leg inertia and center of mass position variations with leg length.

### 📚 References

- [2023 SJTU Wheel-Leg Open Source](https://bbs.robomaster.com/forum.php?mod=viewthread&tid=22756)
- [Harbin Engineering Wheel-Leg Open Source](https://zhuanlan.zhihu.com/p/563048952)

### 📝 Changelog

- **2026/01/09**: Completed new dynamics model derivation, implemented LQR controller computation and leg length fitting functionality

---

## License

This project is provided as-is for educational and research purposes.

## Contact

For questions or issues, please open an issue on GitHub.
