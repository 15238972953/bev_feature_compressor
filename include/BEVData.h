#pragma once
#include <eigen3/Eigen/Dense>
#include <cstdint>
#include <vector>
#include <array>

// 前置声明：传感器状态枚举（便于缓存决策）
enum class SensorHealth : uint8_t {
    NORMAL = 0,    // 传感器正常
    DEGRADED = 1,  // 传感器性能降级
    FAULT = 2      // 传感器故障
};

// 特征图元数据（压缩算法依赖）
struct BEVFeatureMeta {
    uint32_t rows;               // 特征图行数（如256）
    uint32_t cols;               // 特征图列数（如256）
    float value_min;             // 特征值最小值（如-1.0f，用于量化）
    float value_max;             // 特征值最大值（如1.0f，用于量化）
    uint8_t channel;             // 特征通道（单通道为0，多通道场景扩展）
    bool is_normalized;          // 是否已归一化（压缩算法分支选择依据）
};

// 传感器上下文（缓存决策依赖）
struct SensorContext {
    float ego_speed;             // 本车速度（m/s，高速场景特征变化快，缓存策略调整）
    SensorHealth health;         // 传感器健康状态（故障数据不缓存）
    std::array<float, 3> ego_pose; // 本车位置（x,y,yaw，用于空间关联性缓存）
};

// 核心输入数据结构
struct BEVFeaturePacket {
    Eigen::MatrixXf feature;     // 原始BEV特征图（浮点矩阵，核心数据）
    BEVFeatureMeta feature_meta; // 特征图元数据（压缩算法参数）
    SensorContext sensor_ctx;    // 传感器上下文（缓存策略参数）
    uint64_t timestamp;          // 纳秒级Unix时间戳（核心：时序排序与缓存淘汰）
};