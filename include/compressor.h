#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>

class BEVCompressor {
public:
    struct Config {
        int block_size = 16;          // 分块大小
        float compression_ratio = 5.0f; // 目标压缩比
        bool lossless = false;        // 无损模式开关
    };

    explicit BEVCompressor(const Config& config);
    
    // 压缩接口：输入Eigen矩阵，输出压缩后的字节流
    std::vector<uint8_t> compress(const Eigen::MatrixXf& matrix);
    
    // 解压接口：输入字节流，输出Eigen矩阵
    Eigen::MatrixXf decompress(const std::vector<uint8_t>& compressed);

private:
    Config config_;
    
    // 压缩单个Eigen块
    std::vector<uint8_t> compress_block(const Eigen::Ref<const Eigen::MatrixXf>& block);
    
    // 解压单个块到Eigen矩阵
    void decompress_block(const uint8_t* data, Eigen::Ref<Eigen::MatrixXf> block);
};