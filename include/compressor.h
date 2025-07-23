#pragma once
#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>
#include "BEVData.h"
#include <filesystem>

class BEVCompressor {
public:
    struct Config {
        int block_size = 16;          // 分块大小
        float compression_ratio = 5.0f; // 目标压缩比
        bool lossless = true;        // 无损模式开关
        const int ZFP_MODE_LOSSLESS = 0;  // 无损模式
        const int ZFP_MODE_DEFAULT = 1;   // 默认（有损）模式
    };

    // 压缩后的数据结构体
    struct compressor
    {
        int nums_packets;
        std::vector<uint8_t> compresseds;
    };


    struct BlockMeta {
        size_t offset;               // 压缩数据起始位置
        size_t size;                 // 压缩数据长度
    };

     // 缓存项的键
    struct CacheKey {
        uint64_t timestamp;
        int x;
        int y;
        bool operator==(const CacheKey& other) const {
            return timestamp == other.timestamp && x == other.x && y == other.y;
        }
    };
    // 哈希函数
    struct CacheKeyHash {
        std::size_t operator()(const CacheKey& key) const {
            return std::hash<uint64_t>{}(key.timestamp) ^ std::hash<int>{}(key.x) ^ std::hash<int>{}(key.y);
        }
    };

    // 存储压缩块的元数据
    std::unordered_map<CacheKey, BlockMeta, CacheKeyHash> block_meta_map;

    // 构造函数
    explicit BEVCompressor(const Config& config);
    
    // 压缩接口：输入Eigen矩阵，输出压缩后的字节流
    std::vector<uint8_t> compress(const BEVFeaturePacket& matrix);
    
    // 解压完整矩阵
    BEVFeaturePacket decompress_complete_block(
        const std::vector<uint8_t>& compresseds,
        uint32_t offset,
        uint16_t Lencompressed);

    // 解压子矩阵
    Eigen::MatrixXf decompress_son_block(
        const std::vector<uint8_t>& compresseds,
        const CacheKey& key,
        uint64_t initial_timestamp,
        size_t Lencompressed);

private:
    Config config_;

    // 压缩单个Eigen块
    std::vector<uint8_t> compress_block(const Eigen::Ref<const Eigen::MatrixXf>& block);
    
    // 解压单个块到Eigen矩阵
    void decompress_block(const uint8_t* data, Eigen::Ref<Eigen::MatrixXf> block);
};