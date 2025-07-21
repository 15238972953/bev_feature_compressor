#include "compressor.h"
#include "cache_system.h"
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <memory>

// 读取并解析数据（配套解析函数）
std::vector<BEVFeaturePacket> read_multi_frames(const std::string& file_path) {
    std::ifstream in(file_path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("无法打开文件读取: " + file_path);
    }

    // 读取总帧数
    uint32_t num_packets;
    in.read(reinterpret_cast<char*>(&num_packets), sizeof(num_packets));

    std::vector<BEVFeaturePacket> packets;
    packets.reserve(num_packets);

    // 逐帧读取
    for (uint32_t i = 0; i < num_packets; ++i) {
        BEVFeaturePacket packet;

        // 读取时间戳
        in.read(reinterpret_cast<char*>(&packet.timestamp), sizeof(packet.timestamp));
        
        // 读取传感器上下文
        in.read(reinterpret_cast<char*>(&packet.sensor_ctx.ego_speed), sizeof(packet.sensor_ctx.ego_speed));
        in.read(reinterpret_cast<char*>(&packet.sensor_ctx.health), sizeof(packet.sensor_ctx.health));
        in.read(reinterpret_cast<char*>(packet.sensor_ctx.ego_pose.data()), 
                  packet.sensor_ctx.ego_pose.size() * sizeof(float));
        
        // 读取特征元数据
        in.read(reinterpret_cast<char*>(&packet.feature_meta.rows), sizeof(packet.feature_meta.rows));
        in.read(reinterpret_cast<char*>(&packet.feature_meta.cols), sizeof(packet.feature_meta.cols));
        in.read(reinterpret_cast<char*>(&packet.feature_meta.value_min), sizeof(packet.feature_meta.value_min));
        in.read(reinterpret_cast<char*>(&packet.feature_meta.value_max), sizeof(packet.feature_meta.value_max));
        in.read(reinterpret_cast<char*>(&packet.feature_meta.channel), sizeof(packet.feature_meta.channel));
        in.read(reinterpret_cast<char*>(&packet.feature_meta.is_normalized), sizeof(packet.feature_meta.is_normalized));
        
        // 读取矩阵数据
        packet.feature = Eigen::MatrixXf(packet.feature_meta.rows, packet.feature_meta.cols);
        size_t data_size = packet.feature.size() * sizeof(float);
        in.read(reinterpret_cast<char*>(packet.feature.data()), data_size);

        if (!in) {
            throw std::runtime_error("读取第 " + std::to_string(i) + " 帧失败");
        }

        packets.push_back(std::move(packet));
    }

    return packets;
}

// 打印矩阵数据
void print_data(const BEVFeaturePacket& packet, int len){
    // 打印前len行，前len列数据
    for (int i = 0; i < len; ++i) {
        for (int j = 0; j < len; ++j) {
            // 设置固定宽度（如8字符），右对齐，保留3位小数
            std::cout << std::right << std::setw(8) 
                    << std::fixed << std::setprecision(3) 
                    << packet.feature(i, j) << " ";
        }
        std::cout << "\n";
    }
}


void print_data(const Eigen::Ref<const Eigen::MatrixXf>& block, int len){
    // 打印前len行，前len列数据
    for (int i = 0; i < len; ++i) {
        for (int j = 0; j < len; ++j) {
            // 设置固定宽度（如8字符），右对齐，保留3位小数
            std::cout << std::right << std::setw(8) 
                    << std::fixed << std::setprecision(3) 
                    << block(i, j) << " ";
        }
        std::cout << "\n";
    }
}

void test_compression(const std::string& filename) {
    BEVCompressor::Config config;
    config.compression_ratio = 16.0f;
    config.block_size = 16;
    config.lossless = false;
    
    BEVCompressor compressor(config);

    // 从文件读取数据包
    std::vector<BEVFeaturePacket> packets = read_multi_frames(filename);
    std::cout << "读取 " << packets.size() << " 个数据包" << std::endl;

    if (packets.empty()) {
        std::cout << "文件中没有有效数据包" << std::endl;
        return;
    }

    // 选择第一个数据包进行测试
    // BEVFeaturePacket& test_packet = packets[0];
    // BEVCache::timestamp_t timestamp = test_packet.timestamp;
    // std::cout << "正在处理数据包: " << timestamp << std::endl;


    // 缓存解压结果（注意：这里缓存的是解压缩的数据，而非压缩数据包）
    BEVCache::BEVCacheConfig cache_config;
    cache_config.max_cache_size = 250; // 最多缓存250个块
    BEVCache cache(cache_config);

    // 存储所有压缩数据
    std::vector<uint8_t> compresseds;
    uint64_t initial_timestamp = static_cast<uint64_t>(packets[0].timestamp);
    uint16_t Lencompressed = 0;
    uint16_t Frame_ID = 0;  // 帧数
    for (Frame_ID = 0; Frame_ID < packets.size(); Frame_ID++)
    {
        // 压缩数据包
        std::vector<uint8_t> compressed = compressor.compress(packets[Frame_ID]);
        Lencompressed = compressed.size();
        compresseds.insert(compresseds.end(), compressed.begin(), compressed.end());
        // print_data(packets[i], 10);
        // std::cout << "compresseds.size():" << compresseds.size() << " bytes." << std::endl;
        // 将压缩数据插入缓存
        // cache.insertPackets(compressed);
    }
    std::cout << "压缩完成，共 " << compresseds.size() << " 字节" << std::endl;
    // 解压某一帧数据包
    uint64_t timestamp = static_cast<uint64_t>(packets[0].timestamp);
    std::cout << "正在解压数据包: " << timestamp << std::endl;

    // std::cout << "数据包个数：" << compresseds.size() /Lencompressed << std::endl;
    // 时间戳相减是“大错误”
    BEVFeaturePacket decompressed1 = compressor.decompress_complete_block(compresseds, 0 * Lencompressed, Lencompressed);
    print_data(decompressed1, 10);

    // 解压某一帧第3行第二个子块数据包
    // auto key_obj = BEVCompressor::CacheKey{timestamp, 3, 2};
    // Eigen::MatrixXf decompressed2 = compressor.decompress_son_block(compresseds, key_obj, initial_timestamp, Lencompressed);
    // print_data(decompressed2, 10);



    // 打印matrix信息（调试用）
    // std::cout << "Matrix size: " << decompressed[0].feature.rows() << "x" << decompressed[0].feature.cols() << std::endl;
    // for (int i = 0; i < 10; ++i) {
    //     for (int j = 0; j < 10; ++j) {
    //         // 设置固定宽度（如8字符），右对齐，保留3位小数
    //         std::cout << std::right << std::setw(8) 
    //                 << std::fixed << std::setprecision(3) 
    //                 << decompressed[0].feature(i, j) << " ";
    //     }
    //     std::cout << "\n";
    // }

    // std::cout << "decompressed.size():" << decompressed.size() << std::endl;


    // uint64_t timestamp = static_cast<uint64_t>(packets[6].timestamp);
    // std::vector<uint8_t> retrieved_data;
    // uint16_t rows, cols;
    // bool found = cache.retrieve(timestamp, 0, 0, retrieved_data, rows, cols);
    // if (found){
    //     // 解压缩验证
    //     std::vector<BEVFeaturePacket> decompressed = compressor.decompress(retrieved_data);
    //     for (int i = 0; i < 10; ++i) {
    //         for (int j = 0; j < 10; ++j) {
    //             // 设置固定宽度（如8字符），右对齐，保留3位小数
    //             std::cout << std::right << std::setw(8) 
    //                     << std::fixed << std::setprecision(3) 
    //                     << decompressed[0].feature(i, j) << " ";
    //         }
    //         std::cout << "\n";
    //     }
    // }
    

    // // 输出缓存命中率
    // std::cout << "Cache hit rate: " << cache.getHitRate() << std::endl;
    // // 导出统计信息到JSON
    // std::string stats = cache.getStatsAsJSON();
    // std::cout << "Cache stats: " << stats << std::endl;

    // // 从缓存读取原始数据包（如果需要使用原始数据）


    // test
    // size_t original_size = packets[0].feature.size();
    // std::cout << "original_size:" << original_size << std::endl;

    // 计算压缩率
    // size_t original_size = packets[0].feature.size()* sizeof(float) * packets.size();
    // std::cout << "原始大小: " << original_size << " 字节" << std::endl;
    // std::cout << "压缩后大小: " << compressed.size() << std::endl;
    // std::cout << "解压缩后大小: " << decompressed[0].feature.size() * sizeof(float) * decompressed.size() << " 字节" << std::endl;
    // std::cout <<" 字节 (" << original_size / float(compressed.size()) << "x)" << std::endl;

    // 验证解压缩结果（比较范数）
    // if (!decompressed.empty() && decompressed[0].feature.size() == packets[0].feature.size()) {
    //     float norm_diff = (decompressed[0].feature - packets[0].feature).norm();
    //     std::cout << "解压缩误差范数: " << norm_diff << std::endl;
    // }
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "错误：缺少输入文件路径" << std::endl;
        std::cerr << "用法示例：" << argv[0] << " <bin文件路径>" << std::endl;
        return 1;
    }
    
    try {
        test_compression(argv[1]);
    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}