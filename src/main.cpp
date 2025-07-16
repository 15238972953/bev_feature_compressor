#include "compressor.h"
#include "cache_system.h"
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

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

void test_compression(const std::string& filename) {
    BEVCompressor::Config config;
    config.compression_ratio = 5.0f;
    config.block_size = 16;
    config.lossless = false;
    
    BEVCompressor compressor(config);
    BEVCache cache(100);  // 缓存最近100帧

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

    // 压缩数据包
    std::vector<uint8_t> compressed = compressor.compress(packets);
    std::cout << "compressed.size():" << compressed.size() << std::endl;

    // 解压缩验证
    std::vector<BEVFeaturePacket> decompressed = compressor.decompress(compressed);

    // std::cout << "decompressed.size():" << decompressed.size() << std::endl;
    
    // 缓存压缩结果（注意：这里缓存的是压缩后的数据，而非原始数据包）
    // cache.put(timestamp, std::move(test_packet));

    // // 从缓存读取原始数据包（如果需要使用原始数据）
    // BEVFeaturePacket retrieved_packet;
    // if (cache.get(timestamp, retrieved_packet)) {
    //     std::cout << "Cache hit! 成功获取时间戳: " << timestamp << std::endl;
    // } else {
    //     std::cout << "Cache miss!" << std::endl;
    // }

    // test
    // size_t original_size = packets[0].feature.size();
    // std::cout << "original_size:" << original_size << std::endl;

    // 计算压缩率 & 验证解压缩结果（比较范数）


        // if (!decompressed.empty() && decompressed[i].feature.size() == packets[i].feature.size()) {
        //     float norm_diff = (decompressed[i].feature - packets[i].feature).norm();
        //     std::cout << "解压缩误差范数: " << norm_diff << std::endl;
        // }
    size_t original_size = packets[0].feature.size()* sizeof(float);
    std::cout << "原始大小: " << original_size << " 字节" << std::endl;
    std::cout << "压缩后大小: " << compressed.size() << " 字节 ("
                << original_size / float(compressed.size()) << "x)" << std::endl;
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