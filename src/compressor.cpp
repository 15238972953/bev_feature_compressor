#include "compressor.h"
#include <zfp.h>
// #include <eigen3/Eigen/Core>
#include <iostream>

BEVCompressor::BEVCompressor(const Config& config) : config_(config) {}

std::vector<uint8_t> BEVCompressor::compress(const std::vector<BEVFeaturePacket>& packets) {
    std::vector<uint8_t> compressed_data;
    const int bs = config_.block_size;
    
    // 写入数据包数量
    uint32_t num_packets = packets.size();
    compressed_data.insert(compressed_data.end(), 
                          reinterpret_cast<uint8_t*>(&num_packets),
                          reinterpret_cast<uint8_t*>(&num_packets + 1));
    
    // 遍历每个数据包
    std::cout << "timestamp[0]" << packets[0].timestamp << std::endl;
    for (const auto& packet : packets) {
        const Eigen::MatrixXf& matrix = packet.feature;
        // 打印matrix信息（调试用）
        // std::cout << "Matrix size: " << matrix.rows() << "x" << matrix.cols() << std::endl;
        // for (int i = 0; i < 10; ++i) {
        //     for (int j = 0; j < 10; ++j) {
        //         std::cout << matrix(i,j) << " ";
        //     }
        //     std::cout << "\n";
        // }
        
        // 写入时间戳
        uint64_t timestamp = static_cast<uint64_t>(packet.timestamp);
        compressed_data.insert(compressed_data.end(), 
                              reinterpret_cast<const uint8_t*>(&timestamp),
                              reinterpret_cast<const uint8_t*>(&timestamp + 1));
        
        uint16_t nums_block = matrix.rows() * matrix.cols() / ( bs * bs);
        compressed_data.insert(compressed_data.end(), 
                              reinterpret_cast<const uint8_t*>(&nums_block),
                              reinterpret_cast<const uint8_t*>(&nums_block + 1));
        // 遍历所有块
        for (int i = 0; i < matrix.rows(); i += bs) {
            for (int j = 0; j < matrix.cols(); j += bs) {
                // 处理边缘块（如果不足block_size）
                int block_rows = std::min<int>(bs, matrix.rows() - i);
                int block_cols = std::min<int>(bs, matrix.cols() - j);
                
                // 使用Eigen的block()获取子矩阵视图
                auto block = matrix.block(i, j, block_rows, block_cols);
                auto compressed_block = compress_block(block);
                // std::cout << "compressed_block.size():" << compressed_block.size() << std::endl;
                // throw std::runtime_error("结束调试！");

                // 写入块头信息（位置+大小+行数+列数）
                uint16_t header[4] = {
                    static_cast<uint16_t>(i), 
                    static_cast<uint16_t>(j),
                    static_cast<uint16_t>(block_rows),
                    static_cast<uint16_t>(compressed_block.size())
                };
                compressed_data.insert(
                    compressed_data.end(), 
                    reinterpret_cast<uint8_t*>(header), 
                    reinterpret_cast<uint8_t*>(header + 4)
                );
                
                // 写入压缩数据
                compressed_data.insert(
                    compressed_data.end(), 
                    compressed_block.begin(), 
                    compressed_block.end()
                );
            }
        }
    }
    std::cout << "Compressed " << compressed_data.size() << " bytes." << std::endl;
    return compressed_data;
}


std::vector<uint8_t> BEVCompressor::compress_block(
    const Eigen::Ref<const Eigen::MatrixXf>& block) 
{
    // 1. 检查块尺寸合法性
    if (block.rows() <= 0 || block.cols() <= 0) {
        throw std::invalid_argument("压缩块尺寸无效（行数或列数为0）");
    }

    // 2. 准备ZFP字段（处理const指针的类型转换）
    zfp_type type = zfp_type_float;  // 匹配Eigen::MatrixXf的float类型
    // 注意：zfp_field_2d要求非const指针，但实际不会修改数据（仅读取）
    zfp_field* field = zfp_field_2d(
        const_cast<float*>(block.data()),  // 类型转换（const_cast安全，因zfp_compress仅读取）
        type, 
        static_cast<size_t>(block.rows()),  // zfp要求size_t类型
        static_cast<size_t>(block.cols())
    );
    if (!field) {
        throw std::runtime_error("ZFP字段创建失败");
    }

    // 3. 配置ZFP压缩流
    zfp_stream* stream = zfp_stream_open(nullptr);
    if (!stream) {
        zfp_field_free(field);  // 释放已分配资源
        throw std::runtime_error("ZFP流创建失败");
    }

    // 设置压缩率：根据配置选择无损/有损模式
    int flags = config_.lossless ? config_.ZFP_MODE_LOSSLESS :config_.ZFP_MODE_DEFAULT;
    zfp_stream_set_rate(
        stream, 
        config_.compression_ratio,  // 压缩率（比特/值）
        type, 
        2,  // 2D数据
        flags
    );

    // 4. 分配压缩缓冲区（预计算最大所需大小）
    size_t bufsize = zfp_stream_maximum_size(stream, field);
    if (bufsize == 0) {
        zfp_stream_close(stream);
        zfp_field_free(field);
        throw std::runtime_error("无法计算压缩缓冲区大小");
    }
    std::vector<uint8_t> buffer(bufsize);

    // 5. 绑定比特流并执行压缩
    bitstream* bit = stream_open(buffer.data(), bufsize);
    if (!bit) {
        zfp_stream_close(stream);
        zfp_field_free(field);
        throw std::runtime_error("比特流创建失败");
    }
    zfp_stream_set_bit_stream(stream, bit);

    // 执行压缩（返回压缩后的块数，2D场景应为1）
    size_t compressed_blocks = zfp_compress(stream, field);
    if (!compressed_blocks) {
        stream_close(bit);
        zfp_stream_close(stream);
        zfp_field_free(field);
        throw std::runtime_error("块压缩失败");
    }

    // 6. 获取实际压缩大小（裁剪缓冲区到有效长度）
    size_t actual_size = stream_size(bit) / 8;  // 比特流大小转换为字节
    buffer.resize(actual_size);

    // 7. 释放资源（按顺序释放，避免内存泄漏）
    stream_close(bit);         // 先关闭比特流
    zfp_stream_close(stream);  // 再关闭压缩流
    zfp_field_free(field);     // 最后释放字段

    return buffer;
}

std::vector<BEVFeaturePacket> BEVCompressor::decompress(const std::vector<uint8_t>& compressed) {
    std::vector<BEVFeaturePacket> packets;
    const uint8_t* ptr = compressed.data();
    const uint8_t* end = compressed.data() + compressed.size();
    std::cout << "Decompressing " << compressed.size() << " bytes..." << std::endl;

    // 读取数据包数量
    if (ptr + sizeof(uint32_t) > end) {
        throw std::runtime_error("压缩数据不完整：缺少数据包数量");
    }
    uint32_t num_packets = *reinterpret_cast<const uint32_t*>(ptr);
    // std::cout << "num_packets:" << num_packets << std::endl;
    ptr += sizeof(uint32_t);

    packets.reserve(num_packets);

    // 逐个解压缩数据包
    for (uint32_t p = 0; p < num_packets; ++p) {
        BEVFeaturePacket packet;

        // 读取时间戳（替代frame_id）
        if (ptr + sizeof(uint64_t) > end) {
            throw std::runtime_error("压缩数据不完整：缺少时间戳");
        }
        packet.timestamp = *reinterpret_cast<const uint64_t*>(ptr);
        // std::cout << "timestamp:" << packet.timestamp << std::endl;
        ptr += sizeof(uint64_t);

        // 读取blocks的数量
        if (ptr + sizeof(uint16_t) > end) {
            throw std::runtime_error("压缩数据不完整：缺少block数量");
        }
        uint16_t nums_blocks = *reinterpret_cast<const uint16_t*>(ptr);
        // std::cout << "nums_blocks:" << nums_blocks << std::endl;
        ptr += sizeof(uint16_t);


        // 初始化特征矩阵（假设所有块大小一致）
        packet.feature = Eigen::MatrixXf::Zero(256, 256);  // 需根据实际情况调整尺寸

        // 解压缩所有块
        for (int i=0; i<nums_blocks; ++i) { // 块头包含4个uint16_t
            // 读取块头（行偏移、列偏移、块行数、压缩大小）
            uint16_t row_offset = *reinterpret_cast<const uint16_t*>(ptr);
            ptr += sizeof(uint16_t);
            uint16_t col_offset = *reinterpret_cast<const uint16_t*>(ptr);
            ptr += sizeof(uint16_t);
            uint16_t block_rows = *reinterpret_cast<const uint16_t*>(ptr);
            ptr += sizeof(uint16_t);
            uint16_t block_size = *reinterpret_cast<const uint16_t*>(ptr);
            ptr += sizeof(uint16_t);
            
            // std::cout << block_rows << std::endl;

            // 确保指针不越界
            if (ptr + block_size > end) {
                throw std::runtime_error("压缩数据不完整：块数据缺失");
            }

            // 获取目标块（处理边缘块）
            int block_cols = config_.block_size;
            if (col_offset + block_cols > packet.feature.cols()) {
                block_cols = packet.feature.cols() - col_offset;
            }

            auto block = packet.feature.block(row_offset, col_offset, block_rows, block_cols);

            // 解压块
            decompress_block(ptr, block);
            ptr += block_size;
        }

        packets.push_back(std::move(packet));
    }

    return packets;
}

void BEVCompressor::decompress_block(const uint8_t* data, Eigen::Ref<Eigen::MatrixXf> block) {
    // 创建ZFP解压流
    bitstream* bit = stream_open(const_cast<uint8_t*>(data), block.size() * sizeof(float));
    zfp_stream* stream = zfp_stream_open(bit);
    
    // 创建ZFP字段（描述解压数据的结构）
    zfp_field* field = zfp_field_2d(
        block.data(),
        zfp_type_float,
        static_cast<size_t>(block.rows()),
        static_cast<size_t>(block.cols())
    );

    // 设置解压参数（需与压缩时一致）
    zfp_stream_set_rate(
        stream,
        config_.compression_ratio,
        zfp_type_float,
        2,  // 2D数据
        config_.lossless ? config_.ZFP_MODE_LOSSLESS : config_.ZFP_MODE_DEFAULT
    );

    // 执行解压
    bool success = zfp_decompress(stream, field);
    if (!success) {
        throw std::runtime_error("ZFP解压失败");
    }

    // 释放资源
    zfp_field_free(field);
    zfp_stream_close(stream);
    stream_close(bit);
}