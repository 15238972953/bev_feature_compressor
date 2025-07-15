#include "compressor.h"
#include <zfp.h>
#include <eigen3/Eigen/Core>
#include <iostream>

BEVCompressor::BEVCompressor(const Config& config) : config_(config) {}

std::vector<uint8_t> BEVCompressor::compress(const Eigen::MatrixXf& matrix) {
    std::vector<uint8_t> compressed_data;
    const int bs = config_.block_size;
    
    // 遍历所有块
    for (int i = 0; i < matrix.rows(); i += bs) {
        for (int j = 0; j < matrix.cols(); j += bs) {
            // 使用Eigen的block()获取子矩阵视图
            auto block = matrix.block(i, j, bs, bs);
            auto compressed_block = compress_block(block);
            
            // 写入块头信息（位置+大小）
            uint16_t header[3] = {
                static_cast<uint16_t>(i), 
                static_cast<uint16_t>(j),
                static_cast<uint16_t>(compressed_block.size())
            };
            compressed_data.insert(
                compressed_data.end(), 
                reinterpret_cast<uint8_t*>(header), 
                reinterpret_cast<uint8_t*>(header + 3)
            );
            
            // 写入压缩数据
            compressed_data.insert(
                compressed_data.end(), 
                compressed_block.begin(), 
                compressed_block.end()
            );
        }
    }
    return compressed_data;
}

std::vector<uint8_t> BEVCompressor::compress_block(
    const Eigen::Ref<const Eigen::MatrixXf>& block) 
{
    zfp_type type = zfp_type_float;
    zfp_field* field = zfp_field_2d(
        const_cast<float*>(block.data()), // ZFP需要非const指针
        type, 
        block.rows(), 
        block.cols()
    );
    
    zfp_stream* stream = zfp_stream_open(nullptr);
    zfp_stream_set_rate(
        stream, 
        config_.compression_ratio, 
        type, 
        2, 
        config_.lossless ? 0 : 1
    );
    
    size_t bufsize = zfp_stream_maximum_size(stream, field);
    std::vector<uint8_t> buffer(bufsize);
    
    bitstream* bit = stream_open(buffer.data(), bufsize);
    zfp_stream_set_bit_stream(stream, bit);
    zfp_compress(stream, field);
    
    zfp_field_free(field);
    zfp_stream_close(stream);
    stream_close(bit);
    
    return buffer;
}

Eigen::MatrixXf BEVCompressor::decompress(const std::vector<uint8_t>& compressed) {
    Eigen::MatrixXf matrix(256, 256);
    const uint8_t* ptr = compressed.data();
    
    while (ptr < compressed.data() + compressed.size()) {
        // 读取块头
        uint16_t header[3];
        memcpy(header, ptr, 6);
        ptr += 6;
        
        int i = header[0], j = header[1], size = header[2];
        auto block = matrix.block(i, j, config_.block_size, config_.block_size);
        
        // 解压到Eigen块
        decompress_block(ptr, block);
        ptr += size;
    }
    return matrix;
}

void BEVCompressor::decompress_block(
    const uint8_t* data, 
    Eigen::Ref<Eigen::MatrixXf> block
) {
    bitstream* bit = stream_open(const_cast<uint8_t*>(data), block.size() * sizeof(float));
    zfp_stream* stream = zfp_stream_open(bit);
    zfp_field* field = zfp_field_2d(block.data(), zfp_type_float, block.rows(), block.cols());
    
    zfp_decompress(stream, field);
    
    zfp_field_free(field);
    zfp_stream_close(stream);
    stream_close(bit);
}