#include "compressor.h"
#include "cache_system.h"
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>

// 从二进制文件读取矩阵
Eigen::MatrixXf read_matrix(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        throw std::runtime_error("无法打开文件: " + filename);
    }
    
    // 读取矩阵尺寸
    int rows, cols;
    file.read(reinterpret_cast<char*>(&rows), sizeof(rows));
    file.read(reinterpret_cast<char*>(&cols), sizeof(cols));
    
    // 读取矩阵数据
    Eigen::MatrixXf matrix(rows, cols);
    file.read(reinterpret_cast<char*>(matrix.data()), rows * cols * sizeof(float));
    
    if (!file) {
        throw std::runtime_error("读取文件失败: " + filename);
    }
    
    return matrix;
}

void test_compression(const std::string& filename) {
    BEVCompressor::Config config;
    config.compression_ratio = 5.0f;
    
    BEVCompressor compressor(config);

    // 缓存最近50帧
    BEVCache cache(10);
    
    // 从文件读取矩阵
    Eigen::MatrixXf matrix = read_matrix(filename);
    std::cout << "读取矩阵: " << matrix.rows() << "x" << matrix.cols() << std::endl;

    uint64_t timestamp = 523478528054;
    
    // 压缩并缓存
    auto compressed = compressor.compress(matrix);
    cache.put(timestamp, matrix);

    // 从缓存读取
    Eigen::MatrixXf retrieved;
    if (cache.get(timestamp,retrieved)){
        std::cout << "Cache hit! Matrix norm:" << retrieved.norm() << std::endl;
    }
    std::cout << "压缩后大小: " << compressed.size() << " 字节 ("
              << float(compressed.size()) / (matrix.size() * sizeof(float)) << "x)" << std::endl;
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