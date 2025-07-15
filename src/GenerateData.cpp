#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <random>
#include <vector>
#include <filesystem>

namespace fs = std::filesystem; 

/**
 * @brief 生成带时间相关性的BEV帧数据
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @param data_type 数据类型：0-随机数, 1-渐变值, 2-稀疏矩阵
 * @param noise_level 噪声级别（0.0~1.0）
 * @param frame_id 帧ID（用于实现时间变化）
 * @return 生成的Eigen矩阵
 */
Eigen::MatrixXf generate_bev_frame(int rows, int cols, int data_type, 
                                  float noise_level, int frame_id) {
    Eigen::MatrixXf matrix(rows, cols);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    // 根据帧ID生成随时间变化的数据
    switch(data_type) {
        case 0:  // 随机数据（每帧独立）
            matrix = Eigen::MatrixXf::Random(rows, cols);
            break;

        case 1:  // 渐变数据（随时间缓慢变化）
        {
            float time_factor = 0.5f + 0.5f * std::sin(frame_id * 0.1f);  // 周期性变化
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    matrix(i, j) = (i + j) / static_cast<float>(rows + cols) * time_factor;
                }
            }
            break;
        }

        case 2:  // 稀疏数据（模拟移动障碍物）
        {
            matrix.setZero();
            int obstacle_speed = 2;  // 障碍物移动速度（每帧偏移量）
            int offset = (frame_id * obstacle_speed) % cols;  // 随帧移动的偏移

            // 在中间区域生成障碍物
            int start_row = rows / 4;
            int end_row = rows * 3 / 4;
            int start_col = cols / 4;
            int end_col = cols * 3 / 4;

            for (int i = start_row; i < end_row; ++i) {
                for (int j = start_col; j < end_col; ++j) {
                    int shifted_j = (j + offset) % cols;  // 列偏移，模拟水平移动
                    matrix(i, shifted_j) = dist(gen);  // 障碍物区域赋值
                }
            }
            break;
        }
    }

    // 添加噪声
    if (noise_level > 0) {
        Eigen::MatrixXf noise = Eigen::MatrixXf::Random(rows, cols) * noise_level;
        matrix += noise;
        matrix = matrix.cwiseMax(0.0f).cwiseMin(1.0f);  // 限制在[0,1]范围
    }

    return matrix;
}

/**
 * @brief 将多帧数据写入单个文件
 * @param file_path 目标文件路径
 * @param frames 所有帧的矩阵数据
 */
void save_multi_frames(const std::string& file_path, const std::vector<Eigen::MatrixXf>& frames) {
    std::ofstream out(file_path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("无法打开文件写入: " + file_path);
    }

    // 第一步：写入总帧数
    int num_frames = frames.size();
    out.write(reinterpret_cast<const char*>(&num_frames), sizeof(num_frames));
    if (!out) {
        throw std::runtime_error("写入帧数失败: " + file_path);
    }

    // 第二步：逐帧写入数据（帧尺寸 + 帧数据）
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto& frame = frames[i];
        int rows = frame.rows();
        int cols = frame.cols();

        // 写入当前帧的尺寸（行数 + 列数）
        out.write(reinterpret_cast<const char*>(&rows), sizeof(rows));
        out.write(reinterpret_cast<const char*>(&cols), sizeof(cols));

        // 写入当前帧的数据
        size_t data_size = rows * cols * sizeof(float);
        out.write(reinterpret_cast<const char*>(frame.data()), data_size);

        // 检查写入是否成功
        if (!out) {
            throw std::runtime_error("写入第 " + std::to_string(i) + " 帧失败");
        }
    }
}

int main(int argc, char** argv) {
    // 可配置参数
    int num_frames = 50;
    int rows = 256;
    int cols = 256;
    int data_type = 0;      // 0-随机 1-渐变 2-稀疏
    float noise_level = 0.2f;
    const std::string output_file = "bev_test_data.bin";  // 测试数据保存路径

    std::cout << "请输入文件参数：1-num_frames(default=50) 2-rows(default=256) 3-cols(default=256) 4-data_type(0-随机 1-渐变 2-稀疏,default=0) 5-noise_level(default=0.2)\n" << "输入（空格分隔，空则用默认）：" << std::endl;
    
    std::string line;
    std::getline(std::cin, line);

    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;

    while (iss >> token) {
        tokens.push_back(token);
    }

    // 读取参数（按顺序）
    if (tokens.size() >= 1 && !tokens[0].empty()) std::istringstream(tokens[0]) >> num_frames;
    if (tokens.size() >= 2 && !tokens[1].empty()) std::istringstream(tokens[1]) >> rows;
    if (tokens.size() >= 3 && !tokens[2].empty()) std::istringstream(tokens[2]) >> cols;
    if (tokens.size() >= 4 && !tokens[3].empty()) std::istringstream(tokens[3]) >> data_type;
    if (tokens.size() >= 5 && !tokens[4].empty()) std::istringstream(tokens[4]) >> noise_level;
    
    try {
        // 生成所有帧数据
        std::vector<Eigen::MatrixXf> all_frames;
        all_frames.reserve(num_frames);  // 预分配内存，提高效率

        for (int i = 0; i < num_frames; ++i) {
            Eigen::MatrixXf frame = generate_bev_frame(rows, cols, data_type, noise_level, i);
            all_frames.push_back(std::move(frame));  // 移动语义，减少拷贝

            // 打印进度
            if (i % 10 == 0) {  // 每10帧打印一次
                std::cout << "已生成 " << i + 1 << "/" << num_frames << " 帧" << std::endl;
            }
        }

        // 写入多帧数据到单个文件
        save_multi_frames(output_file, all_frames);

        // 输出统计信息
        size_t total_data_size = num_frames * rows * cols * sizeof(float);
        std::cout << "\n===== 生成完成 =====" << std::endl;
        std::cout << "总帧数: " << num_frames << std::endl;
        std::cout << "单帧尺寸: " << rows << "x" << cols << std::endl;
        std::cout << "总数据量: " << total_data_size / (1024 * 1024) << " MB" << std::endl;
        std::cout << "文件路径: " << fs::absolute(output_file) << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}