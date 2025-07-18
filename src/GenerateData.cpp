#include "GenerateData.h"
#include <fstream>
#include <chrono>
#include <thread>

namespace fs = std::filesystem; 
const float NS_TO_S_RATE = 1e-8f;  // 纳秒到秒的转换的比例因子
const float ROTATION_RATE = 1e-10f;  // 车辆旋转速率（模拟）的比例因子

/**
 * @brief 生成带时间相关性的BEV帧数据
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 * @param data_type 数据类型：0-随机数, 1-渐变值, 2-稀疏矩阵
 * @param noise_level 噪声级别（0.0~1.0）
 * @param frame_id 帧ID（用于实现时间变化）
 * @return 生成的Eigen矩阵
 */
BEVFeaturePacket BEVDataGenerator::generate_bev_frame(int rows, int cols, int data_type, float noise_level) {
    BEVFeaturePacket packet;
    // 获取纳秒级时间戳
    auto now = std::chrono::high_resolution_clock::now();
    auto timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
    packet.timestamp = timestamp_ns; // 纳秒级时间戳
    // 模拟传感器上下文
    packet.sensor_ctx.ego_speed = 15.0f + std::sin(timestamp_ns) * 5.0f; // 10-20m/s波动
    packet.sensor_ctx.health = SensorHealth::NORMAL;
    packet.sensor_ctx.ego_pose = {timestamp_ns * NS_TO_S_RATE, timestamp_ns * NS_TO_S_RATE, timestamp_ns * ROTATION_RATE}; // 模拟车辆旋转
    
    packet.feature = Eigen::MatrixXf(rows, cols);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist(0.0f, 1.0f);
    std::normal_distribution<float> noise_dist(0.0f, noise_level);

    // 根据数据类型生成不同特征
    switch (data_type) {
        case 0: {  // 随机噪声
            packet.feature = Eigen::MatrixXf::Random(rows, cols);
            packet.feature_meta.value_min = -1.0f;
            packet.feature_meta.value_max = 1.0f;
            break;
        }
        case 1: {  // 渐变分布（模拟距离衰减）
            packet.feature.setZero();
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    float dist_from_center = std::sqrt(
                        std::pow(i - rows/2.0f, 2) + 
                        std::pow(j - cols/2.0f, 2)
                    );
                    float normalized_dist = dist_from_center / std::sqrt(rows*rows + cols*cols) * 2.0f;
                    packet.feature(i, j) = std::max(0.0f, 1.0f - normalized_dist);
                }
            }
            packet.feature_meta.value_min = 0.0f;
            packet.feature_meta.value_max = 1.0f;
            break;
        }
        case 2: {  // 移动障碍物（时间相关）
            packet.feature.setZero();
            
            // 障碍物参数
            int obstacle_size = std::min(rows, cols) / 10;
            int obstacle_x = (cols / 2) + (timestamp_ns / 10) * (cols / 20) - (cols / 4);
            int obstacle_y = (rows / 2) + (timestamp_ns / 5) * (rows / 10) - (rows / 10);
            
            // 绘制障碍物
            for (int i = std::max(0, obstacle_y - obstacle_size); i < std::min(rows, obstacle_y + obstacle_size); ++i) {
                for (int j = std::max(0, obstacle_x - obstacle_size); j < std::min(cols, obstacle_x + obstacle_size); ++j) {
                    packet.feature(i, j) = 1.0f;
                }
            }
            packet.feature_meta.value_min = 0.0f;
            packet.feature_meta.value_max = 1.0f;
            break;
        }
        case 3: {  // 道路结构（网格）
            packet.feature.setZero();
            int grid_size = std::min(rows, cols) / 16;
            
            for (int i = 0; i < rows; ++i) {
                for (int j = 0; j < cols; ++j) {
                    if (i % grid_size == 0 || j % grid_size == 0) {
                        packet.feature(i, j) = 0.8f;
                    }
                }
            }
            packet.feature_meta.value_min = 0.0f;
            packet.feature_meta.value_max = 1.0f;
            break;
        }
        default:
            throw std::invalid_argument("Unsupported data type");
    }

    // 添加噪声
    if (noise_level > 0) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                packet.feature(i, j) += noise_dist(gen);
                packet.feature(i, j) = std::max(packet.feature_meta.value_min, 
                                             std::min(packet.feature_meta.value_max, packet.feature(i, j)));
            }
        }
    }

    // 设置元数据
    packet.feature_meta.rows = rows;
    packet.feature_meta.cols = cols;
    packet.feature_meta.channel = 0;
    packet.feature_meta.is_normalized = true;

    return packet;
}

/**
 * @brief 将多帧数据写入单个文件
 * @param file_path 目标文件路径
 * @param frames 所有帧的矩阵数据
 */
void BEVDataGenerator::save_multi_frames(const std::string& file_path, const std::vector<BEVFeaturePacket>& packets) {
    // 确保目录存在
    fs::path path(file_path);
    if (!path.parent_path().empty()) {
        fs::create_directories(path.parent_path());
    }

    std::ofstream file(file_path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("Failed to open file: " + file_path);
    }

    // 写入帧数
    uint32_t num_packets = packets.size();
    file.write(reinterpret_cast<const char*>(&num_packets), sizeof(num_packets));

    for (const auto& packet : packets) {
        // 写入时间戳
        file.write(reinterpret_cast<const char*>(&packet.timestamp), sizeof(packet.timestamp));
        
        // 写入传感器上下文
        file.write(reinterpret_cast<const char*>(&packet.sensor_ctx.ego_speed), sizeof(packet.sensor_ctx.ego_speed));
        file.write(reinterpret_cast<const char*>(&packet.sensor_ctx.health), sizeof(packet.sensor_ctx.health));
        file.write(reinterpret_cast<const char*>(packet.sensor_ctx.ego_pose.data()), 
                  packet.sensor_ctx.ego_pose.size() * sizeof(float));
        
        // 写入特征元数据
        file.write(reinterpret_cast<const char*>(&packet.feature_meta.rows), sizeof(packet.feature_meta.rows));
        file.write(reinterpret_cast<const char*>(&packet.feature_meta.cols), sizeof(packet.feature_meta.cols));
        file.write(reinterpret_cast<const char*>(&packet.feature_meta.value_min), sizeof(packet.feature_meta.value_min));
        file.write(reinterpret_cast<const char*>(&packet.feature_meta.value_max), sizeof(packet.feature_meta.value_max));
        file.write(reinterpret_cast<const char*>(&packet.feature_meta.channel), sizeof(packet.feature_meta.channel));
        file.write(reinterpret_cast<const char*>(&packet.feature_meta.is_normalized), sizeof(packet.feature_meta.is_normalized));
        
        // 写入矩阵数据
        file.write(reinterpret_cast<const char*>(packet.feature.data()), 
                  packet.feature.size() * sizeof(float));
    }

    if (!file) {
        throw std::runtime_error("Failed to write to file: " + file_path);
    }
}

int main(int argc, char** argv) {
    const int target_fps = 25;             // BEV帧率
    const std::chrono::milliseconds frame_time(1000/target_fps); 

    // 可配置参数
    int num_frames = 50;
    int rows = 256;
    int cols = 256;
    int data_type = 0;      // 0-随机 1-渐变 2-稀疏
    float noise_level = 0.2f;
    const std::string output_file = "bev_test_data.bin";  // 测试数据保存路径

    std::cout << "请输入文件参数：1-num_frames(default=50) 2-rows(default=256) 3-cols(default=256) 4-data_type(0-随机 1-渐变 2-稀疏,default=0) 5-noise_level(default=0.2)\n" << "输入（空格分隔，直接回车则用默认值）：" << std::endl;
    
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

    std::cout << "生成中，请稍候..." << std::endl;

    try {
        BEVDataGenerator generator;
        // 生成所有帧数据
        std::vector<BEVFeaturePacket> all_frames;
        all_frames.reserve(num_frames);  // 预分配内存，提高效率

        for (int i = 0; i < num_frames; ++i) {
            auto start = std::chrono::steady_clock::now();  // 记录循环开始时间 

            BEVFeaturePacket frame = generator.generate_bev_frame(rows, cols, data_type, noise_level);
            all_frames.push_back(std::move(frame));  // 移动语义，减少拷贝

            auto end = std::chrono::steady_clock::now();    // 记录循环结束时间
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            if (elapsed < frame_time) {
                // 如果执行太快，休眠剩余时间
                std::this_thread::sleep_for(frame_time - elapsed);
            } else {
                // 如果执行太慢，可以输出警告或调整逻辑
                std::cerr << "警告: 循环执行时间过长 (" 
                        << elapsed.count() << "ms)" << std::endl;
            }
        }

        // 写入多帧数据到单个文件
        generator.save_multi_frames(output_file, all_frames);

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