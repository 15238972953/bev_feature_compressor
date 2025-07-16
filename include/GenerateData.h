#pragma once
#include "BEVData.h"
#include <iostream>
#include <random>
#include <filesystem>

class BEVDataGenerator {
public:
    // 生成BEV帧数据
    BEVFeaturePacket generate_bev_frame(int rows, int cols, int data_type, float noise_level);
    void save_multi_frames(const std::string& file_path, const std::vector<BEVFeaturePacket>& packets);

private:
    
};