#include "cache_system.h"

BEVCache::BEVCache(size_t max_size) : max_size_(max_size) {
}

void BEVCache::put(timestamp_t ts, Eigen::MatrixXf& matrix) {
    if (cache_map_.find(ts) != cache_map_.end()) {
        cache_list_.erase(cache_map_[ts]);
    }
    
    // 利用移动语义避免矩阵拷贝
    cache_list_.push_front({ts, std::move(matrix)});
    cache_map_[ts] = cache_list_.begin();
    
    if (cache_list_.size() > max_size_) {
        cache_map_.erase(cache_list_.back().ts);
        cache_list_.pop_back();
    }
}

bool BEVCache::get(timestamp_t ts, Eigen::MatrixXf& out_matrix) {
    requests_++;
    auto it = cache_map_.find(ts);
    if (it == cache_map_.end()) return false;
    
    hits_++;
    out_matrix = it->second->matrix;  // Eigen的赋值操作会自动处理内存
    cache_list_.splice(cache_list_.begin(), cache_list_, it->second);
    return true;
}