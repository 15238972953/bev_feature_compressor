#include "cache_system.h"
#include <iostream>

BEVCache::BEVCache(size_t max_size) : max_size_(max_size) {
}

void BEVCache::put(timestamp_t ts, BEVFeaturePacket&& packet) {
    // 检查是否已存在相同时间戳的数据
    auto it = cache_map_.find(ts);
    if (it != cache_map_.end()) {
        cache_list_.erase(it->second);
        cache_map_.erase(it);
    }
    
    // 利用移动语义避免深拷贝
    cache_list_.push_front({ts, std::move(packet)});
    cache_map_[ts] = cache_list_.begin();
    
    // 超出容量时淘汰最旧数据
    if (cache_list_.size() > max_size_) {
        cache_map_.erase(cache_list_.back().ts);
        cache_list_.pop_back();
    }
}

bool BEVCache::get(timestamp_t ts, BEVFeaturePacket& out_packet) {
    requests_++;
    auto it = cache_map_.find(ts);
    if (it == cache_map_.end()) return false;
    
    hits_++;
    // 使用移动语义或深拷贝（根据需求选择）
    out_packet = std::move(it->second->packet);  // 移动语义（若不需要保留缓存中的副本）
    // 或者使用深拷贝（保留缓存中的原始数据）
    // out_packet = it->second->packet;
    
    // 将访问的数据移至链表头部（LRU策略）
    cache_list_.splice(cache_list_.begin(), cache_list_, it->second);
    return true;
}