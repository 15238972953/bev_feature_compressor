#pragma once
#include <eigen3/Eigen/Dense>
#include <list>
#include <unordered_map>
#include "BEVData.h"

class BEVCache {
public:
    using timestamp_t = uint64_t;
    
    explicit BEVCache(size_t max_size = 10);
    
    void put(timestamp_t ts, BEVFeaturePacket&& packet);
    bool get(timestamp_t ts, BEVFeaturePacket& out_packet);
    
    float hit_rate() const { 
        return requests_ > 0 ? static_cast<float>(hits_) / requests_ : 0; 
    }

private:
    struct CacheNode {
        timestamp_t ts;
        BEVFeaturePacket packet;
    };
    
    std::list<CacheNode> cache_list_;
    std::unordered_map<timestamp_t, decltype(cache_list_.begin())> cache_map_;
    size_t max_size_;
    size_t hits_ = 0;
    size_t requests_ = 0;
};