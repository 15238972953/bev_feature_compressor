#include "cache_system.h"
#include <iostream>
#include <cstring>
#include <mutex>

// SimpleMemoryPool实现
SimpleMemoryPool::SimpleMemoryPool(size_t block_size, size_t initial_blocks)
    : block_size_(block_size + sizeof(Block))
{
    // 分配初始块
    char* initial_chunk = new char[block_size_ * initial_blocks];
    allocated_chunks_.push_back(initial_chunk);
    
    // 初始化空闲链表
    free_list_ = reinterpret_cast<Block*>(initial_chunk);
    Block* current = free_list_;
    
    for (size_t i = 0; i < initial_blocks - 1; ++i) {
        current->in_use = false;
        current->next = reinterpret_cast<Block*>(initial_chunk + (i + 1) * block_size_);
        current = current->next;
    }
    
    current->in_use = false;
    current->next = nullptr;
}

SimpleMemoryPool::~SimpleMemoryPool() {
    for (char* chunk : allocated_chunks_) {
        delete[] chunk;
    }
}

void* SimpleMemoryPool::allocate(size_t size) {
    if (size > block_size_ - sizeof(Block)) {
        // 请求的大小超过块大小，使用标准分配
        return ::operator new(size);
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 如果空闲链表为空，分配新块
    if (!free_list_) {
        char* new_chunk = new char[block_size_ * 1024];
        allocated_chunks_.push_back(new_chunk);
        
        // 将新块添加到空闲链表
        free_list_ = reinterpret_cast<Block*>(new_chunk);
        Block* current = free_list_;
        
        for (int i = 0; i < 1023; ++i) {
            current->in_use = false;
            current->next = reinterpret_cast<Block*>(new_chunk + (i + 1) * block_size_);
            current = current->next;
        }
        
        current->in_use = false;
        current->next = nullptr;
    }
    
    // 从空闲链表获取块
    Block* block = free_list_;
    free_list_ = block->next;
    block->in_use = true;
    
    // 返回块中数据部分的指针
    return reinterpret_cast<char*>(block) + sizeof(Block);
}

void SimpleMemoryPool::deallocate(void* ptr) {
    if (!ptr) return;
    
    // 计算块头的指针
    Block* block = reinterpret_cast<Block*>(reinterpret_cast<char*>(ptr) - sizeof(Block));
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 将块添加回空闲链表
    block->in_use = false;
    block->next = free_list_;
    free_list_ = block;
}

// BEVCache实现
BEVCache::BEVCache(const BEVCacheConfig& config)
    : memory_pool_(config.memory_pool ? config.memory_pool : std::make_shared<SimpleMemoryPool>(1024)),
      max_cache_size_(config.max_cache_size)
{
}

BEVCache::~BEVCache() {
    // 清理缓存
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_map_.clear();
    lru_list_.clear();
}

void BEVCache::insertPackets(const std::vector<uint8_t>& compressed_data) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    const uint8_t* data_ptr = compressed_data.data();
    size_t remaining_size = compressed_data.size();
    
    // 读取数据包数量
    if (remaining_size < sizeof(uint32_t)) return;
    
    uint32_t num_packets = *reinterpret_cast<const uint32_t*>(data_ptr);
    data_ptr += sizeof(uint32_t);
    remaining_size -= sizeof(uint32_t);
    
    // 处理每个数据包
    for (uint32_t i = 0; i < num_packets && remaining_size > 0; ++i) {
        // 读取时间戳
        if (remaining_size < sizeof(uint64_t)) break;
        
        uint64_t timestamp = *reinterpret_cast<const uint64_t*>(data_ptr);
        data_ptr += sizeof(uint64_t);
        remaining_size -= sizeof(uint64_t);
        
        // 读取块数量
        if (remaining_size < sizeof(uint16_t)) break;
        
        uint16_t num_blocks = *reinterpret_cast<const uint16_t*>(data_ptr);
        data_ptr += sizeof(uint16_t);
        remaining_size -= sizeof(uint16_t);
        
        // 处理每个块
        for (uint16_t j = 0; j < num_blocks && remaining_size > 0; ++j) {
            // 读取块头
            if (remaining_size < 4 * sizeof(uint16_t)) break;
            
            uint16_t x = *reinterpret_cast<const uint16_t*>(data_ptr);
            data_ptr += sizeof(uint16_t);
            
            uint16_t y = *reinterpret_cast<const uint16_t*>(data_ptr);
            data_ptr += sizeof(uint16_t);
            
            uint16_t rows = *reinterpret_cast<const uint16_t*>(data_ptr);
            data_ptr += sizeof(uint16_t);
            
            uint16_t block_size = *reinterpret_cast<const uint16_t*>(data_ptr);
            data_ptr += sizeof(uint16_t);
            
            remaining_size -= 4 * sizeof(uint16_t);
            
            // 读取压缩数据
            if (remaining_size < block_size) break;
            
            // 创建缓存项
            BEVCacheItem item;
            item.timestamp = timestamp;
            item.x = x;
            item.y = y;
            item.rows = rows;
            item.compressed_data.assign(data_ptr, data_ptr + block_size);
            
            // 更新数据指针
            data_ptr += block_size;
            remaining_size -= block_size;
            
            // 生成键
            CacheKey key = {timestamp, x, y};
            
            // 检查是否已存在
            auto it = cache_map_.find(key);
            if (it != cache_map_.end()) {
                // 移除旧项
                lru_list_.erase(it->second.lru_iterator);
                cache_map_.erase(it);
            }
            
            // 如果缓存已满，移除最旧的项
            if (cache_map_.size() >= max_cache_size_) {
                evictOldestItem();
            }
            
            // 将新项添加到LRU链表尾部（最近使用）
            lru_list_.push_back(timestamp);
            item.lru_iterator = --lru_list_.end();
            
            // 添加到缓存映射
            cache_map_[key] = std::move(item);
        }
    }
}

bool BEVCache::retrieve(uint64_t timestamp, uint16_t x, uint16_t y, 
                       std::vector<uint8_t>& data, uint16_t& rows, uint16_t& cols) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    
    // 生成键
    CacheKey key = {timestamp, x, y};
    
    // 查找缓存项
    auto it = cache_map_.find(key);
    if (it == cache_map_.end()) {
        // 未命中
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        total_misses_++;
        return false;
    }
    
    // 命中
    {
        std::lock_guard<std::mutex> stats_lock(stats_mutex_);
        total_hits_++;
    }
    
    // 获取缓存项
    const BEVCacheItem& item = it->second;
    
    // 更新LRU链表（移到尾部表示最近使用）
    lru_list_.erase(item.lru_iterator);
    lru_list_.push_back(timestamp);
    const_cast<BEVCacheItem&>(item).lru_iterator = --lru_list_.end();
    
    // 返回数据
    data = item.compressed_data;
    rows = item.rows;
    cols = 16; // 假设列数固定为16，根据实际情况调整
    
    return true;
}

double BEVCache::getHitRate() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    uint64_t total = total_hits_ + total_misses_;
    return total > 0 ? static_cast<double>(total_hits_) / total : 0.0;
}

std::string BEVCache::getStatsAsJSON() const {
    std::lock_guard<std::mutex> lock(stats_mutex_);
    
    Json::Value root;
    root["total_hits"] = static_cast<Json::UInt64>(total_hits_);
    root["total_misses"] = static_cast<Json::UInt64>(total_misses_);
    root["hit_rate"] = getHitRate();
    root["cache_size"] = static_cast<Json::UInt64>(cache_map_.size());
    root["max_cache_size"] = static_cast<Json::UInt64>(max_cache_size_);
    
    Json::FastWriter writer;
    return writer.write(root);
}

void BEVCache::evictOldestItem() {
    if (lru_list_.empty()) return;
    
    // 获取最旧的时间戳
    uint64_t oldest_timestamp = lru_list_.front();
    
    // 查找最旧的项并移除
    for (auto it = cache_map_.begin(); it != cache_map_.end(); ++it) {
        if (it->second.timestamp == oldest_timestamp) {
            lru_list_.erase(it->second.lru_iterator);
            cache_map_.erase(it);
            break;
        }
    }
}    