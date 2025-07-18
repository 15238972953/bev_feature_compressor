#ifndef BEV_CACHE_H
#define BEV_CACHE_H

#include <json/json.h>
#include <vector>
#include <list>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <eigen3/Eigen/Dense>

// 内存池接口
class MemoryPool {
public:
    virtual ~MemoryPool() = default;
    virtual void* allocate(size_t size) = 0;
    virtual void deallocate(void* ptr) = 0;
};

// 简单内存池实现
class SimpleMemoryPool : public MemoryPool {
public:
    explicit SimpleMemoryPool(size_t block_size, size_t initial_blocks = 1024);
    ~SimpleMemoryPool() override;
    
    void* allocate(size_t size) override;
    void deallocate(void* ptr) override;
    
private:
    struct Block {
        bool in_use;
        Block* next;
    };
    
    size_t block_size_;
    Block* free_list_;
    std::vector<char*> allocated_chunks_;

    mutable std::mutex mutex_;
};

// BEV缓存项
struct BEVCacheItem {
    uint64_t timestamp;
    uint16_t x;
    uint16_t y;
    uint16_t rows;
    std::vector<uint8_t> compressed_data;
    
    // 用于LRU链表的迭代器
    using LRUIterator = std::list<uint64_t>::iterator;
    LRUIterator lru_iterator;
};

// BEV缓存系统
class BEVCache {
public:
    // BEV缓存配置
    struct BEVCacheConfig {
        size_t max_cache_size = 1024; // 最大缓存项数
        std::shared_ptr<MemoryPool> memory_pool; // 内存池
    };

    explicit BEVCache(const BEVCacheConfig& config);
    ~BEVCache();
    
    // 插入压缩数据包
    void insertPackets(const std::vector<uint8_t>& compressed_data);
    
    // 检索缓存项
    bool retrieve(uint64_t timestamp, uint16_t x, uint16_t y, 
                  std::vector<uint8_t>& data, uint16_t& rows, uint16_t& cols);
    
    // 获取缓存命中率
    double getHitRate() const;
    
    // 获取统计信息JSON
    std::string getStatsAsJSON() const;
    
private:
    // 缓存项的键
    struct CacheKey {
        uint64_t timestamp;
        uint16_t x;
        uint16_t y;
        
        bool operator==(const CacheKey& other) const {
            return timestamp == other.timestamp && x == other.x && y == other.y;
        }
    };
    
    // 哈希函数
    struct CacheKeyHash {
        std::size_t operator()(const CacheKey& key) const {
            return (key.timestamp << 32) | (key.x << 16) | key.y;
        }
    };
    
    // 解析压缩数据并插入缓存
    void parseAndInsertPacket(const uint8_t* data, size_t size);
    
    // 从缓存中移除最旧的项
    void evictOldestItem();
    
    // 内存池分配器
    std::shared_ptr<MemoryPool> memory_pool_;
    
    // 缓存存储
    std::unordered_map<CacheKey, BEVCacheItem, CacheKeyHash> cache_map_;
    
    // LRU链表（最近使用的在尾部）
    std::list<uint64_t> lru_list_;
    
    // 缓存配置
    size_t max_cache_size_;
    
    // 统计信息
    mutable std::mutex stats_mutex_;
    uint64_t total_hits_ = 0;
    uint64_t total_misses_ = 0;
    
    // 互斥锁
    mutable std::mutex cache_mutex_;
};

#endif // BEV_CACHE_H    