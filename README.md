# BEV特征数据压缩与缓存系统开发
## 任务背景
BEV感知模型在实车运行时需处理高频率的BEV特征图（如10Hz的256x256浮点矩阵），导致：  
**内存压力大：** 单帧特征图占用2.5MB，10秒数据即需250MB  
**数据传输慢：** 车端到云端的数据回传带宽有限  
需开发一套基于C++的 特征数据压缩与缓存系统，支持：  
1. 实时压缩BEV特征图  
2. 智能缓存高频访问数据  
3. 兼容现有训练/推理管线

## 项目内容  
BEV特征压缩模块  
实现基于**Eigen矩阵块划分**的压缩算法：  
将256x256矩阵分块为16x16子矩阵  
对每个子矩阵应用 ZFP浮点压缩算法（开源库集成）  
压缩比可控（默认目标：5:1），支持无损/有损模式  
智能缓存系统  
设计 LRU缓存策略，自动保留高频访问的特征图块  
使用 内存池技术 避免频繁内存分配/释放  
提供API供算法模块快速检索历史特征（如get_bev_feature(timestamp)）  
性能统计与日志  
记录压缩率、缓存命中率等指标  
通过 JSON日志 输出运行时统计信息  

技术栈
核心语言：C++17（移动语义、智能指针）  
数学库：Eigen（矩阵分块操作）  
压缩算法：ZFP（或自定义差分编码+DCT变换）  
性能工具：Google Benchmark测试吞吐量  
```
bev_feature_compressor/
├── include/                  # 头文件
│   ├── compressor.h          # 压缩模块接口
│   ├── cache_system.h        # 缓存系统接口
│   └── utils.h               # 工具函数（时间统计等）
├── src/
│   ├── compressor.cpp        # 压缩模块实现
│   ├── cache_system.cpp      # 缓存系统实现
│   ├── main.cpp              # 单元测试入口
│   └── utils.cpp             # 工具函数实现
├── third_party/             # 第三方库
│   └── zfp/                 # ZFP压缩库
├── test/                    # 测试代码
│   ├── test_compressor.cpp   # 压缩模块测试
│   └── test_cache.cpp        # 缓存系统测试
├── config/                  # 配置文件
│   └── params.json          # 压缩比/缓存大小等参数
└── build/                   # 编译输出
```

## 踩坑
1. **Eigen库的安装** 
Eigen库明明已经安装好了，但是因为eigen3 被默认安装到了usr/include里了（是系统默认的路径），在很多程序中include时经常使用`#include <Eigen/Dense>`而不是使用`#include <eigen3/Eigen/Dense>`所以要做下处理，否则一些程序在编译时会因找不到`Eigen/Dense`而报错。
```bash
方法一：将usr/local/include/eigen3文件夹中的Eigen文件递归地复制到上一层文件夹（直接放到/usr/local/include中，否则系统无法默认搜索到
cp -r /usr/local/include/eigen3/Eigen /usr/local/include

方法二：在文件中直接这样写
#include <eigen3/Eigen/Dense>
```
