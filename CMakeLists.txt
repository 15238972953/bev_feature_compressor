cmake_minimum_required(VERSION 3.12)
project(bev_feature_compressor LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)

# 外部库
# set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3")
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# set(JSONCPP_INCLUDE_DIR "/usr/include/jsoncpp")
find_package(jsoncpp REQUIRED NO_MODULE)

set(OpenMP_C_FOUND TRUE)
# set(ZFP_INCLUDE_DIRS "/usr/local/include/zfp")
find_package(OpenMP REQUIRED)
find_package(zfp REQUIRED)


# 主库目标
add_library(bev_cache_lib SHARED
    src/cache_system.cpp
    src/compressor.cpp
    src/main.cpp
    src/utils.cpp
)

add_executable(bev_cache 
    src/main.cpp
    src/cache_system.cpp
    src/compressor.cpp
    src/utils.cpp
)

add_executable(GenerateData
    src/GenerateData.cpp
)

target_include_directories(bev_cache PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${JSONCPP_INCLUDE_DIR}
    ${ZFP_INCLUDE_DIRS}
)

target_include_directories(GenerateData PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${JSONCPP_INCLUDE_DIR}
    ${ZFP_INCLUDE_DIRS}
)

target_link_libraries(bev_cache PUBLIC 
    zfp::zfp
    Eigen3::Eigen
    JsonCpp::JsonCpp
    OpenMP::OpenMP_CXX
)

target_link_libraries(GenerateData PUBLIC 
    zfp::zfp
    Eigen3::Eigen
    JsonCpp::JsonCpp
    OpenMP::OpenMP_CXX
)