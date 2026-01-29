file(GLOB_RECURSE COMMON_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"  # 查找当前目录及子目录中的所有 .c 文件
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"  # 查找当前目录及子目录中的所有 .cc 文件
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"  # 查找当前目录及子目录中的所有 .cpp 文件
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"  # 查找当前目录及子目录中的所有 .h 文件
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"  # 查找当前目录及子目录中的所有 .hpp 文件
)

# 定义 common 模块为一个静态库
add_library(common STATIC ${COMMON_SOURCES})

# opencv 依赖
set(OpenCV_DIR "${PROJECT_SOURCE_DIR}/third_party/opencv_rv1106_410/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(common ${OpenCV_LIBS})

# 设置头文件包含目录
target_include_directories(common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/app
    ${CMAKE_CURRENT_SOURCE_DIR}/spi
    ${CMAKE_CURRENT_SOURCE_DIR}/uart
    ${CMAKE_CURRENT_SOURCE_DIR}/uart/module
    ${CMAKE_CURRENT_SOURCE_DIR}/uart/message
)

# 编译选项 
target_compile_options(common PRIVATE
    -Wall
    -Wextra
)
