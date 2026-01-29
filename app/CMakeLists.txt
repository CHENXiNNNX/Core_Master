# App module CMakeLists.txt

# 收集当前目录及子目录中的所有代码文件
file(GLOB_RECURSE APP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp"
)

# 创建 app 静态库
add_library(app STATIC ${APP_SOURCES})

# opencv 依赖
set(OpenCV_DIR "${PROJECT_SOURCE_DIR}/third_party/opencv_rv1106_410/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(app ${OpenCV_LIBS})

# samplerate 依赖
include_directories(${SAMPLERATE_INCLUDE_DIRS})
target_link_libraries(app ${SAMPLERATE_LIBRARIES})

# speexdsp 依赖
target_include_directories(app PRIVATE ${SPEEX_INCLUDE_DIRS})
target_link_directories(app PRIVATE ${SPEEX_LIBRARY_DIRS})
target_link_libraries(app speexdsp)

# snowboy 唤醒词检测依赖
target_include_directories(app PRIVATE ${SNOWBOY_INCLUDE_DIRS})
target_link_libraries(app ${SNOWBOY_LIBRARY})

# 设置头文件包含目录
target_include_directories(app PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    # media headers
    ${CMAKE_CURRENT_SOURCE_DIR}/media
    ${CMAKE_CURRENT_SOURCE_DIR}/media/audio
    ${CMAKE_CURRENT_SOURCE_DIR}/media/camera
    # protocol headers
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol
    ${CMAKE_CURRENT_SOURCE_DIR}/protocol/rtsp
    # tool headers
    ${CMAKE_CURRENT_SOURCE_DIR}/tool
    ${CMAKE_CURRENT_SOURCE_DIR}/tool/file
    ${CMAKE_CURRENT_SOURCE_DIR}/tool/log
    ${CMAKE_CURRENT_SOURCE_DIR}/tool/mac
    ${CMAKE_CURRENT_SOURCE_DIR}/tool/memory
    ${CMAKE_CURRENT_SOURCE_DIR}/tool/memory/double_buffer
    ${CMAKE_CURRENT_SOURCE_DIR}/tool/memory/ring_buffer
    ${CMAKE_CURRENT_SOURCE_DIR}/tool/time
    # rknn headers
    ${CMAKE_CURRENT_SOURCE_DIR}/rknn
    ${CMAKE_CURRENT_SOURCE_DIR}/rknn/face_detection
    ${CMAKE_CURRENT_SOURCE_DIR}/rknn/face_recognition
    ${PROJECT_SOURCE_DIR}/third_party/rknpu2/include
    # SDK headers
    ${SDK_PATH}/media/rockit/out/include
    ${SDK_PATH}/media/isp/out/include
    ${SDK_PATH}/media/isp/out/include/rkaiq
    ${SDK_PATH}/media/isp/out/include/rkaiq/uAPI2
    ${SDK_PATH}/media/isp/out/include/rkaiq/common
    ${SDK_PATH}/media/isp/out/include/rkaiq/xcore
    ${SDK_PATH}/media/isp/out/include/rkaiq/algos
    ${SDK_PATH}/media/isp/out/include/rkaiq/iq_parser
    ${SDK_PATH}/media/isp/out/include/rkaiq/iq_parser_v2
    ${SDK_PATH}/media/samples/example/common
    ${SDK_PATH}/media/samples/example/common/isp3.x
)

# 编译选项 
target_compile_options(app PRIVATE
    -Wall
    -Wextra
)