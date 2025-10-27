cmake_minimum_required(VERSION 3.22)
project(imgproc)

add_library(imgproc SHARED
        jni/native_analyzer.cpp
        core/filter_min.cpp
)

target_include_directories(imgproc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(imgproc PRIVATE cxx_std_17)
target_link_libraries(imgproc PRIVATE log)

# 余計なシンボルは非公開（JNIだけ公開）
set_target_properties(imgproc PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN YES
)
