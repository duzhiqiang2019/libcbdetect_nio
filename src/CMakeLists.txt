cmake_minimum_required(VERSION 2.8)

find_package(OpenCV)
include_directories(${OpenCV})
include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(example example.cc)
#target_link_libraries(example  ${OpenCV_LIBS})
target_link_libraries(example cbdetect ${OpenCV_LIBS})
