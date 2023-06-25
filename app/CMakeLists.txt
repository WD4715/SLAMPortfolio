# fmt
find_package(FMT REQUIRED)
include_directories(${FMT_INCLUDE_DIRS})

add_executable(run_kitti_stereo run_kitti_stereo.cpp)

target_link_libraries(run_kitti_stereo fmt::fmt myslam ${THIRD_PARTY_LIBS} )