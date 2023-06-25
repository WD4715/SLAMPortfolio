find_package(FMT REQUIRED)
include_directories(${FMT_INCLUDE_DIRS})


add_library(myslam SHARED
        frame.cpp
        mappoint.cpp
        map.cpp
        camera.cpp
        config.cpp
        feature.cpp
        frontend.cpp
        backend.cpp
        viewer.cpp
        visual_odometry.cpp
        dataset.cpp)

target_link_libraries(myslam
        fmt::fmt
        ${THIRD_PARTY_LIBS})
