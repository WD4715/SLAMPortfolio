SET(TEST_SOURCES test_triangulation)
# fmt
find_package(FMT REQUIRED)
include_directories(${FMT_INCLUDE_DIRS})




FOREACH (test_src ${TEST_SOURCES})
    ADD_EXECUTABLE(${test_src} ${test_src}.cpp)
    TARGET_LINK_LIBRARIES(${test_src}
                            fmt::fmt
                            ${THIRD_PARTY_LIBS}
                          myslam)

    ADD_TEST(${test_src} ${test_src})
ENDFOREACH (test_src)
