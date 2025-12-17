# 自动下载gtest
include(FetchContent)
#[[FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)
# 自动发现测试
include(GoogleTest)]]

# 先尝试查找本地已安装的 GoogleTest
find_package(GTest QUIET)

if(NOT GTest_FOUND)
    message(STATUS "GoogleTest not found locally, downloading from GitHub...")
    FetchContent_Declare(
            googletest
            GIT_REPOSITORY https://github.com/google/googletest.git
            GIT_TAG release-1.12.1
    )
    FetchContent_MakeAvailable(googletest)
else()
    message(STATUS "Using locally installed GoogleTest")
endif()
#[[gtest_discover_tests(tests
        EXTRA_ARGS --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results.xml
)]]
