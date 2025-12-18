
# 先尝试查找本地已安装的 GoogleTest
find_package(GTest QUIET)

if(NOT GTest_FOUND)
    # 自动下载gtest
    include(FetchContent)
    message(STATUS "GoogleTest not found locally, downloading from GitHub...")
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/refs/heads/v1.16.x.zip
    )
    FetchContent_MakeAvailable(googletest)
else()
    message(STATUS "Using locally installed GoogleTest")
endif()


set(SUBSYS_NAME demo_gtest)

add_executable(${SUBSYS_NAME})

target_sources(${SUBSYS_NAME} PRIVATE main.cpp)
target_include_directories(${SUBSYS_NAME} PRIVATE
        ${CMAKE_SOURCE_DIR}/)
target_link_libraries(${SUBSYS_NAME} PRIVATE
        ${PCL_LIBRARIES}
        spdlog::spdlog
        GTest::gtest # GoogleTest 库
       )

# 自动发现测试
#[[
include(GoogleTest)
gtest_discover_tests(${SUBSYS_NAME})
enable_testing()
]]


# 为target 设置 宏
target_compile_definitions(${SUBSYS_NAME} PRIVATE
        # PRIVATE：仅当前目标可见；PUBLIC：目标和依赖它的目标可见；INTERFACE：仅依赖它的目标可见
        DEMO_PATH="${CMAKE_CURRENT_SOURCE_DIR}/../"

)

