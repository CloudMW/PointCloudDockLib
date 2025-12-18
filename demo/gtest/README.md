# Gtest
# 资料
https://google.github.io/googletest/
https://ggdocs.cn/googletest/


# CmakeLists.txt 示例
```cmake

# 安装 GoogleTest
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


# 链接
target_link_libraries(${PROJECT_NAME} PRIVATE
        # GTtest::gtest 需要代码中写入main函数，并在main函数中添加需要测试的函数，
        # GTest::gtest_main 则自带main函数，可以直接运行测试用例
        GTest::gtest # GoogleTest 库
        GTest::gtest_main # GoogleTest 主函数库
)

```