//
// Created by Mengfanyong on 2025/12/11.
//
#include "common_utils/time/common_utils_timer.hpp"
#include <thread>

#include "time/common_utils_timer.hpp"
using namespace common_utils;
void example_usage() {
    TimerUtils timer;

    // 方式1: 手动start/stop
    timer.start("preprocessing");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    timer.stop("preprocessing");

    timer.start("computation");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    timer.stop("computation");

    timer.start("postprocessing");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    timer.stop("postprocessing");

    // 打印摘要
    timer.printSummary();

    // 获取特定阶段时间
    std::cout << "\nComputation time: "
              << timer.getStageTime("computation") << " ms" << std::endl;

    std::cout << "Total time: "
              << timer.getTotalTime() << " ms" << std::endl;
}

void example_scoped_timer() {
    TimerUtils timer;

    // 方式2: 使用RAII风格(更安全,自动stop)
    {
        TimerUtils::ScopedTimer st(timer, "stage1");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } // 自动stop

    {
        TimerUtils::ScopedTimer st(timer, "stage2");
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
    }

    timer.printSummary();
}

int main() {
    example_usage();
    example_scoped_timer();
    return 0;
}