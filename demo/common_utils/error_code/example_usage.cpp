//
// Common Error Code Library - Usage Examples
// 展示如何在不同场景下使用错误库
//

#include "error_code/common_utils_error_code.hpp"
#include <iostream>
#include <fstream>
#include <vector>

using namespace common_utils::error_code;

// ============================================================================
// 示例 1: 基本的错误码使用
// ============================================================================
void example1_basic_error_code() {
    std::cout << "\n=== Example 1: Basic Error Code Usage ===\n";

    // 创建错误码
    std::error_code ec = make_error_code(AlgoCode::CLUSTERING_FAILED);

    // 检查错误
    if (ec) {
        std::cerr << "Error occurred: " << ec.message() << "\n";
        std::cerr << "Error category: " << ec.category().name() << "\n";
        std::cerr << "Error value: 0x" << std::hex << ec.value() << std::dec << "\n";
    }

    // 创建成功的错误码
    std::error_code ok = make_error_code(AlgoCode::Ok);
    if (!ok) {
        std::cout << "Operation successful!\n";
    }
}

// ============================================================================
// 示例 2: 使用 Result 类返回值或错误
// ============================================================================
Result<std::vector<double>> readDataFromFile(const std::string& filename) {

    std::vector<double> data;

    if (data.empty()) {
        // 返回算法错误
        return AlgoCode::EMPTY_POINT_CLOUD;
    }

    // 返回成功结果
    return data;
}

void example2_result_pattern() {
    std::cout << "\n=== Example 2: Result Pattern ===\n";

    // 尝试读取不存在的文件
    auto result = readDataFromFile("nonexistent.txt");

    if (result.ok()) {
        std::cout << "Read " << result.value().size() << " points\n";
    } else {
        std::cerr << "Failed to read file: " << result.error().message() << "\n";
    }

    // 使用 value_or 提供默认值
    auto data = result.value_or(std::vector<double>{1.0, 2.0, 3.0});
    std::cout << "Using " << data.size() << " points (with fallback)\n";
}

// ============================================================================
// 示例 3: 使用异常处理
// ============================================================================
void processPointCloud(const std::vector<double>& points) {
    // 参数验证
    if (points.empty()) {
        throw CommonException(AlgoCode::EMPTY_POINT_CLOUD, "Point cloud is empty");
    }

    if (points.size() < 3) {
        throw CommonException(AlgoCode::INSUFFICIENT_POINTS,
                           "Need at least 3 points, got " + std::to_string(points.size()));
    }

    std::cout << "Processing " << points.size() << " points...\n";
}

void example3_exception_handling() {
    std::cout << "\n=== Example 3: Exception Handling ===\n";

    try {
        std::vector<double> emptyCloud;
        processPointCloud(emptyCloud);
    } catch (const CommonException& e) {
        std::cerr << "Common Exception: " << e.what() << "\n";
        std::cerr << "Error code: 0x" << std::hex << e.code().value() << std::dec << "\n";
        std::cerr << "Category: " << e.code().category().name() << "\n";
    } catch (const std::exception& e) {
        std::cerr << "Standard exception: " << e.what() << "\n";
    }
}

// ============================================================================
// 示例 4: 使用宏进行参数验证
// ============================================================================
Result<double> calculateMean(const std::vector<double>& data) {
    // 使用宏进行检查
    try {
        ALGO_CHECK(!data.empty(), AlgoCode::EMPTY_POINT_CLOUD);

        double sum = 0.0;
        for (double val : data) {
            sum += val;
        }

        return sum / data.size();
    } catch (const CommonException& e) {
        return e.code();
    }
}

void example4_macro_usage() {
    std::cout << "\n=== Example 4: Macro Usage ===\n";

    std::vector<double> data = {1.0, 2.0, 3.0, 4.0, 5.0};
    auto result = calculateMean(data);

    if (result) {
        std::cout << "Mean: " << result.value() << "\n";
    } else {
        std::cerr << "Error: " << result.error().message() << "\n";
    }

    // 测试空数据
    std::vector<double> emptyData;
    auto emptyResult = calculateMean(emptyData);
    if (!emptyResult) {
        std::cerr << "Expected error: " << emptyResult.error().message() << "\n";
    }
}





// ============================================================================
// 示例 7: 自定义错误处理策略
// ============================================================================
class ErrorHandler {
public:
    template<typename T>
    static T handleResult(Result<T> result, const T& fallback) {
        if (result.ok()) {
            return result.value();
        }

        // 记录错误
        logError(result.error());

        // 返回备用值
        return fallback;
    }

private:
    static void logError(const std::error_code& ec) {
        std::cerr << "[ERROR] " << ec.category().name()
                  << ": " << ec.message()
                  << " (code: 0x" << std::hex << ec.value() << std::dec << ")\n";
    }
};

void example7_custom_error_handling() {
    std::cout << "\n=== Example 7: Custom Error Handling Strategy ===\n";

    auto result = readDataFromFile("missing.txt");

    // 使用自定义错误处理器
    auto data = ErrorHandler::handleResult(result, std::vector<double>{0.0, 0.0, 0.0});

    std::cout << "Got " << data.size() << " points (with custom handler)\n";
}


// ============================================================================
// Main
// ============================================================================
int main() {
    std::cout << "========================================\n";
    std::cout << "Common Utils Error Code Library - Usage Examples\n";
    std::cout << "========================================\n";

    try {
        example1_basic_error_code();
        example2_result_pattern();
        example3_exception_handling();
        example4_macro_usage();

        example7_custom_error_handling();


    } catch (const std::exception& e) {
        std::cerr << "\nUnexpected error: " << e.what() << "\n";
        return 1;
    }

    std::cout << "\n========================================\n";
    std::cout << "All examples completed!\n";
    std::cout << "========================================\n";

    return 0;
}

