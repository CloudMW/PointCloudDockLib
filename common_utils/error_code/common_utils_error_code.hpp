//
// Created by Mengfanyong on 2025/12/9.
//
// PCDL Error Code Library - Portable and Configurable Error Handling
//

#ifndef POINTCLOUDDOCKLIB_COMMON_ERROR_CODE_H
#define POINTCLOUDDOCKLIB_COMMON_ERROR_CODE_H

#include <system_error>
#include <string>
#include <stdexcept>

namespace common_utils {
    namespace error_code {

        // ============================================================================
        // 错误代码枚举 - Error Code Enumerations
        // ============================================================================



        // 算法相关错误 (0x8002xx)
        enum class AlgoCode {
            Ok = 0,
            EMPTY_POINT_CLOUD = 0x800201,    // 空点云
            INVALID_PARAMETER = 0x800202,     // 无效参数
            SEGMENTATION_FAILED = 0x800203,   // 分割失败
            CLUSTERING_FAILED = 0x800204,     // 聚类失败
            FITTING_FAILED = 0x800205,        // 拟合失败
            CONVERGENCE_FAILED = 0x800206,    // 收敛失败
            INSUFFICIENT_POINTS = 0x800207,   // 点数不足
            DIMENSION_MISMATCH = 0x800208     // 维度不匹配
        };

        // ============================================================================
        // Error Category 实现 - Error Category Implementations
        // ============================================================================
        // 算法错误类别
        class AlgoErrorCategory : public std::error_category {
        public:
            const char* name() const noexcept override {
                return "algorithm";
            }

            std::string message(int ev) const override {
                switch (static_cast<AlgoCode>(ev)) {
                    case AlgoCode::Ok:
                        return "Success";
                    case AlgoCode::EMPTY_POINT_CLOUD:
                        return "Empty point cloud";
                    case AlgoCode::INVALID_PARAMETER:
                        return "Invalid parameter";
                    case AlgoCode::SEGMENTATION_FAILED:
                        return "Segmentation failed";
                    case AlgoCode::CLUSTERING_FAILED:
                        return "Clustering failed";
                    case AlgoCode::FITTING_FAILED:
                        return "Fitting failed";
                    case AlgoCode::CONVERGENCE_FAILED:
                        return "Convergence failed";
                    case AlgoCode::INSUFFICIENT_POINTS:
                        return "Insufficient points for operation";
                    case AlgoCode::DIMENSION_MISMATCH:
                        return "Dimension mismatch";
                    default:
                        return "Unknown algorithm error";
                }
            }
        };



        // ============================================================================
        // Category 单例访问函数 - Category Singleton Accessors
        // ============================================================================

        inline const AlgoErrorCategory& algo_category() {
            static AlgoErrorCategory instance;
            return instance;
        }

        // ============================================================================
        // make_error_code 函数 - Error Code Creation Functions
        // ============================================================================


        inline std::error_code make_error_code(AlgoCode e) {
            return {static_cast<int>(e), algo_category()};
        }

        // ============================================================================
        // 辅助函数和类 - Helper Functions and Classes
        // ============================================================================

        // 错误结果包装类 - 用于返回值和错误码
        template<typename T>
        class Result {
        private:
            T value_;
            std::error_code error_;
            bool has_value_;

        public:
            Result(const T& val) : value_(val), has_value_(true) {}
            Result(T&& val) : value_(std::move(val)), has_value_(true) {}
            Result(std::error_code ec) : error_(ec), has_value_(false) {}

            template<typename ErrorEnum>
            Result(ErrorEnum e) : error_(make_error_code(e)), has_value_(false) {}

            bool ok() const { return has_value_ && !error_; }
            explicit operator bool() const { return ok(); }

            const T& value() const {
                if (!has_value_) {
                    throw std::runtime_error("Result has no value: " + error_.message());
                }
                return value_;
            }

            T& value() {
                if (!has_value_) {
                    throw std::runtime_error("Result has no value: " + error_.message());
                }
                return value_;
            }

            const std::error_code& error() const { return error_; }

            T value_or(const T& default_value) const {
                return has_value_ ? value_ : default_value;
            }
        };

        // void 特化
        template<>
        class Result<void> {
        private:
            std::error_code error_;

        public:
            Result() : error_() {}
            Result(std::error_code ec) : error_(ec) {}

            template<typename ErrorEnum>
            Result(ErrorEnum e) : error_(make_error_code(e)) {}

            bool ok() const { return !error_; }
            explicit operator bool() const { return ok(); }

            const std::error_code& error() const { return error_; }
        };

        // 异常类 - 基于 std::system_error
        class CommonException : public std::system_error {
        public:
            template<typename ErrorEnum>
            explicit CommonException(ErrorEnum e, const std::string& what_arg = "")
                : std::system_error(make_error_code(e), what_arg) {}

            CommonException(std::error_code ec, const std::string& what_arg = "")
                : std::system_error(ec, what_arg) {}
        };

        // 便捷的检查宏
        #define ALGO_CHECK(condition, error_code_) \
            if (!(condition)) { \
                throw common_utils::error_code::CommonException(error_code_, \
                    std::string(__FILE__) + ":" + std::to_string(__LINE__)); \
            }

    } // namespace error_code
} // namespace pcdl

// ============================================================================
// std::error_code 特化 - Enable implicit conversion
// ============================================================================
namespace std {
    template<>
    struct is_error_code_enum<common_utils::error_code::AlgoCode> : true_type {};
}

#endif //POINTCLOUDDOCKLIB_PCDL_ERROR_CODE_H