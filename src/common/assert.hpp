#pragma once

#include <string_view>

#if !__cpp_lib_source_location

namespace detail {

struct SourceLocation {
    [[nodiscard]] std::string_view file_name() const { return file_; }
    [[nodiscard]] std::uint32_t line() const { return line_; }
    [[nodiscard]] std::string_view function_name() const { return func_; }

    std::string_view file_;
    std::uint32_t line_;
    std::string_view func_;
};

} // namespace detail

#define NG_CURRENT_LOCATION \
    detail::SourceLocation { __FILE__, __LINE__, __PRETTY_FUNCTION__ }

#else
#include <source_location>
namespace detail {

using SourceLocation = std::source_location;

}
#define NG_CURRENT_LOCATION std::source_location::current()
#endif

namespace detail {

template<typename... Ts>
[[noreturn]] void panic(SourceLocation loc, std::string_view message, Ts&&... ts)
{
    spdlog::critical(
        "Panicked at {}:{} ({})",
        loc.file_name(),
        loc.line(),
        loc.function_name());
    spdlog::critical(message, ts...);
    std::abort();
}

} // namespace detail

#define NG_PANIC(msg, ...)                                        \
    do {                                                          \
        detail::panic(NG_CURRENT_LOCATION, (msg), ##__VA_ARGS__); \
    } while (false)

// Fires in all builds
#define NG_VERIFYF(cond, msg, ...)        \
    do {                                  \
        if (!(cond)) {                    \
            NG_PANIC(msg, ##__VA_ARGS__); \
        }                                 \
    } while (false)

#define NG_VERIFY(cond) NG_VERIFYF(cond, "")

// Fires only in the debug build
#ifdef NDEBUG
#define NG_ASSERTF(cond, msg) \
    do {                      \
        ((void)(cond));       \
        ((void)(msg));        \
    } while (false)
#else
#define NG_ASSERTF(cond, msg, ...) NG_VERIFYF(cond, msg, ##__VA_ARGS__)
#endif

#define NG_ASSERT(cond) NG_ASSERTF(cond, "")
