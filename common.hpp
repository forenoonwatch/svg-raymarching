#pragma once

#include <cstddef>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(WIN64)
	#define OPERATING_SYSTEM_WINDOWS
#elif defined(__linux__)
	#define OPERATING_SYSTEM_LINUX
#elif defined(__APPLE__)
	#define OPERATING_SYSTEM_MACOS
#else
	#define OPERATING_SYSTEM_OTHER
#endif

#if defined(__clang__)
	#define COMPILER_CLANG
#elif defined(__GNUC__) || defined(__GNUG__)
	#define COMPILER_GCC
#elif defined(_MSC_VER)
	#define COMPILER_MSVC
#else
	#define COMPILER_OTHER
#endif

#ifdef COMPILER_MSVC
	#define ZN_FORCEINLINE __forceinline
	#define ZN_NEVERINLINE __declspec(noinline)
#elif defined(COMPILER_CLANG) || defined(COMPILER_GCC)
	#define ZN_FORCEINLINE inline __attribute__((always_inline))
	#define ZN_NEVERINLINE __attribute__((noinline))
#else
	#define ZN_FORCEINLINE inline
	#define ZN_NEVERINLINE
#endif

#if defined(COMPILER_GCC) || defined(COMPILER_CLANG)
#define ZN_UNREACHABLE __builtin_unreachable
#elif defined(COMPILER_MSVC)
#define ZN_UNREACHABLE() __assume(false)
#else
#define ZN_UNREACHABLE()
#endif

#if __has_cpp_attribute(fallthrough)
#define ZN_FALLTHROUGH [[fallthrough]]
#elif __has_cpp_attribute(clang::fallthrough)
#define ZN_FALLTHROUGH [[clang::fallthrough]]
#elif __has_cpp_attribute(gnu::fallthrough)
#define ZN_FALLTHROUGH [[gnu::fallthrough]]
#else
#define ZN_FALLTHROUGH
#endif

#define ZN_FORBID_HEAP_ALLOCATION()												\
	void* operator new(size_t, void*) = delete;									\
	void* operator new[](size_t, void*) = delete;								\
	void* operator new(size_t) = delete;										\
	void* operator new[](size_t) = delete

#define ZN_EVAL(...) __VA_ARGS__

#define ZN_CONCAT_IMPL2(prefix, suffix) prefix##suffix
#define ZN_CONCAT_IMPL(prefix, suffix) ZN_CONCAT_IMPL2(prefix, suffix)
#define ZN_CONCAT_LABEL(prefix, suffix) ZN_CONCAT_IMPL(ZN_EVAL(prefix), ZN_EVAL(suffix))

#define ZN_MAKE_UNIQUE_VARIABLE_NAME(prefix) ZN_CONCAT_LABEL(prefix##_, __LINE__)

#define NULL_COPY_AND_ASSIGN(ClassName)															\
	ClassName(const ClassName&) = delete;														\
	void operator=(const ClassName&) = delete;													\
	ClassName(ClassName&&) = delete;															\
	void operator=(ClassName&&) = delete

#define DEFAULT_COPY_AND_ASSIGN(ClassName)														\
	ClassName(const ClassName&) = default;														\
	ClassName& operator=(const ClassName&) = default;											\
	ClassName(ClassName&&) noexcept = default;													\
	ClassName& operator=(ClassName&&) noexcept = default

#define DEFAULT_MOVE_NULL_COPY(ClassName)														\
	ClassName(ClassName&&) noexcept = default;													\
	ClassName& operator=(ClassName&&) noexcept = default;										\
	ClassName(const ClassName&) = delete;														\
	void operator=(const ClassName&) = delete

#define ZN_DEFINE_UNARY_ENUM_OPERATOR(T, op)													\
	constexpr T operator op(const T& a) noexcept {												\
		static_assert(std::is_enum_v<T>);														\
		return static_cast<T>(op static_cast<std::underlying_type_t<T>>(a));					\
	}

#define ZN_DEFINE_BINARY_ENUM_OPERATOR(T1, T2, op)												\
	constexpr T1 operator op(const T1& a, const T2& b) noexcept {								\
		static_assert(std::is_enum_v<T1> && std::is_enum_v<T2>);								\
		return static_cast<T1>(static_cast<std::underlying_type_t<T1>>(a)						\
				op static_cast<std::underlying_type_t<T2>>(b));									\
	}

#define ZN_DEFINE_ASSIGNMENT_ENUM_OPERATOR(T1, T2, op)											\
	constexpr T1 operator op##=(T1& a, const T2& b) noexcept {									\
		static_assert(std::is_enum_v<T1> && std::is_enum_v<T2>);								\
		return a = static_cast<T1>(static_cast<std::underlying_type_t<T1>>(a)					\
				op static_cast<std::underlying_type_t<T2>>(b));									\
	}

#define ZN_DEFINE_ENUM_BITFLAG_OPERATORS(Enum)													\
	ZN_DEFINE_UNARY_ENUM_OPERATOR(Enum, ~)														\
	ZN_DEFINE_BINARY_ENUM_OPERATOR(Enum, Enum, |)												\
	ZN_DEFINE_BINARY_ENUM_OPERATOR(Enum, Enum, &)												\
	ZN_DEFINE_ASSIGNMENT_ENUM_OPERATOR(Enum, Enum, |)											\
	ZN_DEFINE_ASSIGNMENT_ENUM_OPERATOR(Enum, Enum, &)											\

constexpr size_t operator ""_uz(unsigned long long v) {
	return static_cast<size_t>(v);
}

