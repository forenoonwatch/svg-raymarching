#pragma once

#include "bezier.hpp"

#include <array>

namespace Hatch {

using real_t = double;

enum class MajorAxis : uint32_t {
	MAJOR_X,
	MAJOR_Y,
};

struct Segment {
	const QuadraticBezier<real_t>* originalBezier{nullptr};
	// because beziers are broken down, depending on the type this is tStart or tEnd
	real_t tStart;
	real_t tEnd;

	std::array<real_t, 2> intersect(const Segment& other) const;
	// Checks if it's a straight line e.g. if you're sweeping along the y axis then it's a line parallel to x
	bool is_straight_line_constant_major() const;
};

constexpr const MajorAxis SELECTED_MAJOR_AXIS = MajorAxis::MAJOR_Y;
constexpr const MajorAxis SELECTED_MINOR_AXIS = MajorAxis::MAJOR_X;

//constexpr const MajorAxis SELECTED_MAJOR_AXIS = MajorAxis::MAJOR_X;
//constexpr const MajorAxis SELECTED_MINOR_AXIS = MajorAxis::MAJOR_Y;

constexpr real_t MINOR_POSITION_COMPARISON_THRESHOLD = real_t(1e-3);
constexpr real_t TANGENT_COMPARISON_THRESHOLD = real_t(1e-7);

bool split_into_major_monotonic_segments(const QuadraticBezier<real_t>& bezier,
		std::array<QuadraticBezier<real_t>, 2>& result);
std::array<real_t, 4> bezier_bezier_intersections(const QuadraticBezier<real_t>& bezier,
		const QuadraticBezier<real_t>& other);
real_t intersect_ortho(const QuadraticBezier<real_t>& bezier, real_t lineConstant, int major);

struct SegmentLess {
	int major = static_cast<int>(SELECTED_MAJOR_AXIS);
	int minor = 1 - static_cast<int>(SELECTED_MAJOR_AXIS);

	bool operator()(const Segment& a, const Segment& b) const;
};

}
