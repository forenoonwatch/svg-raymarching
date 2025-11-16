#pragma once

#include "equations.hpp"
#include "bezier.hpp"

template <typename float_t>
struct Subdivision {
	template <typename Curve, typename AddCurveFunctor>
	void adaptive(const Curve& curve, float_t min, float_t max, float_t targetMaxError,
			AddCurveFunctor&& fnAddCurve, unsigned maxDepth = 12);

	template <typename AddCurveFunctor>
	void adaptive(const EllipticalArcInfo<float_t>& arc, float_t min, float_t max, float_t targetMaxError,
			AddCurveFunctor&& fnAddCurve, unsigned maxDepth = 12);

	private:
		template <typename Curve, typename AddCurveFunctor>
		void adaptive_impl(const Curve& curve, float_t min, float_t max, float_t targetMaxError,
				AddCurveFunctor&& fnAddCurve, unsigned depth);
};

template <typename float_t>
template <typename Curve, typename AddCurveFunctor>
void Subdivision<float_t>::adaptive(const Curve& curve, float_t min, float_t max,
		float_t targetMaxError, AddCurveFunctor&& fnAddCurve, unsigned maxDepth) {
	// if no inflection point then this will return NaN and the adaptive subdivision
	// will continue as normal (from min to max)
	auto inflectX = curve.compute_inflection_point(targetMaxError);

	if (inflectX > min && inflectX < max) {
		adaptive_impl(curve, min, inflectX, targetMaxError, fnAddCurve, maxDepth);
		adaptive_impl(curve, inflectX, max, targetMaxError, fnAddCurve, maxDepth);
	}
	else {
		adaptive_impl(curve, min, max, targetMaxError, fnAddCurve, maxDepth);
	}
}

template <typename float_t>
template <typename AddCurveFunctor>
void Subdivision<float_t>::adaptive(const EllipticalArcInfo<float_t>& arc, float_t min, float_t max,
		float_t targetMaxError, AddCurveFunctor&& fnAddCurve, unsigned maxDepth) {
	using vec2 = glm::vec<2, float_t>;

	auto lengthMajor = glm::length(arc.majorAxis);
	auto lengthMinor = lengthMajor * arc.eccentricity;
	auto normalizedMajor = arc.majorAxis / lengthMajor;

	glm::mat<2, 2, float_t> rotate{
		normalizedMajor,
		vec2(-normalizedMajor.y, normalizedMajor.x)
	};

	auto fnAddTransformedBezier = [&](QuadraticBezier<float_t>&& bezier) {
		bezier.P0 = rotate * bezier.P0 + arc.center;
		bezier.P1 = rotate * bezier.P1 + arc.center;
		bezier.P2 = rotate * bezier.P2 + arc.center;

		fnAddCurve(std::move(bezier));
	};

	if (arc.angleBounds.x != arc.angleBounds.y) {
		AxisAlignedEllipse<float_t> aaEllipse(lengthMajor, lengthMinor, arc.angleBounds.x, arc.angleBounds.y);
		adaptive(aaEllipse, float_t(0.0), float_t(1.0), targetMaxError, fnAddTransformedBezier, maxDepth);
	}
}

// Fix bezier hack for when P1 is "outside" P0->P2
// Wep roject P1 onto the line P0->P2 and see whether it lies inside.
// Because our curves shouldn't go back on themselves in the direction of the chord
template <typename float_t>
void fix_bezier_midpoint(QuadraticBezier<float_t>& bezier) {
	auto localChord = bezier.P2 - bezier.P0;
	auto localX = glm::dot(glm::normalize(localChord), bezier.P1 - bezier.P0);
	bool outside = localX < 0 || localX > glm::length(localChord);

	if (outside || std::isnan(bezier.P1.x) || std::isnan(bezier.P1.y)) {
		// this shouldn't happen but we fix it just in case anyways
		bezier.P1 = bezier.P0 * float_t(0.4) + bezier.P2 * float_t(0.6);
	}
}

template <typename float_t>
template <typename Curve, typename AddCurveFunctor>
void Subdivision<float_t>::adaptive_impl(const Curve& curve, float_t min, float_t max,
		float_t targetMaxError, AddCurveFunctor&& fnAddCurve, unsigned depth) {
	if (min == max) {
		return;
	}

	auto split = curve.inverse_arc_len_bisection_search(float_t(0.5), min, max);

	// shouldn't happen but may happen if we use NewtonRaphson for non-convergent inverse CDF
	if (split <= min || split >= max) {
		split = (min + max) / float_t(2.0);
	}

	auto P0 = curve.compute_position(min);
	auto V0 = curve.compute_tangent(min);
	auto P2 = curve.compute_position(max);
	auto V2 = curve.compute_tangent(max);

	auto bezier = QuadraticBezier<float_t>::from_two_points_and_tangents(P0, V0, P2, V2);

	bool shouldSubdivide = false;

	// FIXME: compare with threshold
	if (depth > 0u && glm::normalize(V0) == glm::normalize(V2)) {
		shouldSubdivide = true;
	}
	else {
		fix_bezier_midpoint(bezier);

		if (depth > 0u) {
			auto posAtSplit = curve.compute_position(split);

			if (glm::distance(P0, P2) < targetMaxError) {
				// If it came down to a bezier small enough that it causes P0 P2 and the position split
				// to be smaller than maxTargetError then we stop
				if (glm::distance(posAtSplit, P0) < targetMaxError) {
					shouldSubdivide = false;
				}
				// But sometimes when P0 and P2 are close together a split will fix them, like a full circle,
				// and needs further subdivision
				else {
					shouldSubdivide = true;
				}
			}
			else {
				using vec2 = glm::vec<2, float_t>;

				auto horizontalAxis = glm::normalize(P2 - P0);
				auto splitPosLocal = P0 + vec2(glm::dot(posAtSplit - P0, horizontalAxis),
						glm::dot(posAtSplit - P0, vec2(-horizontalAxis.y, horizontalAxis.x)));
				auto P1local = P0 + vec2(glm::dot(bezier.P1 - P0, horizontalAxis),
						glm::dot(bezier.P1 - P0, vec2(-horizontalAxis.y, horizontalAxis.x)));
				auto P2local = P0 + vec2(glm::dot(bezier.P2 - P0, horizontalAxis),
						glm::dot(bezier.P2 - P0, vec2(-horizontalAxis.y, horizontalAxis.x)));

				QuadraticBezier<float_t> localBezier{P0, P1local, P2local};

				//auto bezierYAtSplit = bezier.calc_y_at_x(posAtSplit.x);
				auto bezierYAtSplit = localBezier.calc_y_at_x(splitPosLocal.x);

				//if (std::isnan(bezierYAtSplit) || std::abs(posAtSplit.y - bezierYAtSplit) > targetMaxError) {
				if (std::isnan(bezierYAtSplit) || std::abs(splitPosLocal.y - bezierYAtSplit) > targetMaxError) {
					shouldSubdivide = true;
				}
			}

		}
	}

	if (shouldSubdivide) {
		adaptive_impl(curve, min, split, targetMaxError, fnAddCurve, depth - 1u);
		adaptive_impl(curve, split, max, targetMaxError, fnAddCurve, depth - 1u);
	}
	else {
		bool degenerate = bezier.P0 == bezier.P2;

		if (!degenerate) {
			fnAddCurve(std::move(bezier));
		}
	}
}
