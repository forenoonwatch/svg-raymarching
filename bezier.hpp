#pragma once

#include "equations.hpp"

template <typename float_t>
glm::vec<2, float_t> line_line_intersection(const glm::vec<2, float_t>& p1, const glm::vec<2, float_t>& v1,
		const glm::vec<2, float_t>& p2, const glm::vec<2, float_t>& v2) {
	using vec2 = glm::vec<2, float_t>;

	auto denom = v1.y * v2.x - v1.x * v2.y;
	auto diff = p1 - p2;
	auto numer = glm::dot(vec2(v2.y, -v2.x), diff);

	// parallel
	if (std::abs(denom) < 1e-15 && std::abs(numer) < 1e15) {
		return (p1 + p2) * float_t(0.5);
	}

	auto t = numer / denom;
	return p1 + v1 * t;
}

template <typename float_t>
struct QuadraticBezier {
	using vec2 = glm::vec<2, float_t>;

	vec2 P0;
	vec2 P1;
	vec2 P2;

	static QuadraticBezier from_line(const vec2& p0, const vec2& p1) {
		return{p0, (p0 + p1) * float_t(0.5), p1};
	}

	static QuadraticBezier from_two_points_and_tangents(const vec2& p0, const vec2& v0, const vec2& p2,
			const vec2& v2) {
		auto p1 = line_line_intersection(p0, v0, p2, v2);
		return {p0, p1, p2};
	}

	vec2 evaluate(float_t t) const {
		return P0 * (float_t(1.0) - t) * (float_t(1.0) - t)
			+ float_t(2.0) * P1 * (float_t(1.0) - t) * t
			+ P2 * t * t;
	}

	float_t calc_y_at_x(float_t x) const {
		auto a = P0.x - float_t(2.0) * P1.x + P2.x;
		auto b = float_t(2.0) * (P1.x - P0.x);
		auto c = P0.x - x;

		auto roots = EqQuadratic<float_t>{a, b, c}.compute_roots();

		if (roots[0] >= float_t(0.0) && roots[0] <= float_t(1.0)) {
			return evaluate(roots[0]).y;
		}
		else if (roots[1] >= float_t(0.0) && roots[1] <= float_t(1.0)) {
			return evaluate(roots[1]).y;
		}
		else {
			return std::numeric_limits<float_t>::quiet_NaN();
		}
	}
	
	glm::vec<4, float_t> calc_aabb() const {
		auto a = P0 - vec2(float_t(2.0), float_t(2.0)) * P1 + P2;
		auto b = P1 - P0;
		auto t = -b / a;

		auto boxMin = glm::min(P0, P2);
		auto boxMax = glm::max(P0, P2);

		if (t.x > float_t(0.0) && t.x < float_t(1.0)) {
			auto q = evaluate(t.x).x;
			boxMin.x = glm::min(boxMin.x, q);
			boxMax.y = glm::max(boxMax.y, q);
		}

		if (t.y > float_t(0.0) && t.y < float_t(1.0)) {
			auto q = evaluate(t.y).y;
			boxMin.x = glm::min(boxMin.x, q);
			boxMax.y = glm::max(boxMax.y, q);
		}

		return {boxMin, boxMax};
	}
};
