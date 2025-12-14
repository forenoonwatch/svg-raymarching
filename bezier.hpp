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

	vec2 derivative(float_t t) const {
		auto tangentAtStart = float_t(2.0) * (P1 - P0);
		auto tangentAtEnd = float_t(2.0) * (P2 - P1);
		auto tangent = (float_t(1.0) - t) * tangentAtStart + t * tangentAtEnd;

		return tangent;
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

	QuadraticBezier split_from_start(float_t t) const {
		return {
			.P0 = P0,
			.P1 = (float_t(1.0) - t) * P0 + t * P1,
			.P2 = (float_t(1.0) - t) * ((float_t(1.0) - t) * P0 + t * P1)
				+ t * ((float_t(1.0) - t) * P1 + t * P2),
		};
	}

	QuadraticBezier split_to_end(float_t t) const {
		return {
			.P0 = (float_t(1.0) - t) * ((float_t(1.0) - t) * P0 + t * P1)
					+ t * ((float_t(1.0) - t) * P1  + t * P2),
			.P1 = (float_t(1.0) - t) * P1 + t * P2,
			.P2 = P2,
		};
	}

	QuadraticBezier split_from_min_to_max(float_t minT, float_t maxT) const {
		return split_from_start(maxT).split_to_end(minT / maxT);
	}
};

template <typename float_t, typename internal_t = double>
EqQuartic<float_t> get_bezier_bezier_intersection_equation(const QuadraticBezier<float_t>& lhs,
		const QuadraticBezier<float_t>& rhs) {
	// Algorithm based on Computer Aided Geometric Design: 
    // https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=1000&context=facpub#page99
    // Chapter 17.6 describes the implicitization of a curve, which transforms it into the following format:
    // ax^2 + bxy + cy^2 + dx + ey + f = 0 
    // the coefficients a-f above are k0-k5 below 
    // 
    // We then substitute x and y for the other curve's quadratic formulas.
    // This gives us a quartic equation of the form:
    // at^4 + bt^3 + ct^2 + dt + e = 0
    // the coefficients a-e above are a-e below 
    // 
    // The roots for t then become our intersections points between both curves.
    //
    // A Desmos page including math for this as well as some of the graphs it generates is available here:
    // https://www.desmos.com/calculator/mjwqvnvyb8?lang=pt-BR

    // for convenience
    auto p0x = static_cast<internal_t>(rhs.P0.x);
    auto p1x = static_cast<internal_t>(rhs.P1.x);
    auto p2x = static_cast<internal_t>(rhs.P2.x);
    auto p0y = static_cast<internal_t>(rhs.P0.y);
    auto p1y = static_cast<internal_t>(rhs.P1.y);
    auto p2y = static_cast<internal_t>(rhs.P2.y);

    // Implicitize other curve
    auto k0 = (4 * p0y * p1y) - (4 * p0y * p2y) - (4 * (p1y * p1y)) + (4 * p1y * p2y) - ((p0y * p0y))
			+ (2 * p0y * p2y) - ((p2y * p2y));
    auto k1 = -(4 * p0x * p1y) + (4 * p0x * p2y) - (4 * p1x * p0y) + (8 * p1x * p1y) - (4 * p1x * p2y)
			+ (4 * p2x * p0y) - (4 * p2x * p1y) + (2 * p0x * p0y) - (2 * p0x * p2y) - (2 * p2x * p0y)
			+ (2 * p2x * p2y);
    auto k2 = (4 * p0x * p1x) - (4 * p0x * p2x) - (4 * (p1x * p1x)) + (4 * p1x * p2x) - ((p0x * p0x))
			+ (2 * p0x * p2x) - ((p2x * p2x));
    auto k3 = (4 * p0x * (p1y * p1y)) - (4 * p0x * p1y * p2y) - (4 * p1x * p0y * p1y) + (8 * p1x * p0y * p2y)
			- (4 * p1x * p1y * p2y) - (4 * p2x * p0y * p1y) + (4 * p2x * (p1y * p1y)) - (2 * p0x * p0y * p2y)
			+ (2 * p0x * (p2y * p2y)) + (2 * p2x * (p0y * p0y)) - (2 * p2x * p0y * p2y);
    auto k4 = -(4 * p0x * p1x * p1y) - (4 * p0x * p1x * p2y) + (8 * p0x * p2x * p1y) + (4 * (p1x * p1x) * p0y)
			+ (4 * (p1x * p1x) * p2y) - (4 * p1x * p2x * p0y) - (4 * p1x * p2x * p1y) + (2 * (p0x * p0x) * p2y)
			- (2 * p0x * p2x * p0y) - (2 * p0x * p2x * p2y) + (2 * (p2x * p2x) * p0y);
    auto k5 = (4 * p0x * p1x * p1y * p2y) - (4 * (p1x * p1x) * p0y * p2y) + (4 * p1x * p2x * p0y * p1y)
			- ((p0x * p0x) * (p2y * p2y)) + (2 * p0x * p2x * p0y * p2y) - ((p2x * p2x) * (p0y * p0y))
			- (4 * p0x * p2x * (p1y * p1y));
        
    //auto quadratic = QuadraticCurve<float_t>::from_bezier(lhs.P0, lhs.P1, lhs.P2);
    // for convenience
	//glm::vec<2, internal_t> A{static_cast<internal_t>(quadratic.A.x), static_cast<internal_t>(quadratic.A.y)};
	//glm::vec<2, internal_t> B{static_cast<internal_t>(quadratic.B.x), static_cast<internal_t>(quadratic.B.y)};
	//glm::vec<2, internal_t> C{static_cast<internal_t>(quadratic.C.x), static_cast<internal_t>(quadratic.C.y)};

	auto A = lhs.P0 - glm::vec<2, internal_t>(float_t(2.0), float_t(2.0)) * lhs.P1 + lhs.P2;
	auto B = glm::vec<2, float_t>(float_t(2.0), float_t(2.0)) * (lhs.P1 - lhs.P0);
	const auto& C = lhs.P0; // fuck boost

    // substitute parametric into implicit equation:
        
    // Getting the quartic params
    internal_t a = ((A.x * A.x) * k0) + (A.x * A.y * k1) + (A.y * A.y * k2);
    internal_t b = (A.x * B.x * k0 * internal_t(2.0)) + (A.x * B.y * k1) + (B.x * A.y * k1)
			+ (A.y * B.y * k2 * internal_t(2.0));
    internal_t c = (A.x * C.x * k0 * internal_t(2.0)) + (A.x * C.y * k1) + (A.x * k3) + ((B.x * B.x) * k0)
			+ (B.x * B.y * k1) + (C.x * A.y * k1) + (A.y * C.y * k2 * internal_t(2.0)) + (A.y * k4)
			+ ((B.y * B.y) * k2);
    internal_t d = (B.x * C.x * k0 * internal_t(2.0)) + (B.x * C.y * k1) + (B.x * k3) + (C.x * B.y * k1)
			+ (B.y * C.y * k2 * internal_t(2.0)) + (B.y * k4);
    internal_t e = ((C.x * C.x) * k0) + (C.x * C.y * k1) + (C.x * k3) + ((C.y * C.y) * k2) + (C.y * k4) + (k5);

    return EqQuartic<float_t>{static_cast<float_t>(a), static_cast<float_t>(b), static_cast<float_t>(c),
			static_cast<float_t>(d), static_cast<float_t>(e)};
}
