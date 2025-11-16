#pragma once

#include <cmath>

#include <bit>
#include <numbers>
#include <limits>

#include <glm/vec2.hpp>
#include <glm/geometric.hpp>

#include "gauss_legendre.hpp"

template <typename float_t>
struct EqQuadratic {
	float_t a;
	float_t b;
	float_t c;

	float_t evaluate(float_t t) const {
		return t * (a * t + b) + c;
	}

	glm::vec<2, float_t> compute_roots() const {
		glm::vec<2, float_t> res;

		auto det = b * b - static_cast<float_t>(4.0) * a * c;
		auto detSqrt = std::sqrt(det);
		auto rcp = static_cast<float_t>(0.5) / a;
		auto bOver2A = b * rcp;

		if (b >= 0) {
			res[0] = -detSqrt * rcp - bOver2A;
			res[1] = 2 * c / (-b - detSqrt);
		}
		else {
			res[0] = 2 * c / (-b + detSqrt);
			res[1] = +detSqrt * rcp - bOver2A;
		}

		return res;
	}
};

template <typename float_t>
struct EqCubic {
	float_t a;
	float_t b;
	float_t c;
	float_t d;

	glm::vec<3, float_t> compute_roots() const {
		using vec3 = glm::vec<3, float_t>;
		static constexpr const float_t NaN = std::bit_cast<float_t>(std::numeric_limits<float_t>::quiet_NaN());

		vec3 s(NaN, NaN, NaN);

		// normal form: x^3 + Ax^2 + Bx + C = 0
		auto A = b / a;
		auto B = c / a;
		auto C = d / a;

		// substitute x = y - A / 3 to eliminate quadratic term
		auto sqA = A * A;
		auto p = float_t(1.0) / 3 * (float_t(-1.0) / 3 * sqA + B);
		auto q = float_t(1.0) / 2 * (float_t(2.0) / 27 * A * sqA - float_t(1.0) / 3 * A * B + C);

		// use Cardano's formula
		auto cbp = p * p * p;
		auto D = q * q + cbp;

		unsigned rootCount = 0;

		if (D == 0) {
			// one triple solution
			if (q == 0) {
				s[rootCount++] = float_t(0.0);
			}
			// one single and one double solution
			else {
				auto u = std::cbrt(-q);
				s[rootCount++] = 2 * u;
				s[rootCount++] = -u;
			}
		}
		// Three real solutions
		else if (D < 0) {
			auto phi = float_t(1.0) / 3 * std::acos(-q / sqrt(-cbp));
			auto t = 2 * std::sqrt(-p);

			s[rootCount++] = t * std::cos(phi);
			s[rootCount++] = -t * std::cos(phi + std::numbers::pi_v<float_t> / 3);
			s[rootCount++] = -t * std::cos(phi - std::numbers::pi_v<float_t> / 3);
		}
		// One real solution
		else {
			auto sqrtD = std::sqrt(D);
			auto u = std::cbrt(sqrtD - q);
			auto v = -std::cbrt(sqrtD + q);

			s[rootCount++] = u + v;
		}

		// resubstitute
		auto sub = float_t(1.0) / 3 * A;

		for (int i = 0; i < rootCount; ++i) {
			s[i] -= sub;
		}

		return s;
	}
};

template <typename float_t, typename Derived>
struct ParametricCurve {
	float_t differential_arc_len(float_t t) const;
	float_t arc_len(float_t t0, float_t t1) const;

	// compute inverse arc len using bisection search
	float_t inverse_arc_len_bisection_search(float_t targetLen, float_t min, float_t max,
			float_t cdfAccuracyThreshold = 1e-4, unsigned iterationThreshold = 16) const;
};

template <typename float_t, typename Derived>
float_t ParametricCurve<float_t, Derived>::differential_arc_len(float_t t) const {
	return glm::length(static_cast<const Derived*>(this)->compute_tangent(t));
}

template <typename float_t, typename Derived>
float_t ParametricCurve<float_t, Derived>::arc_len(float_t t0, float_t t1) const {
	static constexpr const int IntegrationOrder = 10;

	return GaussLegendreIntegration<IntegrationOrder, float_t>::calculate_integral([this](float_t t) {
		return differential_arc_len(t);
	}, t0, t1);
}

template <typename float_t, typename Derived>
float_t ParametricCurve<float_t, Derived>::inverse_arc_len_bisection_search(float_t targetLen, float_t min,
		float_t max, float_t cdfAccuracyThreshold, unsigned iterationThreshold) const {
	//return min + (max - min) * targetLen;
	
	float_t xi = float_t(0.0);
	auto low = min;
	auto high = max;

	for (unsigned i = 0; i < iterationThreshold; ++i) {
		xi = (low + high) / float_t(2.0);
		auto sum = arc_len(min, xi);
		auto integral = sum + arc_len(xi, max);

		// we could've done sum/integral - targetLen but this is more robust as it avoids a division
		auto valueAtParamGuess = sum - targetLen * integral;

		if (std::abs(valueAtParamGuess) < cdfAccuracyThreshold * integral) {
			// We found xi value that gives us a cdf of targetLen within cdfAccuracyThreshold
			return xi;
		}
		else {
			if (valueAtParamGuess > float_t(0.0)) {
				high = xi;
			}
			else {
				low = xi;
			}
		}
	}

	return xi;
}

template <typename float_t>
struct CubicCurve final : public ParametricCurve<float_t, CubicCurve<float_t>> {
	using vec2 = glm::vec<2, float_t>;

	glm::vec<4, float_t> X;
	glm::vec<4, float_t> Y;

	static CubicCurve from_bezier(const vec2& P0, const vec2& P1, const vec2& P2, const vec2& P3) {
		return {
			.X = {
				-P0.x + float_t(3.0) * (P1.x - P2.x) + P3.x,
				float_t(3.0) * P0.x - float_t(6.0) * P1.x + float_t(3.0) * P2.x,
				float_t(-3.0) * P0.x + float_t(3.0) * P1.x,
				P0.x,
			},
			.Y = {
				-P0.y + float_t(3.0) * (P1.y - P2.y) + P3.y,
				float_t(3.0) * P0.y - float_t(6.0) * P1.y + float_t(3.0) * P2.y,
				float_t(-3.0) * P0.y + float_t(3.0) * P1.y,
				P0.y,
			},
		};
	}

	glm::vec<2, float_t> compute_position(float_t t) const;
	// compute unnormalized tangent vector at t
	glm::vec<2, float_t> compute_tangent(float_t t) const;

	float_t compute_inflection_point(float_t errorThreshold) const;
};

template <typename float_t>
glm::vec<2, float_t> CubicCurve<float_t>::compute_position(float_t t) const {
	return glm::vec<2, float_t>{
		((X[0] * t + X[1]) * t + X[2]) * t + X[3],
		((Y[0] * t + Y[1]) * t + Y[2]) * t + Y[3],
	};
}

template <typename float_t>
glm::vec<2, float_t> CubicCurve<float_t>::compute_tangent(float_t t) const {
	return glm::vec<2, float_t>{
		(float_t(3.0) * X[0] * t + float_t(2.0) * X[1]) * t + X[2],
		(float_t(3.0) * Y[0] * t + float_t(2.0) * Y[1]) * t + Y[2],
	};
}

template <typename float_t>
float_t CubicCurve<float_t>::compute_inflection_point(float_t errorThreshold) const {
	// solve for signed curvature root
	// when x'*y''-x''*y' = 0
	// https://www.wolframalpha.com/input?i=cross+product+%283*x0*t%5E2%2B2*x1%2Bx2%2C3*y0*t%5E2%2B2*y1%2By2%29+and+%286*x0*t%2B2*x1%2C6*y0*t%2B2*y1%29
	
	auto a = float_t(6.0) * (X[0] * Y[1] - X[1] * Y[0]);
	auto b = float_t(6.0) * (float_t(2.0) * X[1] * Y[0] - float_t(2.0) * X[0] * Y[1] + X[2] * Y[0]
			- X[0] * Y[2]);
	auto c = float_t(2.0) * (X[2] * Y[1] - X[1] * Y[2]);

	EqQuadratic<float_t> quadratic{a, b, c};
	auto roots = quadratic.compute_roots();

	if (roots[0] <= 1.0 && roots[0] >= 0.0) {
		return roots[0];
	}

	if (roots[1] <= 1.0 && roots[1] >= 0.0) {
		return roots[1];
	}

	return std::numeric_limits<float_t>::quiet_NaN();
}

// Centered at (0, 0), aligned with x axis
template <typename float_t>
struct AxisAlignedEllipse final : public ParametricCurve<float_t, AxisAlignedEllipse<float_t>> {
	float_t a, b;
	float_t start, end;

	constexpr explicit AxisAlignedEllipse(float inA, float inB, float inStart, float inEnd)
			: a(inA), b(inB), start(inStart), end(inEnd) {}

	glm::vec<2, float_t> compute_position(float_t t) const;
	glm::vec<2, float_t> compute_tangent(float_t t) const;

	float_t compute_inflection_point(float_t) const { return std::numeric_limits<float_t>::quiet_NaN(); }
};

template <typename float_t>
glm::vec<2, float_t> AxisAlignedEllipse<float_t>::compute_position(float_t t) const {
	auto theta = start + (end - start) * t;
	return {a * std::cos(theta), b * std::sin(theta)};
}

template <typename float_t>
glm::vec<2, float_t> AxisAlignedEllipse<float_t>::compute_tangent(float_t t) const {
	auto theta = start + (end - start) * t;
	auto dThetaDt = end - start;
	return {-a * dThetaDt * std::sin(theta), b * dThetaDt * std::cos(theta)};
}

template <typename float_t>
struct EllipticalArcInfo {
	using vec2 = glm::vec<2, float_t>;

	vec2 majorAxis;
	vec2 center;
	vec2 angleBounds; // [0, 2Pi)
	float_t eccentricity; // (0, 1]
};
