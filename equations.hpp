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

		if (D == float_t(0.0)) {
			// one triple solution
			if (q == float_t(0.0)) {
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
		else if (D < float_t(0.0)) {
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

template <typename float_t>
struct EqQuartic {
	// form: ax^4 + bx^3 + cx^2 + dx + e
	float_t a;
	float_t b;
	float_t c;
	float_t d;
	float_t e;

	// Originally from: https://github.com/erich666/GraphicsGems/blob/master/gems/Roots3And4.c
	glm::vec<4, float_t> compute_roots() const {
		static constexpr const float_t NaN = std::bit_cast<float_t>(std::numeric_limits<float_t>::quiet_NaN());

		float_t  coeffs[4];
		glm::vec<4, float_t> s{NaN, NaN, NaN, NaN};
		uint32_t rootCount = 0;

		/* normal form: x^4 + Ax^3 + Bx^2 + Cx + D = 0 */
		auto A = b / a;
		auto B = c / a;
		auto C = d / a;
		auto D = e / a;

		/*  substitute x = y - A/4 to eliminate cubic term: x^4 + px^2 + qx + r = 0 */
		auto sq_A = A * A;
		auto p = -3.0 / 8 * sq_A + B;
		auto q = 1.0 / 8 * sq_A * A - 1.0 / 2 * A * B + C;
		auto r = -3.0 / 256 * sq_A * sq_A + 1.0 / 16 * sq_A * B - 1.0 / 4 * A * C + D;

		if (r == 0.0) {
			/* no absolute term: y(y^3 + py + q) = 0 */
			auto cubic = EqCubic<float_t>{1, 0, p, q}.compute_roots();
			s[rootCount++] = cubic[0];
			s[rootCount++] = cubic[1];
			s[rootCount++] = cubic[2];
		}
		else {
			/* solve the resolvent cubic ... */
			auto cubic = EqCubic<float_t>{1, -1.0 / 2 * p, -r,
					1.0 / 2 * r * p - 1.0 / 8 * q * q}.compute_roots();
			float_t z;

			/* ... and take the one real solution ... */
			for (uint32_t i = 0; i < 3; i ++) {
				if (!std::isnan(cubic[i])) {
					z = cubic[i];
					break;
				}
			}

			/* ... to build two quadratic equations */
			auto u = z * z - r;
			auto v = 2 * z - p;

			if (u == 0.0) {
				u = 0;
			}
			else if (u > 0) {
				u = std::sqrt(u);
			}
			else {
				return s; // (empty)
			}

			if (v == 0.0) {
				v = 0;
			}
			else if (v > 0) {
				v = std::sqrt(v);
			}
			else {
				return s; // (empty)
			}

			auto quadric1 = EqQuadratic<float_t>{1, q < 0 ? -v : v, z - u}.compute_roots();
			auto quadric2 = EqQuadratic<float_t>{1, q < 0 ? v : -v, z + u}.compute_roots();

			for (uint32_t i = 0; i < 2; ++i) {
				if (!std::isinf(quadric1[i]) && !std::isnan(quadric1[i])) {
					s[rootCount++] = quadric1[i];
				}
			}

			for (uint32_t i = 0; i < 2; ++i) {
				if (!std::isinf(quadric2[i]) && !std::isnan(quadric2[i])) {
					s[rootCount++] = quadric2[i];
				}
			}
		}

		/* resubstitute */
		auto sub = 1.0 / 4 * A;

		for (uint32_t i = 0; i < rootCount; ++i) {
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
struct QuadraticCurve {
	using vec2 = glm::vec<2, float_t>;

	vec2 A;
	vec2 B;
	vec2 C;

	static QuadraticCurve from_bezier(const vec2& P0, const vec2& P1, const vec2& P2) {
		return {
			.A = P0 - float_t(2.0) * P1 + P2,
			.B = float_t(2.0) * (P1 - P0),
			.C = P0,
		};
	}

	static constexpr const uint32_t MAX_CANDIDATES = 3;
	using Candidates = glm::vec<MAX_CANDIDATES, float_t>;

	Candidates get_closest_candidates(const vec2& pos) const {
		// p(t)    = (1-t)^2*A + 2(1-t)t*B + t^2*C
        // p'(t)   = 2*t*(A-2*B+C) + 2*(B-A)
        // p'(0)   = 2(B-A)
        // p'(1)   = 2(C-B)
        // p'(1/2) = 2(C-A)
            
        // exponent so large it would wipe the mantissa on any relative operation
        // should be exp2<float_t>(numeric_limits<float_t>::digits) after tgmath has an exp2
        static constexpr auto PARAMETER_THRESHOLD = std::exp2(static_cast<float>(
				std::numeric_limits<float_t>::digits));

        Candidates candidates;
            
        auto Bdiv2 = B * float_t(0.5);
        auto CsubPos = C - pos;
        auto Alen2 = glm::dot(A, A);
            
        // if A = 0, solve linear instead
        if (Alen2 < std::exp2(-23.0f) * glm::dot(B, B)) {
            candidates[0] = std::abs(glm::dot(float_t(2.0) * Bdiv2, CsubPos)) / glm::dot(float_t(2.0) * Bdiv2,
					float_t(2.0) * Bdiv2);
            candidates[1] = PARAMETER_THRESHOLD;
            candidates[2] = PARAMETER_THRESHOLD;
        }
        else {
            // Reducing Quartic to Cubic Solution
            auto kk = float_t(1.0) / Alen2;
            auto kx = kk * glm::dot(Bdiv2, A);
            auto ky = kk * (float_t(2.0) * glm::dot(Bdiv2, Bdiv2) + glm::dot(CsubPos, A)) / float_t(3.0);
            auto kz = kk * glm::dot(CsubPos, Bdiv2);

            // Cardano's Solution to resolvent cubic of the form: y^3 + 3py + q = 0
            // where it was initially of the form x^3 + ax^2 + bx + c = 0 and x was replaced by y - a/3
            // so a/3 needs to be subtracted from the solution to the first form to get the actual solution
            auto p = ky - kx * kx;
            auto p3 = p * p * p;
            auto q = kx * (float_t(2.0) * kx * kx - float_t(3.0) * ky) + kz;
            auto h = q * q + float_t(4.0) * p3;
            
            if (h < float_t(0.0)) {
                // 3 roots
                float_t z = std::sqrt(-p);
                float_t v = std::acos( q / (p * z * float_t(2.0)) ) / float_t(3.0);
                float_t m = std::cos(v);
                float_t n = std::sin(v) * float_t(1.732050808);
                    
                candidates[0] = (m + m) * z - kx;
                candidates[1] = (-n - m) * z - kx;
                candidates[2] = (n - m) * z - kx;
            }
            else {
                // 1 root
                h = std::sqrt(h);
                auto x = (vec2(h, -h) - q) / float_t(2.0);

                // Solving Catastrophic Cancellation when h and q are close (when p is near 0)
                if (std::abs(std::abs(h / q) - float_t(1.0)) < float_t(0.0001)) {
                    // Approximation of x where h and q are close with no carastrophic cancellation
                    // Derivation (for curious minds) -> h=√(q²+4p³)=q·√(1+4p³/q²)=q·√(1+w)
                        // Now let's do linear taylor expansion of √(1+x) to approximate √(1+w)
                        // Taylor expansion at 0 -> f(x)=f(0)+f'(0)*(x) = 1+½x 
                        // So √(1+w) can be approximated by 1+½w
                        // Simplifying later and the fact that w=4p³/q will result in the following.
                    x = vec2(p3 / q, -q - p3 / q);
                }

                auto uv = glm::sign(x) * std::pow(std::abs(x), vec2(float_t(1.0) / float_t(3.0),
						float_t(1.0)/ float_t(3.0)));
                candidates[0u] = uv.x + uv.y - kx;
                candidates[1u] = PARAMETER_THRESHOLD;
                candidates[2u] = PARAMETER_THRESHOLD;
            }
        }
            
        return candidates;
	}
};

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
