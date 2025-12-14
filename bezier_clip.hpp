#pragma once

#include "bezier.hpp"
#include "pair.hpp"

#include <algorithm>
#include <memory>

template <typename float_t>
struct Iteration {
	// The bezier curve to be fat line bounded
	const QuadraticBezier<float_t>* F;
	// The bezier curve to be geometric interval bounded
	const QuadraticBezier<float_t>* G;
	Pair<float_t, float_t> fRange;
	Pair<float_t, float_t> gRange;
	std::unique_ptr<Iteration<float_t>> last;

	Iteration(const QuadraticBezier<float_t>* inF, const QuadraticBezier<float_t>* inG,
			Pair<float_t, float_t> inFRange, Pair<float_t, float_t> inGRange, const Iteration* pLast = nullptr)
		: F(inF)
		, G(inG)
		, fRange(inFRange)
		, gRange(inGRange)
		, last(pLast ? std::make_unique<Iteration>(*pLast) : nullptr) {}

	Iteration() = default;
	Iteration(Iteration&&) noexcept = default;
	Iteration& operator=(Iteration&&) noexcept = default;

	Iteration(const Iteration& other) noexcept {
		*this = other;
	}

	Iteration& operator=(const Iteration& other) noexcept {
		F = other.F;
		G = other.G;
		fRange = other.fRange;
		gRange = other.gRange;
		last = other.last ? std::make_unique<Iteration<float_t>>(*other.last) : nullptr;
		return *this;
	}
};

template <typename float_t>
static float_t distance_to_line(const glm::vec<2, float_t>& L0, const glm::vec<2, float_t>& L1,
		const glm::vec<2, float_t>& p) {
	auto cross = [](const auto& a, const auto& b) {
		return a.x * b.y - a.y * b.x;
	};

	auto p10 = L1 - L0;
	//return std::abs(cross(p, p10) + cross(L1, L0)) / glm::length(p10);
	return (cross(p, p10) + cross(L1, L0)) / glm::length(p10);
}

template <typename float_t>
static std::array<float_t, 2> intersect_bez_line(const QuadraticBezier<float_t>& bez,
		const glm::vec<2, float_t>& L0, const glm::vec<2, float_t>& L1, float_t lineConstant,
		bool filterOutOfRangeValues) {
	auto D = glm::normalize(L1 - L0);
	glm::mat<2, 2, float_t> rotation{ 
		{D.x, -D.y}, 
		{D.y, D.x} 
	};

	QuadraticBezier<float_t> rotatedCurve{
		rotation * (bez.P0 - L0),
		rotation * (bez.P1 - L0),
		rotation * (bez.P2 - L0)
	};

	float_t points[] = {rotatedCurve.P0[1], rotatedCurve.P1[1], rotatedCurve.P2[1]};

	for (auto& pt : points) {
		pt -= lineConstant;
	}

	auto A = points[0] - float_t(2.0) * points[1] + points[2];
	auto B = float_t(2.0) * (points[1] - points[0]);
	auto C = points[0];

	auto roots = EqQuadratic<float_t>{A, B, C}.compute_roots();

	std::array<float_t, 2> result{std::numeric_limits<float_t>::quiet_NaN(),
			std::numeric_limits<float_t>::quiet_NaN()};
	int resultCount = 0;

	if (!filterOutOfRangeValues || (roots.x >= float_t(0.0) && roots.x <= float_t(1.0))) {
		result[resultCount++] = roots.x;
	}

	if (!filterOutOfRangeValues || (roots.y >= float_t(0.0) && roots.y <= float_t(1.0))) {
		result[resultCount++] = roots.y;
	}

	return result;
}

template <typename float_t>
static int clip_curve_by_fat_line_par(const QuadraticBezier<float_t>& bezier, const glm::vec<2, float_t>& L0,
		const glm::vec<2, float_t>& L1, float_t lineOffset, const Pair<float_t, float_t>& gRange,
		std::array<Pair<float_t, float_t>, 2>& results) {
	using std::isnan;

	auto resLow = intersect_bez_line(bezier, L0, L1, float_t(0.0), false);
	auto resHigh = intersect_bez_line(bezier, L0, L1, -lineOffset, false);

	int validRootsLo = static_cast<int>(!isnan(resLow[0])) + static_cast<int>(!isnan(resLow[1]));
	int validRootsHi = static_cast<int>(!isnan(resHigh[0])) + static_cast<int>(!isnan(resHigh[1]));

	auto inRange = [&](float_t result) {
		return result >= gRange.first && result <= gRange.second;
	};

	int rootsInRangeLo = static_cast<int>(inRange(resLow[0])) + static_cast<int>(inRange(resLow[1]));
	int rootsInRangeHi = static_cast<int>(inRange(resHigh[0])) + static_cast<int>(inRange(resHigh[1]));

	if (!validRootsLo && !validRootsHi) {
		// If neither the high or low bound of the fat line intersect the other curve,
		// then they cannot intersect
		return 0;
	}

	// Both lines have 2 intersections in range, meaning there is a clear split
	if (rootsInRangeLo == 2 && rootsInRangeHi == 2) {
		auto tMin0 = resLow[0];
		auto tMax0 = resHigh[0];

		auto tMin1 = resLow[1];
		auto tMax1 = resHigh[1];

		// May have overlaps so just use the union of the two curves
		if (tMin0 >= tMin1 || tMax0 >= tMax1) {
			results[0] = {std::min(tMin0, tMin1), std::max(tMax0, tMax1)};
			return 1;
		}

		int resultCount = 0;

		if (tMin0 != tMax0) {
			results[resultCount++] = {tMin0, tMax0};
		}

		if (tMin1 != tMax1) {
			results[resultCount++] = {tMin1, tMax1};
		}

		return resultCount;
	}
	// Both lines have at least 1 intersection in range, so find  the min and max among them
	else if (rootsInRangeLo > 0 && rootsInRangeHi > 0) {
		auto minLo = !inRange(resLow[0]) || (inRange(resLow[1]) && resLow[0] > resLow[1])
				? resLow[1] : resLow[0];
		auto maxLo = !inRange(resLow[0]) || (inRange(resLow[1]) && resLow[0] < resLow[1])
				? resLow[1] : resLow[0];
		auto minHi = !inRange(resHigh[0]) || (inRange(resHigh[1]) && resHigh[0] > resHigh[1])
				? resHigh[1] : resHigh[0];
		auto maxHi = !inRange(resHigh[0]) || (inRange(resHigh[1]) && resHigh[0] < resHigh[1])
				? resHigh[1] : resHigh[0];

		results[0] = {std::min(minLo, minHi), std::max(maxLo, maxHi)};
		return 1;
	}
	else if (rootsInRangeLo > 0 && validRootsHi > 0) {
		auto rootLo = inRange(resLow[0]) ? resLow[0] : resLow[1];
		auto minHi = isnan(resHigh[0])
				|| abs(rootLo - resHigh[0]) > abs(rootLo - resHigh[1]) ? resHigh[1] : resHigh[0];
		minHi = glm::clamp(minHi, gRange.first, gRange.second);

		results[0] = {std::min(rootLo, minHi), std::max(rootLo, minHi)};
		return 1;
	}
	else if (rootsInRangeHi > 0 && validRootsLo > 0) {
		auto rootHi = inRange(resHigh[0]) ? resHigh[0] : resHigh[1];
		auto minLo = isnan(resLow[0])
				|| abs(rootHi - resLow[0]) > abs(rootHi - resLow[1]) ? resLow[1] : resLow[0];
		minLo = glm::clamp(minLo, gRange.first, gRange.second);

		results[0] = {std::min(rootHi, minLo), std::max(rootHi, minLo)};
		return 1;
	}

	results[0] = gRange;
	return 1;
}

template <typename float_t>
static int clip_curve_by_fat_line_perp(const QuadraticBezier<float_t>& bezier, const glm::vec<2, float_t>& L0,
		const glm::vec<2, float_t>& L1, float_t lineOffset, const Pair<float_t, float_t>& gRange,
		std::array<Pair<float_t, float_t>, 2>& results) {
	using std::abs;
	using std::isnan;

	auto resLow = intersect_bez_line(bezier, L0, L1, float_t(0.0), false);
	auto resHigh = intersect_bez_line(bezier, L0, L1, -lineOffset, false);

	int validRootsLo = static_cast<int>(!isnan(resLow[0])) + static_cast<int>(!isnan(resLow[1]));
	int validRootsHi = static_cast<int>(!isnan(resHigh[0])) + static_cast<int>(!isnan(resHigh[1]));

	auto inRange = [&](float_t result) {
		return result >= gRange.first && result <= gRange.second;
	};

	int rootsInRangeLo = static_cast<int>(inRange(resLow[0])) + static_cast<int>(inRange(resLow[1]));
	int rootsInRangeHi = static_cast<int>(inRange(resHigh[0])) + static_cast<int>(inRange(resHigh[1]));

	if (!rootsInRangeLo && !rootsInRangeHi) {
		// If neither the high or low bound of the fat line intersect the other curve,
		// then they cannot intersect
		return 0;
	}

	// Both lines have at least 1 intersection in range, so find  the min and max among them
	if (rootsInRangeLo > 0 && rootsInRangeHi > 0) {
		auto minLo = !inRange(resLow[0]) || (inRange(resLow[1]) && resLow[0] > resLow[1])
				? resLow[1] : resLow[0];
		auto maxLo = !inRange(resLow[0]) || (inRange(resLow[1]) && resLow[0] < resLow[1])
				? resLow[1] : resLow[0];
		auto minHi = !inRange(resHigh[0]) || (inRange(resHigh[1]) && resHigh[0] > resHigh[1])
				? resHigh[1] : resHigh[0];
		auto maxHi = !inRange(resHigh[0]) || (inRange(resHigh[1]) && resHigh[0] < resHigh[1])
				? resHigh[1] : resHigh[0];

		results[0] = {std::min(minLo, minHi), std::max(maxLo, maxHi)};
		return 1;
	}
	// 1 root in range, but some roots out of range, so the range is whichever out of range root is closer
	else if (rootsInRangeLo == 1 && validRootsHi > 0) {
		auto rootLo = inRange(resLow[0]) ? resLow[0] : resLow[1];
		auto minHi = isnan(resHigh[0])
				|| abs(rootLo - resHigh[0]) > abs(rootLo - resHigh[1]) ? resHigh[1] : resHigh[0];
		minHi = glm::clamp(minHi, gRange.first, gRange.second);

		results[0] = {std::min(rootLo, minHi), std::max(rootLo, minHi)};
		return 1;
	}
	// 1 root in range, but some roots out of range, so the range is whichever out of range root is closer
	else if (rootsInRangeHi == 1 && validRootsLo > 0) {
		auto rootHi = inRange(resHigh[0]) ? resHigh[0] : resHigh[1];
		auto minLo = isnan(resLow[0])
				|| abs(rootHi - resLow[0]) > abs(rootHi - resLow[1]) ? resLow[1] : resLow[0];
		minLo = glm::clamp(minLo, gRange.first, gRange.second);

		results[0] = {std::min(rootHi, minLo), std::max(rootHi, minLo)};
		return 1;
	}

	return 0;
}

template <typename float_t>
static int check_intersection_in_ranges(const Iteration<float_t>& iter,
		std::array<Iteration<float_t>, 2>& newIters) {
	using std::abs;

	const float_t maxClipTSpan = float_t(0.7);

	auto subF = iter.F->split_from_min_to_max(iter.fRange.first, iter.fRange.second);
	auto lineOffset = distance_to_line(subF.P0, subF.P2, subF.P1);
	std::array<Pair<float_t, float_t>, 2> intersections;
	int intersectionCount = clip_curve_by_fat_line_par(*iter.G, subF.P0, subF.P2, lineOffset, iter.gRange,
			intersections);

	if (intersectionCount == 0) {
		return 0;
	}
	else if (intersectionCount == 2) {
		newIters[0] = {iter.G, iter.F, intersections[0], iter.fRange, iter.last.get()};
		newIters[1] = {iter.G, iter.F, intersections[1], iter.fRange, iter.last.get()};
		return 2;
	}

	auto [tMin, tMax] = intersections[0];
	auto tScale = float_t(1.0) / (iter.gRange.second - iter.gRange.first);

	// If the range is too large and a solution hasn't been found yet, try a perpendicular line.
	// This is important for efficiency especially in cases where Bezier curves meet (or almost meet)
	// with nearly the same tangent and curvature.
	if (!iter.last && (tMax - tMin) * tScale > maxClipTSpan) {
		auto perp = subF.P0 + glm::vec<2, float_t>(subF.P0.y - subF.P2.y, subF.P2.x - subF.P0.x);
		auto offs2 = distance_to_line(subF.P0, perp, subF.P2);
		lineOffset = distance_to_line(subF.P0, perp, subF.P1);

		if (abs(offs2) > abs(lineOffset)) {
			lineOffset = offs2;
		}

		intersectionCount = clip_curve_by_fat_line_perp(*iter.G, subF.P0, perp, lineOffset, iter.gRange,
				intersections);

		if (intersectionCount == 1) {
			tMin = std::max(intersections[0].first, tMin);
			tMax = std::min(intersections[0].second, tMax);
		}
		else if (intersectionCount == 2) {
			newIters[0] = {iter.G, iter.F, intersections[0], iter.fRange, iter.last.get()};
			newIters[1] = {iter.G, iter.F, intersections[1], iter.fRange, iter.last.get()};
			return 2;
		}
	}

	// The paper clals for a heuristic that if less than 30% will be clipped, you should just split the
	// longest curve and find intersections in the two halves separately.
	// Note that in this case, F and G are not swapped.
	if (!iter.last && (tMax - tMin) * tScale > maxClipTSpan) {
		if (tMax - tMin >= iter.fRange.second - iter.fRange.first) {
			auto tMid = (tMax + tMin) * float_t(0.5);

			// NOTE: To insert into the stack correctly, the [mid, max] interval is placed first.
			newIters[0] = {iter.F, iter.G, iter.fRange, {tMid, tMax}};
			newIters[1] = {iter.F, iter.G, iter.fRange, {tMin, tMid}};
		}
		else {
			auto tMid = (iter.fRange.second + iter.fRange.first) * float_t(0.5);

			newIters[0] = {iter.F, iter.G, {tMid, iter.fRange.second}, iter.gRange};
			newIters[1] = {iter.F, iter.G, {iter.fRange.first, tMid}, iter.gRange};
		}
		
		return 2;
	}

	newIters[0] = {iter.G, iter.F, {tMin, tMax}, iter.fRange, iter.last.get()};
	return 1;
}

template <typename float_t>
static void combine_xs(std::vector<Pair<Pair<float_t, float_t>, Pair<float_t, float_t>>>& xs) {
	bool testAgain = true;

	while (testAgain) {
		testAgain = false;

		for (size_t i = 1; i < xs.size(); ++i) {
			auto& x1b1 = xs[i - 1].first;
			auto& x2b1 = xs[i].first;

			// If the prior tMax value is higher than the next t value's tMin then they overlap
			if (x1b1.second >= x2b1.first) {
				auto& x1b2 = xs[i - 1].second;
				auto& x2b2 = xs[i].second;
				auto [x1min, x1max] = x1b2;
				auto [x2min, x2max] = x2b2;

				bool overlap = (x1min <= x2max && x1max >= x2min) || (x2min <= x1max && x2max >= x1min);

				// Combine ranges and test again
				if (overlap) {
					testAgain = true;

					auto tMinBez2 = std::min<float_t>({x1min, x1max, x2min, x2max});
					auto tMaxBez2 = std::max<float_t>({x1min, x1max, x2min, x2max});

					auto [x1min1, x1max1] = x1b1;
					auto [x2min1, x2max1] = x2b1;

					auto tMinBez1 = std::min<float_t>({x1min1, x1max1, x2min1, x2max1});
					auto tMaxBez1 = std::max<float_t>({x1min1, x1max1, x2min1, x2max1});

					xs[i - 1] = {{tMinBez1, tMaxBez1}, {tMinBez2, tMaxBez2}};

					for (size_t j = i + 1; j < xs.size(); ++j) {
						xs[j - 1] = xs[j];
					}

					xs.pop_back();

					break;
				}
			}
		}
	}
}

template <typename float_t>
struct BezBezDebugEntry {
	std::array<Iteration<float_t>, 2> iters;
	int count;
};

template <typename float_t>
struct BezBezDebug {
	std::vector<Iteration<float_t>> steps;
	std::vector<Pair<Pair<float_t, float_t>, Pair<float_t, float_t>>> xs;
};

template <typename float_t>
static int bez_bez_intersection_clipping(const QuadraticBezier<float_t>& bezierA,
		const QuadraticBezier<float_t>& bezierB, std::array<Pair<float_t, float_t>, 2>& results,
		BezBezDebug<float_t>& debug) {
	using std::abs;
	using std::pow;

	const float_t tMinimumAccuracy = pow(float_t(2.0), -33);
	const float_t finalTMinimumAccuracy = pow(float_t(2.0), -43);

	int iterations = 0;
	int maxIterations = 60;

	std::vector<Iteration<float_t>> stack;
	stack.emplace_back(&bezierA, &bezierB, Pair<float_t, float_t>{0, 1}, Pair<float_t, float_t>{0, 1});

	std::vector<Pair<Pair<float_t, float_t>, Pair<float_t, float_t>>> xs;

	while (!stack.empty() && iterations < maxIterations) {
		++iterations;

		auto iter = std::move(stack.back());
		stack.pop_back();

		std::array<Iteration<float_t>, 2> newIters;
		int resultCount = check_intersection_in_ranges(iter, newIters);

		debug.steps.emplace_back(iter);

		if (resultCount == 1) {
			auto& newIter = newIters[0];
			auto& fRange = newIter.fRange;

			auto ddelta = std::abs(fRange.second - fRange.first);

			// If the previous iteration was precise enough
			if (newIter.last) {
				auto& lfRange = newIter.last->fRange;

				// This case can occur when the geometric interval clips a piece of the other bezier very
				// far away but is by coincidence of length < tMinimumAccuracy.
				if (ddelta > tMinimumAccuracy) {
					// FIXME: Use implicit solver here
				}

				xs.emplace_back(iter.F == &bezierB ? Pair{fRange, lfRange} : Pair{lfRange, fRange});
			}
			// Else if this iteration is precise enough
			else {
				if (ddelta < tMinimumAccuracy) {
					if (ddelta < finalTMinimumAccuracy) {
						// Destructively change the fRange as a heuristic so it's not too narrow for the final
						// clip. This might only be a proble mif fRange == 0
						fRange.first = std::max(float_t(0.0), fRange.first - finalTMinimumAccuracy);
						fRange.second = std::min(float_t(1.0), fRange.second + finalTMinimumAccuracy);
					}

					newIter.last = std::make_unique<Iteration<float_t>>(newIter.F, newIter.G, newIter.fRange,
							newIter.gRange);
				}

				// Push the (possibly) final iteration
				stack.emplace_back(std::move(newIter));
			}
		}
		else if (resultCount == 2) {
			stack.emplace_back(std::move(newIters[0]));
			stack.emplace_back(std::move(newIters[1]));
		}
	}

	std::sort(xs.begin(), xs.end(), [](const auto& a, const auto& b) {
		return a.first.first < b.first.first;
	});

	combine_xs(xs);

	assert(xs.size() <= 2);

	for (size_t i = 0; i < xs.size(); ++i) {
		results[i] = {(xs[i].first.first + xs[i].first.second) * float_t(0.5),
				(xs[i].second.first + xs[i].second.second) * float_t(0.5)};
	}

	return static_cast<int>(xs.size());
}
