#include "hatch.hpp"
#include "hatch_impl.hpp"

#include "pair.hpp"
#include "svg.hpp"

#include <cassert>
#include <cstring>
#include <cstdio>

#include <algorithm>
#include <limits>
#include <queue>

#include <glm/mat2x2.hpp>

using namespace Hatch;

static Pair<glm::vec<2, real_t>, glm::vec<2, real_t>> calc_bezier_bounding_box_minor(
		const QuadraticBezier<real_t>& bezier);

// Transforms curves into AABB UV space and turns them into quadratic coefficients
static void transform_curves(const QuadraticBezier<real_t>& bezier, const glm::vec<2, real_t>& aabbMin,
		const glm::vec<2, real_t>& aabbMax, glm::vec2* output);

// FIXME: Does not properly detect (P0, P1, P1) segments
static bool is_line_segment(const QuadraticBezier<real_t>& bezier);

template <typename float_t>
static float_t cross(const glm::vec<2, float_t>& a, const glm::vec<2, float_t>& b) {
	return a.x * b.y - a.y * b.x;
}

void generate_hatch_boxes(const CPUQuadraticShape& shape, std::vector<CurveHatchBox>& hatchBoxes) {
	std::vector<QuadraticBezier<real_t>> beziers;

	int major = static_cast<int>(SELECTED_MAJOR_AXIS);
	int minor = 1 - major;

	auto addMonotonicBezier = [&](const QuadraticBezier<real_t>& bezier) {
		auto outputBezier = bezier;

		if (outputBezier.P0[major] > outputBezier.P2[major]) {
			outputBezier.P2 = bezier.P0;
			outputBezier.P0 = bezier.P2;
			assert(outputBezier.P0[major] <= outputBezier.P2[major]);
		}

		// Fix in case of small precision issues when splitting into major monotonic segments
		if (outputBezier.P1[major] < outputBezier.P0[major]) {
			outputBezier.P1[major] = outputBezier.P0[major];
		}

		beziers.emplace_back(std::move(outputBezier));
	};

	for (auto& path : shape.paths) {
		for (size_t ip = 0; ip < path.points.size() - 1; ip += 2) {
			QuadraticBezier<real_t> unsplitBezier{path.points[ip], path.points[ip + 1], path.points[ip + 2]};

			// FIXME: Better detect that a line segment is unsplit

			std::array<QuadraticBezier<real_t>, 2> monotonicSegments;
			bool alreadyMonotonic = split_into_major_monotonic_segments(unsplitBezier, monotonicSegments);

			if (alreadyMonotonic) {
				addMonotonicBezier(unsplitBezier);
			}
			else {
				addMonotonicBezier(monotonicSegments[0]);
				addMonotonicBezier(monotonicSegments[1]);
			}
		}
	}

	std::vector<Segment> segments;

	for (auto& bezier : beziers) {
		segments.emplace_back(Segment{
			.originalBezier = &bezier,
			.tStart = real_t(0.0),
			.tEnd = real_t(1.0),
		});
	}

	std::sort(segments.begin(), segments.end(), [major](const Segment& a, const Segment& b) {
		return a.originalBezier->P0[major] > b.originalBezier->P0[major];
	});

	std::vector<Segment> starts; // Next segments sorted by start points
	std::vector<real_t> ends; // Next end points

	for (auto& segment : segments) {
		starts.emplace_back(segment);
	}

	std::sort(segments.begin(), segments.end(), [major](const Segment& a, const Segment& b) {
		return a.originalBezier->P2[major] > b.originalBezier->P2[major];
	});

	for (auto& segment : segments) {
		ends.emplace_back(segment.originalBezier->P2[major]);
	}

	auto maxMajor = segments.front().originalBezier->P2[major];

	// Sweep line algorithm
	
	// Next intersection points as major coordinate
	std::priority_queue<real_t, std::vector<real_t>, std::greater<real_t>> intersections;
	std::vector<Segment> activeCandidates; // Set of active candidates for neighbor search in sweep line
	
	SegmentLess candidateComparator{major, minor};

	auto addToCandidateSet = [&](const Segment& entry) {
		if (entry.is_straight_line_constant_major()) {
			return;
		}

		// Look for intersections among active candidates. This is O(n^2) but only for n=candidates.size()
		for (auto& segment : activeCandidates) {
			auto intersectionPoints = entry.intersect(segment);

			for (auto point : intersectionPoints) {
				if (std::isnan(point)) {
					continue;
				}

				intersections.push(segment.originalBezier->evaluate(point)[major]);
			}
		}

		activeCandidates.emplace_back(entry);
	};

	auto lastMajor = starts.back().originalBezier->evaluate(starts.back().tStart)[major];

	while (lastMajor != maxMajor) {
		if (ends.empty()) {
			assert(false && "Hatch creation failure: ends stack is empty in the main loop");
			break;
		}

		real_t newMajor;
		bool addStartSegmentToCandidates = false;

		auto maxMajorEnds = ends.back();
		auto nextStartEvent = starts.empty() ? Segment{} : starts.back();
		auto minMajorStart = nextStartEvent.originalBezier
				? nextStartEvent.originalBezier->evaluate(nextStartEvent.tStart)[major] : real_t(0.0);

		// We check which event, within start, end, and intersection events have the smallest major coordinate
		// at this point
		auto intersectionVisit = [&] {
			auto newMajor = intersections.top();
			intersections.pop();
			return newMajor;
		};

		// Next start event is before next end event
		if (nextStartEvent.originalBezier && minMajorStart < maxMajorEnds) {
			// Next start event is before next intersection event
			if (intersections.empty() || minMajorStart < intersections.top()) {
				starts.pop_back();
				newMajor = minMajorStart;
				addStartSegmentToCandidates = true;
			}
			else {
				newMajor = intersectionVisit();
			}
		}
		// Next intersection event is before next end event
		else if (!intersections.empty() && intersections.top() < maxMajorEnds) {
			newMajor = intersectionVisit();
		}
		// (end event)
		else {
			newMajor = maxMajorEnds;
			ends.pop_back();
		}

		// Spawn quads for the previous iterations if we advanced
		if (newMajor > lastMajor) {
			auto candidatesSize = std::distance(activeCandidates.begin(), activeCandidates.end());

			// True in nearly every case for loops, can fail at times because we skip adding bezier (lines)
			// almost constant in the major axis direction. Assuming this means it may be false for open paths
			if (candidatesSize % 2u == 0u) {
				for (size_t i = 0; i < (candidatesSize / 2) * 2;) {
					auto& left = activeCandidates[i++];
					auto& right = activeCandidates[i++];

					CurveHatchBox curveBox;

					// Due to precision, if the curve is right at the end, intersect_ortho may return nan
					auto curveMinEnd = intersect_ortho(*left.originalBezier, newMajor, major);
					auto curveMaxEnd = intersect_ortho(*right.originalBezier, newMajor, major);

					auto splitCurveMin = left.originalBezier->split_from_min_to_max(left.tStart,
							std::isnan(curveMinEnd) ? real_t(1.0) : curveMinEnd);
					auto splitCurveMax = right.originalBezier->split_from_min_to_max(right.tStart,
							std::isnan(curveMaxEnd) ? real_t(1.0) : curveMaxEnd);

					assert(splitCurveMin.evaluate(0.0)[major] <= splitCurveMin.evaluate(1.0)[major]);
					assert(splitCurveMax.evaluate(0.0)[major] <= splitCurveMax.evaluate(1.0)[major]);

					auto curveMinAabb = calc_bezier_bounding_box_minor(splitCurveMin);
					auto curveMaxAabb = calc_bezier_bounding_box_minor(splitCurveMax);
					curveBox.aabbMin = {std::min(curveMinAabb.first.x, curveMaxAabb.first.x), lastMajor};
					curveBox.aabbMax = {std::max(curveMinAabb.second.x, curveMaxAabb.second.x), newMajor};

					transform_curves(splitCurveMin, curveBox.aabbMin, curveBox.aabbMax, curveBox.curveMin);
					transform_curves(splitCurveMax, curveBox.aabbMin, curveBox.aabbMax, curveBox.curveMax);

					hatchBoxes.emplace_back(curveBox);
				}
			}

			// Advance and trim all of the beziers in the candidate set
			auto oit = activeCandidates.begin();
			for (auto iit = activeCandidates.begin(); iit != activeCandidates.end(); ++iit) {
				auto evalAtMajor = iit->originalBezier->evaluate(iit->tEnd)[major];

				// If we scrolled past the end of the segment, remove it. Basically, we memcpy everything
				// after something is different and we skip on the memcpy for any items that are also different
				// This is supposedly a pattern with input/output operators
				if (newMajor < evalAtMajor) {
					auto newTStart = intersect_ortho(*iit->originalBezier, newMajor, major);

					// Little optimization: don't memcpy anything before something was removed
					if (oit != iit) {
						*oit = *iit;
					}

					oit->tStart = newTStart;
					++oit;
				}
			}

			// Trim
			auto newSize = std::distance(activeCandidates.begin(), oit);
			activeCandidates.resize(newSize);
		}

		// If we had a start event, we need to add the candidate
		if (addStartSegmentToCandidates) {
			addToCandidateSet(nextStartEvent);
		}

		// We'll need to sort if we had a start event and added to the candidate set, or if we have advanced
		// our candidate set
		if (addStartSegmentToCandidates || newMajor > lastMajor) {
			std::sort(activeCandidates.begin(), activeCandidates.end(), candidateComparator);
		}

		if (newMajor > lastMajor) {
			lastMajor = newMajor;
		}
	}
}

bool Hatch::split_into_major_monotonic_segments(const QuadraticBezier<real_t>& bezier,
		std::array<QuadraticBezier<real_t>, 2>& result) {
	// Getting derivatives for the quadratic bezier
	auto quadratic = QuadraticCurve<real_t>::from_bezier(bezier.P0, bezier.P1, bezier.P2);
	auto a = quadratic.A[static_cast<int>(SELECTED_MAJOR_AXIS)];
	auto b = quadratic.B[static_cast<int>(SELECTED_MAJOR_AXIS)];

	// Finding roots for the quadratic bezier derivatives (a straight line)
	auto t = -b / (real_t(2.0) * a);

	if (std::isnan(t) || t <= real_t(0.0) || t >= real_t(1.0)) {
		return true;
	}

	result[0] = bezier.split_from_start(t); // lower
	result[1] = bezier.split_to_end(t); // upper

	return false;
}

real_t Hatch::intersect_ortho(const QuadraticBezier<real_t>& bezier, real_t lineConstant, int component) {
	// https://pomax.github.io/bezierinfo/#intersections
	real_t points[] = {bezier.P0[component], bezier.P1[component], bezier.P2[component]};

	for (size_t i = 0; i < 3; ++i) {
		points[i] -= lineConstant;
	}

	auto A = points[0] - real_t(2.0) * points[1] + points[2];
	auto B = real_t(2.0) * (points[1] - points[0]);
	auto C = points[0];

	auto roots = EqQuadratic<real_t>{A, B, C}.compute_roots();

	if (roots.x >= real_t(0.0) && roots.x <= real_t(1.0)) {
		return roots.x;
	}
	else if (roots.y >= real_t(0.0) && roots.y <= real_t(1.0)) {
		return roots.y;
	}

	return std::numeric_limits<real_t>::quiet_NaN();
}


// Assumes the curve is monotonic in the major axis, only considers the t = 0, t = 1, and minor axis extremities
// https://pomax.github.io/bezierinfo/#boundingbox
static Pair<glm::vec<2, real_t>, glm::vec<2, real_t>> calc_bezier_bounding_box_minor(
		const QuadraticBezier<real_t>& bezier) {
	auto minor = static_cast<uint32_t>(SELECTED_MINOR_AXIS);
	auto A = bezier.P0[minor] - real_t(2.0) * bezier.P1[minor] + bezier.P2[minor];
	auto B = real_t(2.0) * (bezier.P1[minor] - bezier.P0[minor]);

	real_t searchT[] = {real_t(0.0), real_t(1.0), -B / (real_t(2.0) * A)};

	glm::vec<2, real_t> bMin{std::numeric_limits<real_t>::infinity(), std::numeric_limits<real_t>::infinity()};
	glm::vec<2, real_t> bMax{-std::numeric_limits<real_t>::infinity(),
			-std::numeric_limits<real_t>::infinity()};

	for (auto t : searchT) {
		if (t < real_t(0.0) || t > real_t(1.0) || std::isnan(t)) {
			continue;
		}

		auto value = bezier.evaluate(t);
		bMin.x = std::min(bMin.x, value.x);
		bMin.y = std::min(bMin.y, value.y);
		bMax.x = std::max(bMax.x, value.x);
		bMax.y = std::max(bMax.y, value.y);
	}

	return {bMin, bMax};
}

std::array<real_t, 4> Hatch::bezier_bezier_intersections(const QuadraticBezier<real_t>& lhs,
		const QuadraticBezier<real_t>& rhs) {
	static constexpr real_t QUARTIC_THRESHOLD = real_t(1e-10);

	auto quarticEquation = get_bezier_bezier_intersection_equation<real_t>(lhs, rhs);

	// Only two candidates in range, ever
	std::array<real_t, 4> t{std::numeric_limits<real_t>::quiet_NaN(), std::numeric_limits<real_t>::quiet_NaN(),
			std::numeric_limits<real_t>::quiet_NaN(), std::numeric_limits<real_t>::quiet_NaN()};

	auto quadCoeffMag = std::max(std::abs(quarticEquation.d), std::abs(quarticEquation.e));
	auto cubCoeffMag = std::max(std::abs(quarticEquation.c), quadCoeffMag);
	auto quartCoeffMag = std::max(std::abs(quarticEquation.b), cubCoeffMag);

	if (std::abs(quarticEquation.a) > quartCoeffMag * QUARTIC_THRESHOLD) {
		auto res = quarticEquation.compute_roots();
		std::memcpy(&t[0], &res.x, sizeof(real_t) * 4);
	}
	else if (abs(quarticEquation.b) > quadCoeffMag * QUARTIC_THRESHOLD) {
		auto res = EqCubic<real_t>{quarticEquation.b, quarticEquation.c, quarticEquation.d, 
				quarticEquation.e}.compute_roots();
		std::memcpy(&t[0], &res.x, sizeof(real_t) * 3);
	}
	else {
		auto res = EqQuadratic<real_t>{quarticEquation.c, quarticEquation.d, quarticEquation.e}.compute_roots();
		std::memcpy(&t[0], &res.x, sizeof(real_t) * 2);
	}

	return t;
}

// Transforms curves into AABB UV space and turns them into quadratic coefficients
static void transform_curves(const QuadraticBezier<real_t>& bezier, const glm::vec<2, real_t>& aabbMin,
		const glm::vec<2, real_t>& aabbMax, glm::vec2* output) {
	auto rcpAabbExtents = glm::vec<2, real_t>(1, 1) / (aabbMax - aabbMin);
	QuadraticBezier<real_t> transformedBezier{
		(bezier.P0 - aabbMin) * rcpAabbExtents,
		(bezier.P1 - aabbMin) * rcpAabbExtents,
		(bezier.P2 - aabbMin) * rcpAabbExtents
	};

	if (is_line_segment(transformedBezier)) {
		transformedBezier.P1 = transformedBezier.P2;
	}

	/*auto quadratic = QuadraticCurve<real_t>::from_bezier(transformedBezier.P0, transformedBezier.P1,
			transformedBezier.P2);

	if (is_line_segment(transformedBezier)) {
		quadratic.A = glm::vec<2, real_t>{0, 0};
	}

	output[0] = quadratic.A;
	output[1] = quadratic.B;
	output[2] = quadratic.C;*/

	output[0] = transformedBezier.P0;
	output[1] = transformedBezier.P1;
	output[2] = transformedBezier.P2;
}

static bool is_line_segment(const QuadraticBezier<real_t>& bezier) {
	//auto quadratic = QuadraticCurve<real_t>::from_bezier(bezier.P0, bezier.P1, bezier.P2);
	//auto lenSqA = glm::dot(quadratic.A, quadratic.A);
	//return lenSqA < std::exp(-23.f) * glm::dot(quadratic.B, quadratic.B);
	
	auto p10 = bezier.P1 - bezier.P0;
	auto p20 = bezier.P2 - bezier.P0;
	auto cr = cross(bezier.P1 - bezier.P0, bezier.P2 - bezier.P0);
	return (cr * cr) / (glm::dot(p10, p10) + glm::dot(p20, p20)) < 1e-4;
}

std::array<real_t, 2> Segment::intersect(const Segment& other) const {
	auto major = static_cast<int>(SELECTED_MAJOR_AXIS);
	auto minor = 1 - major;

	std::array<real_t, 2> result{std::numeric_limits<real_t>::quiet_NaN(),
			std::numeric_limits<real_t>::quiet_NaN()};
	int resultIdx = 0;

	// Use line intersections if one or both of the beziers are linear (a = 0)
	bool selfLinear = is_line_segment(*originalBezier);
	bool otherLinear = is_line_segment(*other.originalBezier);

	if (selfLinear && otherLinear) {
		// Line/line intersection
		auto intersectionPoint =  line_line_intersection<real_t>(originalBezier->P0,
				originalBezier->P2 - originalBezier->P0, other.originalBezier->P0,
				other.originalBezier->P2 - other.originalBezier->P0);

		auto x1 = originalBezier->P0.x, y1 = originalBezier->P0.y;
		auto x2 = originalBezier->P2.x, y2 = originalBezier->P2.y;
		auto x3 = other.originalBezier->P0.x, y3 = other.originalBezier->P0.y;
		auto x4 = other.originalBezier->P2.x, y4 = other.originalBezier->P2.y;

		// Return if point is on the lines
		if (std::min(x1, x2) <= intersectionPoint.x && y1 <= intersectionPoint.y
				&& std::max(x1, x2) >= intersectionPoint.x && y2 >= intersectionPoint.y
				&& std::min(x3, x4) <= intersectionPoint.x && y3 <= intersectionPoint.y
				&& std::max(x3, x4) >= intersectionPoint.x && y4 >= intersectionPoint.y) {
			// Gets t for "other" by using intersectOrtho
			auto otherT = intersect_ortho(*other.originalBezier, intersectionPoint.y, major);
			auto intersectionMajor = other.originalBezier->evaluate(otherT)[major];
			auto thisT = intersect_ortho(*originalBezier, intersectionMajor, major);

			if (otherT >= other.tStart && otherT <= other.tEnd && thisT >= tStart && thisT <= tEnd) {
				result[0] = otherT;
			}
		}
	}
	else if (selfLinear || otherLinear) {
		// Line/curve intersection
		auto& line = selfLinear ? *originalBezier : *other.originalBezier;
		auto& curve = selfLinear ? *other.originalBezier : *originalBezier;

		auto D = glm::normalize(line.P2 - line.P0);
		glm::mat<2, 2, real_t> rotation{ 
			{D.x, -D.y}, 
			{D.y, D.x} 
		};

		QuadraticBezier<real_t> rotatedCurve{
			rotation * (curve.P0 - line.P0),
			rotation * (curve.P1 - line.P0),
			rotation * (curve.P2 - line.P0)
		};

		auto intersectionCurveT = intersect_ortho(rotatedCurve, 0,
				(int)MajorAxis::MAJOR_Y /* Always in rotation to align with X Axis */);
		auto intersectionMajor = other.originalBezier->evaluate(intersectionCurveT)[major];
		auto intersectionLineT = intersect_ortho(line, intersectionMajor, major);

		auto thisT = selfLinear ? intersectionLineT : intersectionCurveT;
		auto otherT = selfLinear ? intersectionCurveT : intersectionLineT;

		if (otherT >= other.tStart && otherT <= other.tEnd && thisT >= tStart && thisT <= tEnd) {
			result[0] = otherT;
		}
	}
	else {
		// to get correct P0, P1, P2 for intersection testing
		auto thisBezier = originalBezier->split_from_min_to_max(tStart, tEnd);

		auto p0 = thisBezier.P0;
		auto p1 = thisBezier.P1;
		auto p2 = thisBezier.P2;
		bool sideP1 = cross(p2 - p0, p1 - p0) >= 0.0;
		
		auto& otherBezier = *other.originalBezier;
		auto intersections = bezier_bezier_intersections(otherBezier, thisBezier);

		for (auto t : intersections) {
			if (std::isnan(t) || other.tStart >= t || t >= other.tEnd) {
				continue;
			}

			auto intersection = otherBezier.evaluate(t);
			
			// Optimization instead of doing SDF to find other T and check against bounds:
			// If both P1 and the intersection point are on the same side of the P0 -> P2 line of thisBezier,
			// it's a a valid intersection
			bool sideIntersection = cross(p2 - p0, intersection - p0) >= 0.0;
			if (sideP1 != sideIntersection) {
				continue;
			}

			bool duplicateT = (resultIdx > 0 && t == result[0]) || (resultIdx > 1 && t == result[1]);
			if (!duplicateT) {
				if (resultIdx < 2) {
					result[resultIdx] = t;
					resultIdx++;
				}
				else {
					assert(false && "more intersections than expected");
				}
			}
		}
	}

	return result;
}

bool Segment::is_straight_line_constant_major() const {
	auto major = static_cast<uint32_t>(SELECTED_MAJOR_AXIS);
	auto p0 = originalBezier->P0[major];
	auto p1 = originalBezier->P1[major];
	auto p2 = originalBezier->P2[major];

	//assert(p0 <= p1 && p1 <= p2); (PRECISION ISSUES ARISE ONCE MORE)
	return std::abs(p1 - p0) <= std::exp2(-24.0) && std::abs(p2 - p0) <= std::exp(-24.0f);
}

bool SegmentLess::operator()(const Segment& a, const Segment& b) const {
	auto lhs = a.originalBezier->evaluate(a.tStart)[minor];
	auto rhs = b.originalBezier->evaluate(b.tStart)[minor];

	auto lenLhs = glm::distance(a.originalBezier->P0, a.originalBezier->P2);
	auto lenRhs = glm::distance(b.originalBezier->P0, b.originalBezier->P2);
	auto minLen = std::fminf(lenLhs, lenRhs);

	// Threshold here for intersection points, where the minor values for the curves are very close but
	// could be smaller, causing the curves to be in the wrong order
	if (std::abs(lhs - rhs) < MINOR_POSITION_COMPARISON_THRESHOLD * minLen) {
		// This is how you want to order the derivatives dmin/dmaj=-IMF dmin/dmaj = 0 dmin/dmaj = INF
		// also leverage the guarantee that `dmaj>=0` to get numerically stable compare
		auto lhsQuadratic = QuadraticCurve<real_t>::from_bezier(a.originalBezier->P0, a.originalBezier->P1,
				a.originalBezier->P2);
		auto rhsQuadratic = QuadraticCurve<real_t>::from_bezier(b.originalBezier->P0, b.originalBezier->P1,
				b.originalBezier->P2);

		auto lTan = a.originalBezier->derivative(a.tStart);
		auto rTan = b.originalBezier->derivative(b.tStart);

		lhs = lTan[minor] * rTan[major];
		rhs = rTan[minor] * lTan[major];

		// Negative values mess with the comparison operator when using multiplication. They should be
		// positive because of major monotonicity
		assert(lTan[major] >= real_t(0.0));
		assert(rTan[major] >= real_t(0.0));

		if (std::abs(lhs - rhs) < TANGENT_COMPARISON_THRESHOLD) {
			auto lAcc = real_t(2.0) * lhsQuadratic.A;
			auto rAcc = real_t(2.0) * rhsQuadratic.A;

			// In this branch, lhs == rhs == 0 (tangents are both 0)
			if (std::abs(lhs - real_t(0.0)) < TANGENT_COMPARISON_THRESHOLD) {
				bool lTanSign = lTan[minor] >= real_t(0.0);
				bool rTanSign = rTan[minor] >= real_t(0.0);

				// CASE A: If the signs of the horizontal tangents differ, we know the negative one belongs
				// to the left curve
				if (lTanSign != rTanSign) {
					return rTanSign;
				}

				// CASE B: Otherwise, in this case the tangents are both on the same side, so only the
				// magnitude/abs of the d2Major/dMinor2 is important
				lhs = -std::abs(lAcc[minor] * lTan[major] - lAcc[major] * lTan[minor])
						* std::pow(rTan[minor], real_t(3.0));
				rhs = -std::abs(rAcc[minor] * rTan[major] - rAcc[major] * rTan[minor])
						* std::pow(lTan[minor], real_t(3.0));
			}
			else {
				lhs = (lAcc[minor] * lTan[major] - lAcc[major] * lTan[minor])
						* std::pow(rTan[major], real_t(3.0));
				rhs = (rAcc[minor] * rTan[major] - rAcc[major] * rTan[minor])
						* std::pow(lTan[major], real_t(3.0));
			}
		}
	}

	return lhs < rhs;
}
