#include "bezier_martinez_impl.hpp"

#include "svg.hpp"
#include "hatch_impl.hpp"
#include "martinez_impl.hpp"

#include <cfloat>
#include <set>
#include <utility>

#include <glm/gtc/epsilon.hpp>
#include <glm/mat2x2.hpp>

using namespace BezierMartinez;

static constexpr const AABB AABB_INVALID = {FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};

static bool point_equals(const glm::vec<2, real_t>& a, const glm::vec<2, real_t>& b) {
	return glm::all(glm::epsilonEqual(a, b, Martinez::LINEAR_EPSILON));
}

static void process_path(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const CPUQuadraticPath& contourOrHole, AABB& bbox, bool isSubject, uint32_t contourID);

static void possible_intersection(SweepEvent& a, SweepEvent& b, EventQueue& queue,
		std::vector<std::unique_ptr<SweepEvent>>& eventOwner);
static void divide_segment(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const QuadraticBezier<real_t>& bezier, real_t t, const glm::vec<2, real_t>& p,
		SweepEvent*& lastSplitLeft, SweepEvent* lastSplitRight);

static Contour& initialize_contour_from_context(std::vector<Contour>& contours, const SweepEvent& event,
		uint32_t contourID);

static void deep_copy_shape(CPUQuadraticShape& dst, const CPUQuadraticShape& src);

void martinez_boolean_bezier(const CPUQuadraticShape& subject, const CPUQuadraticShape& clipping,
		CPUQuadraticShape& result, BooleanOperation operation) {
	// Check trivial results: when at least one shape is empty
	if (subject.paths.empty() || clipping.paths.empty()) {
		switch (operation) {
			case BooleanOperation::INTERSECTION:
				return;
			case BooleanOperation::DIFFERENCE:
				deep_copy_shape(result, subject);
				return;
			case BooleanOperation::UNION:
			case BooleanOperation::XOR:
				deep_copy_shape(result, subject.paths.empty() ? clipping : subject);
				return;
		}
	}

	auto sbbox = AABB_INVALID;
	auto cbbox = AABB_INVALID;

	EventQueue eventQueue;
	std::vector<std::unique_ptr<SweepEvent>> eventOwner;
	fill_queue(eventQueue, eventOwner, subject, clipping, sbbox, cbbox, operation);

	// Trivial result: Complete lack of intersection
	if (!sbbox.intersects(cbbox)) {
		switch (operation) {
			case BooleanOperation::INTERSECTION:
				return;
			case BooleanOperation::DIFFERENCE:
				deep_copy_shape(result, subject);
				return;
			case BooleanOperation::UNION:
			case BooleanOperation::XOR:
				deep_copy_shape(result, subject);
				deep_copy_shape(result, clipping);
				return;
		}
	}

	std::vector<SweepEvent*> sortedEvents;
	subdivide_segments(eventQueue, eventOwner, sortedEvents, subject, clipping, sbbox, cbbox, operation);

	std::vector<Contour> contours;
	connect_edges(sortedEvents, contours, operation);

	result.fillColor = subject.fillColor;
	result.strokeColor = subject.strokeColor;
	result.strokeWidth = subject.strokeWidth;

	// Convert contours to polygons
	for (auto& contour : contours) {
		// FIXME: not accounting for this being a polygon set
		if (contour.is_exterior()) {
			// The exterior ring goes first
			{
				result.paths.emplace_back();
				auto& path = result.paths.back();
				path.points.reserve(contour.points.size());

				for (auto& point : contour.points) {
					path.points.emplace_back(point.x, point.y);
				}
			}

			// Followed by holes, if any
			for (auto holeID : contour.holeIDs) {
				auto& hole = contours[holeID].points;
				auto& path = result.paths.back();
				path.points.reserve(hole.size());

				for (auto& point : hole) {
					path.points.emplace_back(point.x, point.y);
				}
			}
		}
	}
}

void BezierMartinez::fill_queue(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const CPUQuadraticShape& subject, const CPUQuadraticShape& clipping, AABB& sbbox, AABB& cbbox,
		BooleanOperation operation) {
	uint32_t contourID = 0;

	for (size_t i = 0; i < subject.paths.size(); ++i) {
		bool isExteriorRing = i == 0;

		if (isExteriorRing) {
			++contourID;
		}

		process_path(queue, eventOwner, subject.paths[i], sbbox, true, contourID);
	}

	for (size_t i = 0; i < clipping.paths.size(); ++i) {
		bool isExteriorRing = i == 0 && operation != BooleanOperation::DIFFERENCE;

		if (isExteriorRing) {
			++contourID;
		}

		process_path(queue, eventOwner, clipping.paths[i], cbbox, false, contourID);
	}
}

static void add_monotonic_bezier(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const QuadraticBezier<real_t>& bezier, bool isSubject, uint32_t contourID) {
	static constexpr const int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);

	auto outputBezier = bezier;

	if (outputBezier.P0[major] > outputBezier.P2[major]) {
		outputBezier.P2 = bezier.P0;
		outputBezier.P0 = bezier.P2;
		assert(outputBezier.P0[major] <= outputBezier.P2[major]);
	}

	// Fix in case of small precision issues when splitting into majro monotonic segments
	if (outputBezier.P1[major] < outputBezier.P0[major]) {
		outputBezier.P1[major] = outputBezier.P0[major];
	}

	//outputBezier.P1 = (outputBezier.P0 + outputBezier.P2) * 0.5;

	auto e1 = std::make_unique<SweepEvent>(outputBezier.P0, 0.0, outputBezier, nullptr, contourID, false,
			isSubject);
	auto e2 = std::make_unique<SweepEvent>(outputBezier.P2, 1.0, outputBezier, e1.get(), contourID, false,
			isSubject);
	e1->otherEvent = e2.get();

	if (CompareEvents{}(e1.get(), e2.get()) == std::strong_ordering::greater) {
		e2->left = true;
	}
	else {
		e1->left = true;
	}

	queue.push(e1.get());
	queue.push(e2.get());

	eventOwner.emplace_back(std::move(e1));
	eventOwner.emplace_back(std::move(e2));
}

static void process_path(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const CPUQuadraticPath& path, AABB& bbox, bool isSubject, uint32_t contourID) {
	for (size_t i = 0; i < path.points.size() - 1; i += 2) {
		QuadraticBezier<real_t> bezier{path.points[i], path.points[i + 1], path.points[i + 2]};
		auto bounds = bezier.calc_aabb();

		bbox.minX = std::fmin(bounds[0], bbox.minX);
		bbox.minY = std::fmin(bounds[1], bbox.minY);
		bbox.maxX = std::fmax(bounds[2], bbox.maxX);
		bbox.maxY = std::fmax(bounds[3], bbox.maxY);

		std::array<QuadraticBezier<real_t>, 2> monotonicSegments;
		bool alreadyMonotonic = Hatch::split_into_major_monotonic_segments(bezier, monotonicSegments);

		if (alreadyMonotonic) {
			add_monotonic_bezier(queue, eventOwner, bezier, isSubject, contourID);
		}
		else {
			add_monotonic_bezier(queue, eventOwner, monotonicSegments[0], isSubject, contourID);
			add_monotonic_bezier(queue, eventOwner, monotonicSegments[1], isSubject, contourID);
		}
	}
}

void BezierMartinez::subdivide_segments(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		std::vector<SweepEvent*>& sortedEvents, const CPUQuadraticShape& subject,
		const CPUQuadraticShape& clipping, const AABB& sbbox, const AABB& cbbox, BooleanOperation operation) {
	static constexpr const int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);
	static constexpr const int minor = 1 - major; 

	std::multiset<SweepEvent*, CompareSegmentsLess> sweepLine;

	auto rightBound = Hatch::SELECTED_MAJOR_AXIS == Hatch::MajorAxis::MAJOR_X
			? std::min(sbbox.maxX, cbbox.maxX)
			: std::min(sbbox.maxY, cbbox.maxY);
	auto subjMaxBound = Hatch::SELECTED_MAJOR_AXIS == Hatch::MajorAxis::MAJOR_X ? sbbox.maxX : sbbox.maxY;

	while (!queue.empty()) {
		auto* pEvent = queue.top();
		queue.pop();

		sortedEvents.emplace_back(pEvent);

		if ((operation == BooleanOperation::INTERSECTION && pEvent->point[major] > rightBound)
				|| (operation == BooleanOperation::DIFFERENCE && pEvent->point[major] > subjMaxBound)) {
			break;
		}

		if (pEvent->left) {
			auto next = sweepLine.insert(pEvent);
			auto prev = next;
			auto begin = sweepLine.begin();
			bool hasPrev = prev != begin;

			if (hasPrev) {
				--prev;
			}

			++next;

			auto* prevEvent = hasPrev ? *prev : nullptr;
			compute_fields(*pEvent, prevEvent, operation);

			if (next != sweepLine.end()) {
				possible_intersection(*pEvent, **next, queue, eventOwner);
			}

			if (hasPrev) {
				possible_intersection(*prevEvent, *pEvent, queue, eventOwner);
			}
		}
		else {
			pEvent = pEvent->otherEvent;
			auto curr = sweepLine.find(pEvent);
			auto next = curr;
			auto prev = curr;

			if (next != sweepLine.end()) {
				++next;

				if (prev != sweepLine.begin() && next != sweepLine.end()) {
					--prev;
					possible_intersection(**prev, **next, queue, eventOwner);
				}

				sweepLine.erase(curr);
			}
		}
	}
}

static real_t cross(const glm::vec<2, real_t>& a, const glm::vec<2, real_t>& b) {
	return a.x * b.y - a.y * b.x;
}

template <typename Functor>
static int handle_intersections(const QuadraticBezier<real_t>& bezierA, real_t startA, real_t endA,
		const QuadraticBezier<real_t>& bezierB, real_t startB, real_t endB, Functor&& functor) {
	static constexpr const auto major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);

	bool isLinearA = Hatch::is_line_segment(bezierA);
	bool isLinearB = Hatch::is_line_segment(bezierB);

	if (isLinearA && isLinearB) {
		auto va = bezierA.P2 - bezierA.P0;
		auto vb = bezierB.P2 - bezierB.P0;
		auto e = bezierB.P0 - bezierA.P0;

		if (auto denom = cross(va, vb); denom * denom > real_t(0.0)) {
			auto s = cross(e, vb) / denom;

			// Not on line segment A
			if (s < startA || s > endA) {
				return 0;
			}

			auto t = cross(e, va) / denom;

			// Not on line segment B
			if (t < startB || t > endB) {
				return 0;
			}

			// The line segments intersect at an end point of both line segments. For the purposes of this
			// algorithm it is discarded
			if ((s == startA || s == endA) && (t == startB || t == endB)) {
				return 0;
			}

			functor(s, t, bezierB.P0 + vb * t);

			return 1;
		}

		// FIXME: Colinear case
	}
	else if (isLinearA || isLinearB) {
		auto& line = isLinearA ? bezierA : bezierB;
		auto& curve = isLinearA ? bezierB : bezierA;

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

		auto intersectionCurveT = Hatch::intersect_ortho(rotatedCurve, 0,
				(int)Hatch::MajorAxis::MAJOR_Y /* Always in rotation to align with X Axis */);
		auto intersection = bezierB.evaluate(intersectionCurveT);
		auto intersectionLineT = Hatch::intersect_ortho(line, intersection[major], major);

		auto tA = isLinearA ? intersectionLineT : intersectionCurveT;
		auto tB = isLinearA ? intersectionCurveT : intersectionLineT;

		// The line segments intersect at an end point of both segments. For the purposes of this
		// algorithm it is discarded
		if ((tA == startA || tA == endA) && (tB == startB || tB == endB)) {
			return 0;
		}

		functor(tA, tB, intersection);

		return 1;
	}
	else {
		int resultIdx = 0;
		auto lastResult = std::numeric_limits<real_t>::quiet_NaN();

		// to get correct P0, P1, P2 for intersection testing
		auto thisBezier = bezierA.split_from_min_to_max(startA, endA);

		auto p0 = thisBezier.P0;
		auto p1 = thisBezier.P1;
		auto p2 = thisBezier.P2;
		bool sideP1 = cross(p2 - p0, p1 - p0) >= 0.0;
		
		auto intersections = Hatch::bezier_bezier_intersections(bezierB, thisBezier);

		for (auto t : intersections) {
			if (std::isnan(t) || startB >= t || t >= endB) {
				continue;
			}

			auto intersection = bezierB.evaluate(t);
			
			// Optimization instead of doing SDF to find other T and check against bounds:
			// If both P1 and the intersection point are on the same side of the P0 -> P2 line of thisBezier,
			// it's a a valid intersection
			bool sideIntersection = cross(p2 - p0, intersection - p0) >= 0.0;
			if (sideP1 != sideIntersection) {
				continue;
			}

			bool duplicateT = t == lastResult;
			if (!duplicateT) {
				if (resultIdx < 2) {
					auto s = Hatch::intersect_ortho(bezierA, intersection[major], major);

					if (std::isnan(s) || s < startA || s > endA) {
						continue;
					}

					//printf("%g(%g) <= %g <= %g(%g) | %g\n", bezierA.P0[major], thisBezier.P0[major],
							//intersection[major], bezierA.P2[major], thisBezier.P2[major], s);

					functor(s, t, intersection);
					lastResult = t;
					++resultIdx;
				}
				else {
					assert(false && "more intersections than expected");
				}
			}
		}

		return resultIdx;
	}

	return 0;
}

static void possible_intersection(SweepEvent& a, SweepEvent& b, EventQueue& queue,
		std::vector<std::unique_ptr<SweepEvent>>& eventOwner) {
	static constexpr const int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);

	auto* lastLeftA = &a;
	auto* lastRightA = a.otherEvent;
	auto* lastLeftB = &b;
	auto* lastRightB = b.otherEvent;

	handle_intersections(a.bezier, a.t, a.otherEvent->t, b.bezier, b.t, b.otherEvent->t,
			[&](real_t tA, real_t tB, const glm::vec<2, real_t>& point) {
		bool isEndOfA = tA == lastLeftA->t || tA == lastRightA->t;
		bool isEndOfB = tB == lastLeftB->t || tB == lastRightB->t;

		if (isEndOfA && !isEndOfB) {
			divide_segment(queue, eventOwner, b.bezier, tB, point, lastLeftB, lastRightB);
		}
		else if (isEndOfB && !isEndOfA) {
			divide_segment(queue, eventOwner, a.bezier, tA, point, lastLeftA, lastRightA);
		}
		else if (!isEndOfA && !isEndOfB) {
			divide_segment(queue, eventOwner, a.bezier, tA, point, lastLeftA, lastRightA);
			divide_segment(queue, eventOwner, b.bezier, tB, point, lastLeftB, lastRightB);
		}
	});

	/*Hatch::Segment segA{.originalBezier = &a.bezier, .tStart = a.t, .tEnd = a.otherEvent->t};
	Hatch::Segment segB{.originalBezier = &b.bezier, .tStart = b.t, .tEnd = b.otherEvent->t};

	auto intersections = segA.intersect(segB);

	for (auto tB : intersections) {
		if (std::isnan(tB)) {
			continue;
		}

		auto pB = b.bezier.evaluate(tB);
		// Due to precision, if the curve is right at the end, intersect_ortho may return nan
		auto tA = Hatch::intersect_ortho(a.bezier, pB[major], major);
		tA = std::isnan(tA) ? lastRightA->t : tA;

		bool isEndOfA = tA == lastLeftA->t || tA == lastRightA->t;
		bool isEndOfB = tB <= lastLeftB->t || tB == lastRightB->t;

		if (isEndOfA && isEndOfB) {
			continue;
		}
		else if (isEndOfA) {
			divide_segment(queue, eventOwner, b.bezier, tB, pB, lastLeftB, lastRightB);
		}
		else if (isEndOfB) {
			divide_segment(queue, eventOwner, a.bezier, tA, pB, lastLeftA, lastRightA);
		}
		else {
			divide_segment(queue, eventOwner, b.bezier, tB, pB, lastLeftB, lastRightB);
			divide_segment(queue, eventOwner, a.bezier, tA, pB, lastLeftA, lastRightA);
		}
	}*/
}

static void divide_segment(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const QuadraticBezier<real_t>& bezier, real_t t, const glm::vec<2, real_t>& p,
		SweepEvent*& lastSplitLeft, SweepEvent* lastSplitRight) {
	auto r = std::make_unique<SweepEvent>(p, t, bezier, lastSplitLeft, lastSplitLeft->contourID, false,
			lastSplitLeft->isSubject);
	auto l = std::make_unique<SweepEvent>(p, t, bezier, lastSplitRight, lastSplitLeft->contourID, true,
			lastSplitLeft->isSubject);

	if (CompareEvents{}(l.get(), lastSplitLeft->otherEvent) == std::strong_ordering::greater) {
		lastSplitLeft->otherEvent->left = true;
		l->left = false;
	}

	lastSplitLeft->otherEvent->otherEvent = l.get();
	lastSplitLeft->otherEvent = r.get();

	lastSplitLeft = l.get();

	queue.push(l.get());
	queue.push(r.get());

	eventOwner.emplace_back(std::move(l));
	eventOwner.emplace_back(std::move(r));
}

static bool compute_in_result(const SweepEvent& event, BooleanOperation operation) {
	switch (event.edgeType) {
		case EdgeType::NORMAL:
			switch (operation) {
				case BooleanOperation::INTERSECTION:
					return !event.otherInOut;
				case BooleanOperation::UNION:
					return event.otherInOut;
				case BooleanOperation::DIFFERENCE:
					return (event.isSubject && event.otherInOut)
							|| (!event.isSubject && !event.otherInOut);
				case BooleanOperation::XOR:
					return true;
			}

			std::unreachable();
		case EdgeType::SAME_TRANSITION:
			return operation == BooleanOperation::INTERSECTION || operation == BooleanOperation::UNION;
		case EdgeType::DIFFERENT_TRANSITION:
			return operation == BooleanOperation::DIFFERENCE;
		case EdgeType::NON_CONTRIBUTING:
			return false;
	}

	std::unreachable();
}

static ResultTransition determine_result_transition(SweepEvent& event, BooleanOperation operation) {
	bool thisIn = !event.inOut;
	bool thatIn = !event.otherInOut;

	switch (operation) {
		case BooleanOperation::INTERSECTION:
			return (thisIn && thatIn) ? ResultTransition::OUT_IN : ResultTransition::IN_OUT;
		case BooleanOperation::UNION:
			return (thisIn || thatIn) ? ResultTransition::OUT_IN : ResultTransition::IN_OUT;
		case BooleanOperation::XOR:
			return (thisIn ^ thatIn) ? ResultTransition::OUT_IN : ResultTransition::IN_OUT;
		case BooleanOperation::DIFFERENCE:
			if (event.isSubject) {
				return (thisIn && !thatIn) ? ResultTransition::OUT_IN : ResultTransition::IN_OUT;
			}
			else {
				return (thatIn && !thisIn) ? ResultTransition::OUT_IN : ResultTransition::IN_OUT;
			}
	}

	std::unreachable();
}

void BezierMartinez::compute_fields(SweepEvent& event, SweepEvent* prev, BooleanOperation operation) {
	// Compute inOut and otherInOut fields
	if (!prev) {
		event.inOut = false;
		event.otherInOut = true;
	}
	else {
		// Previous line segment in sweepLine belongs to the same polygon
		if (event.isSubject == prev->isSubject) {
			event.inOut = !prev->inOut;
			event.otherInOut = prev->otherInOut;
		}
		// Previous line segment in sweepLine belongs to the clipping polygon
		else {
			event.inOut = !prev->otherInOut;
			event.otherInOut = prev->is_vertical() ? !prev->inOut : prev->inOut;
		}

		// compute prevInResult field
		event.prevInResult = (!compute_in_result(*prev, operation) || prev->is_vertical())
				? prev->prevInResult : prev;
	}

	// Check if the line segment belongs to the Boolean operation
	if (compute_in_result(event, operation)) {
		event.resultTransition = determine_result_transition(event, operation);
	}
	else {
		event.resultTransition = ResultTransition::NOT_IN_RESULT;
	}
}

void BezierMartinez::connect_edges(std::vector<SweepEvent*>& sortedEvents, std::vector<Contour>& contours,
		BooleanOperation operation) {
	std::vector<SweepEvent*> resultEvents;
	order_events(sortedEvents, resultEvents);

	std::vector<bool> processed;
	processed.resize(resultEvents.size());

	for (size_t i = 0; i < resultEvents.size(); ++i) {
		if (processed[i]) {
			continue;
		}

		auto contourID = static_cast<uint32_t>(contours.size());
		auto& contour = initialize_contour_from_context(contours, *resultEvents[i], contourID);

		auto pos = i;
		auto origPos = i;
		auto initial = resultEvents[i]->point;

		contour.points.emplace_back(initial);

		for (;;) {
			processed[pos] = true;
			resultEvents[pos]->outputContourID = contourID;

			auto tStart = resultEvents[pos]->t;
			bool left = resultEvents[pos]->left;

			pos = resultEvents[pos]->otherPos;

			auto tEnd = resultEvents[pos]->t;

			processed[pos] = true;
			resultEvents[pos]->outputContourID = contourID;

			if (!left) {
				std::swap(tStart, tEnd);
			}

			auto bezier = resultEvents[pos]->bezier.split_from_min_to_max(tStart, tEnd);

			contour.points.emplace_back(bezier.P1);
			contour.points.emplace_back(resultEvents[pos]->point);

			pos = next_pos(pos, resultEvents, processed, origPos);

			if (pos == origPos || pos >= resultEvents.size()) {
				break;
			}
		}
	}
}

void BezierMartinez::order_events(const std::vector<SweepEvent*>& sortedEvents,
		std::vector<SweepEvent*>& resultEvents) {
	for (auto* event : sortedEvents) {
		if ((event->left && event->is_in_result()) || (!event->left && event->otherEvent->is_in_result())) {
			resultEvents.emplace_back(event);
		}
	}

	// Due to overlapping edges the resultEvents array cannot be fully sorted
	bool sorted = false;
	while (!sorted) {
		sorted = true;

		for (size_t i = 0; i < resultEvents.size(); ++i) {
			// FIXME: size() - 1?
			if ((i + 1) < resultEvents.size()
					&& CompareEvents{}(resultEvents[i], resultEvents[i + 1]) == std::strong_ordering::greater) {
				std::swap(resultEvents[i], resultEvents[i + 1]);
				sorted = false;
			}
		}
	}

	for (size_t i = 0; i < resultEvents.size(); ++i) {
		resultEvents[i]->otherPos = i;
	}

	// Imagine the right event is found in the beginning of the queue when the left counterpart is not marked
	// yet
	for (size_t i = 0; i < resultEvents.size(); ++i) {
		auto& event = *resultEvents[i];

		if (!event.left) {
			std::swap(event.otherPos, event.otherEvent->otherPos);
		}
	}
}

static Contour& initialize_contour_from_context(std::vector<Contour>& contours, const SweepEvent& event,
		uint32_t contourID) {
	contours.emplace_back();
	auto& contour = contours.back();

	if (event.prevInResult) {
		auto& prevInResult = *event.prevInResult;

		// Note that it is valid to query the "previous in result" for its output contour ID
		// because we must have already processed it (i.e. assigned an output contour ID) in an earlier
		// iteration, otherwise it wouldn't be possible that it is "previous in result"
		auto lowerContourID = prevInResult.outputContourID;
		auto lowerResultTransition = prevInResult.resultTransition;

		if (lowerResultTransition == ResultTransition::OUT_IN) {
			// We are inside. Now we have to check if the thing below us is another hole or an exterior
			// contour.
			auto& lowerContour = contours[lowerContourID];

			// The lower contour is a hole => Connect the new contour as a hole to its parent, and use the
			// same depth
			if (lowerContour.holeOf != ~0u) {
				auto parentContourID = lowerContour.holeOf;
				contours[parentContourID].holeIDs.emplace_back(contourID);
				contour.holeOf = parentContourID;
				contour.depth = contours[lowerContourID].depth;
			}
			// The lower contour is an exterior contour => Connect the new contour as a hole, and increment
			// depth
			else {
				lowerContour.holeIDs.emplace_back(contourID);
				contour.holeOf = lowerContourID;
				contour.depth = lowerContour.depth + 1;
			}
		}
		// We are outside => this contour is an exterior contour of the same depth
		else {
			contour.holeOf = ~0u;
			contour.depth = contours[lowerContourID].depth;
		}
	}
	// There is no lower/previous contour => this contour is an exterior contour of depth 0.
	else {
		contour.holeOf = ~0u;
		contour.depth = 0;
	}

	return contour;
}

size_t BezierMartinez::next_pos(size_t pos, const std::vector<SweepEvent*>& resultEvents,
		const std::vector<bool>& processed, size_t origPos) {
	auto newPos = pos + 1;
	auto p = resultEvents[pos]->point;

	if (newPos < resultEvents.size()) {
		auto p1 = resultEvents[newPos]->point;

		while (newPos < resultEvents.size() && point_equals(p, p1)) {
			if (!processed[newPos]) {
				return newPos;
			}
			else {
				++newPos;
			}

			if (newPos < resultEvents.size()) {
				p1 = resultEvents[newPos]->point;
			}
		}
	}

	newPos = pos - 1;

	while (processed[newPos] && newPos > origPos) {
		--newPos;
	}

	return newPos;
}

static real_t signed_area(const glm::vec<2, real_t>& p0, const glm::vec<2, real_t>& p1,
		const glm::vec<2, real_t>& p2) {
	if constexpr (Hatch::SELECTED_MAJOR_AXIS == Hatch::MajorAxis::MAJOR_X) {
		return Martinez::signed_area(p0, p1, p2);
	}
	else {
		return -Martinez::signed_area(p0, p1, p2);
	}
}

static void deep_copy_shape(CPUQuadraticShape& dst, const CPUQuadraticShape& src) {
	for (auto& path : src.paths) {
		dst.paths.emplace_back();
		auto& dstPath = dst.paths.back();
		dstPath.points = path.points;
		dstPath.flags = path.flags;
	}

	dst.fillColor = src.fillColor;
	dst.strokeColor = src.strokeColor;
	dst.strokeWidth = src.strokeWidth;
}

bool SweepEvent::is_below(const glm::vec<2, real_t>& p) const {
	return left
			? (signed_area(point, otherEvent->point, p) > 0)
			: (signed_area(otherEvent->point, point, p) > 0);
}

bool SweepEvent::is_above(const glm::vec<2, real_t>& p) const {
	return !is_below(p);
}

bool SweepEvent::is_vertical() const {
	static constexpr const int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);
	// NOTE: Candidate for epsilonEqual
	//return point[major] == otherEvent->point[major];
	return std::abs(point[major] - otherEvent->point[major]) < Martinez::LINEAR_EPSILON;
}

bool SweepEvent::is_in_result() const {
	return resultTransition != ResultTransition::NOT_IN_RESULT;
}

std::strong_ordering CompareEvents::operator()(const SweepEvent* a, const SweepEvent* b) const {
	static constexpr const int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);
	static constexpr const int minor = 1 - major; 

	// Different major coordinate
	if (std::abs(a->point[major] - b->point[major]) > Martinez::LINEAR_EPSILON) {
		if (a->point[major] > b->point[major]) {
			return std::strong_ordering::greater;
		}
		else if (a->point[major] < b->point[major]) {
			return std::strong_ordering::less;
		}
	}

	// Different points, but same major coordinate
	if (std::abs(a->point[minor] - b->point[minor]) > Martinez::LINEAR_EPSILON) {
		if (a->point[minor] > b->point[minor]) {
			return std::strong_ordering::greater;
		}
		else if (a->point[minor] < b->point[minor]) {
			return std::strong_ordering::less;
		}
	}

	// Special cases:
	// 1. same coordinates, but one is a left endpoint and the other is a right endpoint. The right
	// endpoint is processed first
	if (a->left != b->left) {
		return a->left ? std::strong_ordering::greater : std::strong_ordering::less;
	}

	// Same coordinates, both events are left endpoints or right endpoints, but not collinear
	//if (signed_area(a->point, a->otherEvent->point, b->otherEvent->point) != 0) {
	if (!Martinez::points_are_collinear(a->point, a->otherEvent->point, b->otherEvent->point)) {
		// The event associated with the bottom segment is processed first
		return a->is_below(b->otherEvent->point) ? std::strong_ordering::less : std::strong_ordering::greater;
	}

	return (a->isSubject || !b->isSubject) ? std::strong_ordering::less : std::strong_ordering::greater;
}

bool CompareEventsGreater::operator()(const SweepEvent* a, const SweepEvent* b) const {
	return CompareEvents{}(a, b) == std::strong_ordering::greater;
}

std::strong_ordering CompareSegments::operator()(const SweepEvent* a, const SweepEvent* b) const {
	static constexpr const int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);
	static constexpr const int minor = 1 - major; 

	Hatch::Segment segA{&a->bezier, a->t, a->otherEvent->t};
	Hatch::Segment segB{&b->bezier, b->t, b->otherEvent->t};
	return Hatch::SegmentLess{}(segA, segB) ? std::strong_ordering::less : std::strong_ordering::greater;

	if (a == b) {
		return std::strong_ordering::equal;
	}

	// Segments are not collinear
	if (!Martinez::points_are_collinear(a->point, a->otherEvent->point, b->point)
			|| !Martinez::points_are_collinear(a->point, a->otherEvent->point, b->otherEvent->point)) {
		// If they share their left endpoint use the right endpoint to sort
		// NOTE: Candidate for epsilonEqual
		//if (a->point == b->point) {
		if (point_equals(a->point, b->point)) {
			return a->is_below(b->otherEvent->point) ? std::strong_ordering::less
					: std::strong_ordering::greater;
		}

		// Different left endpoint: use the left endpoint to sort
		//if (a->point[major] == b->point[major]) {
		if (std::abs(a->point[major] - b->point[major]) < Martinez::LINEAR_EPSILON) {
			return a->point[minor] < b->point[minor] ? std::strong_ordering::less
					: std::strong_ordering::greater;
			}

		// Has the line segment associated with a been inserted into S after the line segment associated
		// with b?
		if (CompareEvents{}(a, b) == std::strong_ordering::greater) {
			return b->is_above(a->point) ? std::strong_ordering::less : std::strong_ordering::greater;
		}

		// The line segment associated with b has been inserted into S after the line segment associated
		// with a
		return a->is_below(b->point) ? std::strong_ordering::less : std::strong_ordering::greater;
	}

	// Same polygon
	if (a->isSubject == b->isSubject) {
		// NOTE: Candidate for epsilonEqual
		//if (a->point == b->point) {
		if (point_equals(a->point, b->point)) {
			// NOTE: Candidate for epsilonEqual
			//if (a->otherEvent->point == b->otherEvent->point) {
			if (point_equals(a->otherEvent->point, b->otherEvent->point)) {
				return std::strong_ordering::equal;
			}

			return a->contourID <= b->contourID ? std::strong_ordering::less : std::strong_ordering::greater;
		}
	}
	// Segments are collinear but belong to separate polygons
	else {
		return a->isSubject ? std::strong_ordering::less : std::strong_ordering::greater;
	}

	return CompareEvents{}(a, b);
}

bool CompareSegmentsLess::operator()(const SweepEvent* a, const SweepEvent* b) const {
	return CompareSegments{}(a, b) == std::strong_ordering::less;
}

bool Contour::is_exterior() const {
	return holeOf == ~0u;
}
