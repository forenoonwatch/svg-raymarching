#include "bezier_martinez_impl.hpp"

#include "svg.hpp"
#include "hatch_impl.hpp"
#include "martinez_impl.hpp"

#include <utility>

#include <glm/gtc/epsilon.hpp>

using namespace BezierMartinez;

static bool point_equals(const glm::vec<2, real_t>& a, const glm::vec<2, real_t>& b) {
	return glm::any(glm::epsilonEqual(a, b, real_t(1e-4)));
}

static void process_path(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const CPUQuadraticPath& contourOrHole, bool isSubject, uint32_t contourID);

void martinez_boolean_bezier(const CPUQuadraticShape& subject, const CPUQuadraticShape& clipping,
		CPUQuadraticShape& result, BooleanOperation operation) {
	EventQueue eventQueue;
	std::vector<std::unique_ptr<SweepEvent>> eventOwner;
	fill_queue(eventQueue, eventOwner, subject, clipping, operation);

	std::vector<SweepEvent*> sortedEvents;
	subdivide_segments(eventQueue, eventOwner, sortedEvents, subject, clipping, operation);
}

void BezierMartinez::fill_queue(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const CPUQuadraticShape& subject, const CPUQuadraticShape& clipping, BooleanOperation operation) {
	uint32_t contourID = 0;

	for (size_t i = 0; i < subject.paths.size(); ++i) {
		bool isExteriorRing = i == 0;

		if (isExteriorRing) {
			++contourID;
		}

		process_path(queue, eventOwner, subject.paths[i], true, contourID);
	}

	for (size_t i = 0; i < clipping.paths.size(); ++i) {
		bool isExteriorRing = i == 0 && operation != BooleanOperation::DIFFERENCE;

		if (isExteriorRing) {
			++contourID;
		}

		process_path(queue, eventOwner, clipping.paths[i], false, contourID);
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
	/*if (outputBezier.P1.y < outputBezier.P0.y) {
		outputBezier.P1.y = outputBezier.P0.y;
	}*/

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
		const CPUQuadraticPath& path, bool isSubject, uint32_t contourID) {
	for (size_t i = 0; i < path.points.size() - 1; i += 2) {
		QuadraticBezier<real_t> bezier{path.points[i], path.points[i + 1], path.points[i + 2]};

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
		const CPUQuadraticShape& clipping, BooleanOperation operation) {
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

static float signed_area(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2) {
	if constexpr (Hatch::SELECTED_MAJOR_AXIS == Hatch::MajorAxis::MAJOR_X) {
		return Martinez::signed_area(p0, p1, p2);
	}
	else {
		return -Martinez::signed_area(p0, p1, p2);
	}
}

bool SweepEvent::is_below(const glm::vec2& p) const {
	return left
			? (signed_area(point, otherEvent->point, p) > 0)
			: (signed_area(otherEvent->point, point, p) > 0);
}

bool SweepEvent::is_above(const glm::vec2& p) const {
	return !is_below(p);
}

bool SweepEvent::is_vertical() const {
	static constexpr const int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);
	// NOTE: Candidate for epsilonEqual
	return point[major] == otherEvent->point[major];
}

std::strong_ordering CompareEvents::operator()(const SweepEvent* a, const SweepEvent* b) const {
	static constexpr const int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);
	static constexpr const int minor = 1 - major; 

	// Different major coordinate
	if (a->point[major] > b->point[major]) {
		return std::strong_ordering::greater;
	}
	else if (a->point[major] < b->point[major]) {
		return std::strong_ordering::less;
	}

	// Different points, but same major coordinate
	if (a->point[minor] > b->point[minor]) {
		return std::strong_ordering::greater;
	}
	else if (a->point[minor] < b->point[minor]) {
		return std::strong_ordering::less;
	}

	// Special cases:
	// 1. same coordinates, but one is a left endpoint and the other is a right endpoint. The right
	// endpoint is processed first
	if (a->left != b->left) {
		return a->left ? std::strong_ordering::greater : std::strong_ordering::less;
	}

	// Same coordinates, both events are left endpoints or right endpoints, but not collinear
	if (signed_area(a->point, a->otherEvent->point, b->otherEvent->point) != 0) {
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

	if (a == b) {
		return std::strong_ordering::equal;
	}

	// Segments are not collinear
	if (signed_area(a->point, a->otherEvent->point, b->point) != 0
			|| signed_area(a->point, a->otherEvent->point, b->otherEvent->point) != 0) {
		// If they share their left endpoint use the right endpoint to sort
		// NOTE: Candidate for epsilonEqual
		//if (a->point == b->point) {
		if (point_equals(a->point, b->point)) {
			return a->is_below(b->otherEvent->point) ? std::strong_ordering::less
					: std::strong_ordering::greater;
		}

		// Different left endpoint: use the left endpoint to sort
		//if (a->point[major] == b->point[major]) {
		if (std::abs(a->point[major] - b->point[major]) < 1e-4) {
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
