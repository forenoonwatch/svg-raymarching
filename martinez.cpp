#include "martinez.hpp"

#include <queue>
#include <set>
#include <memory>
#include <utility>

#include <cfloat>
#include <cmath>
#include <cstdio>

#include <glm/geometric.hpp>

namespace {

enum class EdgeType {
	NORMAL,
	NON_CONTRIBUTING,
	SAME_TRANSITION,
	DIFFERENT_TRANSITION,
};

enum class ResultTransition {
	NOT_IN_RESULT = 0,
	OUT_IN = 1,
	IN_OUT = -1
};

struct AABB {
	float minX;
	float minY;
	float maxX;
	float maxY;

	bool intersects(const AABB& other) const;
};

struct SweepEvent {
	glm::vec2 point;
	SweepEvent* otherEvent;
	uint32_t contourID;
	// Edge contribution type
	EdgeType edgeType;
	// Is the left endpoint?
	bool left;
	// Belongs to the source or clipping polygon
	bool isSubject;
	// FIXME: Potentially unused?
	bool isExteriorRing;

	bool inOut{false};
	bool otherInOut{false};

	SweepEvent* prevInResult{nullptr};
	ResultTransition resultTransition{ResultTransition::NOT_IN_RESULT};

	size_t otherPos;
	uint32_t outputContourID;

	bool is_below(const glm::vec2& p) const;
	bool is_above(const glm::vec2& p) const;

	bool is_vertical() const;
	bool is_in_result() const;
};

struct CompareEvents {
	bool operator()(const SweepEvent* a, const SweepEvent* b) const;
};

struct CompareSegments {
	bool operator()(const SweepEvent* a, const SweepEvent* b) const;
};

using EventQueue = std::priority_queue<SweepEvent*, std::vector<SweepEvent*>, CompareEvents>;

struct Contour {
	std::vector<glm::vec2> points;
	std::vector<uint32_t> holeIDs;
	uint32_t holeOf{~0u};
	uint32_t depth{};

	bool is_exterior() const;
};

}

static constexpr const AABB AABB_INVALID = {FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};

static void fill_queue(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const Polygon& subject, const Polygon& clipping, AABB& sbbox, AABB& cbbox, BooleanOperation operation);
static void process_polygon(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const PolygonSubShape& contourOrHole, bool isSubject, AABB& bbox, bool isExteriorRing,
		uint32_t contourID);

static void subdivide_segments(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		std::vector<SweepEvent*>& sortedEvents, const Polygon& subject, const Polygon& clipping,
		const AABB& sbbox, const AABB& cbbox, BooleanOperation operation);

static void compute_fields(SweepEvent& event, SweepEvent* prev, BooleanOperation operation);
static int possible_intersection(SweepEvent& a, SweepEvent& b, EventQueue& queue,
		std::vector<std::unique_ptr<SweepEvent>>& eventOwner);
static void divide_segment(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		SweepEvent& event, const glm::vec2& p);

static void connect_edges(std::vector<SweepEvent*>& sortedEvents, std::vector<Contour>& contours,
		BooleanOperation operation);
static void order_events(const std::vector<SweepEvent*>& sortedEvents, std::vector<SweepEvent*>& resultEvents);
static Contour& initialize_contour_from_context(std::vector<Contour>& contours, const SweepEvent& event,
		uint32_t contourID);
static size_t next_pos(size_t pos, const std::vector<SweepEvent*>& resultEvents,
		const std::vector<bool>& processed, size_t origPos);

static float cross(const glm::vec2& a, const glm::vec2& b);

static void deep_copy_shape(Polygon& dst, const Polygon& src);

void martinez_boolean(const Polygon& subject, const Polygon& clipping, Polygon& result,
		BooleanOperation operation) {
	// Check trivial results: when at least one shape is empty
	if (subject.subShapes.empty() || clipping.subShapes.empty()) {
		switch (operation) {
			case BooleanOperation::INTERSECTION:
				return;
			case BooleanOperation::DIFFERENCE:
				deep_copy_shape(result, subject);
				return;
			case BooleanOperation::UNION:
			case BooleanOperation::XOR:
				deep_copy_shape(result, subject.subShapes.empty() ? clipping : subject);
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

	// Convert contours to polygons
	for (auto& contour : contours) {
		// FIXME: not accounting for this being a polygon set
		if (contour.is_exterior()) {
			// The exterior ring goes first
			result.subShapes.emplace_back(contour.points);

			// Followed by holes, if any
			for (auto holeID : contour.holeIDs) {
				result.subShapes.emplace_back(contours[holeID].points);
			}
		}
	}
}

static void fill_queue(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const Polygon& subject, const Polygon& clipping, AABB& sbbox, AABB& cbbox, BooleanOperation operation) {
	uint32_t contourID = 0;

	for (size_t i = 0; i < subject.subShapes.size(); ++i) {
		bool isExteriorRing = i == 0;

		if (isExteriorRing) {
			++contourID;
		}

		process_polygon(queue, eventOwner, subject.subShapes[i], true, sbbox, isExteriorRing, contourID);
	}

	for (size_t i = 0; i < clipping.subShapes.size(); ++i) {
		bool isExteriorRing = i == 0;

		if (isExteriorRing) {
			++contourID;
		}

		process_polygon(queue, eventOwner, clipping.subShapes[i], false, cbbox, isExteriorRing, contourID);
	}
}

static void process_polygon(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const PolygonSubShape& contourOrHole, bool isSubject, AABB& bbox, bool isExteriorRing,
		uint32_t contourID) {
	for (size_t i = 0; i < contourOrHole.points.size() - 1; ++i) {
		auto& s1 = contourOrHole.points[i];
		auto& s2 = contourOrHole.points[i + 1];
		auto e1 = std::make_unique<SweepEvent>(s1, nullptr, contourID, EdgeType::NORMAL, false, isSubject,
				isExteriorRing);
		auto e2 = std::make_unique<SweepEvent>(s2, e1.get(), contourID, EdgeType::NORMAL, false, isSubject,
				isExteriorRing);
		e1->otherEvent = e2.get();

		// Skip collapsed edges or it breaks
		if (s1.x == s2.x && s1.y == s2.y) {
			continue;
		}

		if (CompareEvents{}(e1.get(), e2.get())) {
			e1->left = true;
		}
		else {
			e2->left = true;
		}

		bbox.minX = std::fminf(bbox.minX, s1.x);
		bbox.minY = std::fminf(bbox.minY, s1.y);
		bbox.maxX = std::fmaxf(bbox.maxX, s1.x);
		bbox.maxY = std::fmaxf(bbox.maxY, s1.y);

		// Pushing it so the queue is sorted from left to right, with the object on the left having the
		// highest priority
		queue.push(e1.get());
		queue.push(e2.get());
		eventOwner.emplace_back(std::move(e1));
		eventOwner.emplace_back(std::move(e2));
	}
}

static void subdivide_segments(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		std::vector<SweepEvent*>& sortedEvents, const Polygon& subject, const Polygon& clipping,
		const AABB& sbbox, const AABB& cbbox, BooleanOperation operation) {
	using Node = std::multiset<SweepEvent*, CompareSegments>::iterator;

	auto rightBound = std::fminf(sbbox.maxX, cbbox.maxX);

	std::multiset<SweepEvent*, CompareSegments> sweepLine;

	Node next, prev, begin;

	while (!queue.empty()) {
		auto* event = queue.top();
		queue.pop();

		sortedEvents.emplace_back(event);

		if ((operation == BooleanOperation::INTERSECTION && event->point.x > rightBound)
				|| (operation == BooleanOperation::DIFFERENCE && event->point.x > sbbox.minX)) {
			break;
		}

		if (event->left) {
			next = prev = sweepLine.insert(event);
			begin = sweepLine.begin();
			bool hasPrev = prev != begin;

			if (hasPrev) {
				--prev;
			}

			++next;

			auto* prevEvent = hasPrev ? *prev : nullptr;
			compute_fields(*event, prevEvent, operation);

			if (next != sweepLine.end()) {
				if (possible_intersection(*event, **next, queue, eventOwner) == 2) {
					compute_fields(*event, prevEvent, operation);
					compute_fields(**next, event, operation);
				}
			}

			if (hasPrev) {
				if (possible_intersection(*prevEvent, *event, queue, eventOwner) == 2) {
					auto prevprev = prev;

					if (prevprev != begin) {
						--prevprev;
						compute_fields(*prevEvent, *prevprev, operation);
					}
					else {
						compute_fields(*prevEvent, nullptr, operation);
					}

					compute_fields(*event, prevEvent, operation);
				}
			}
		}
		else {
			event = event->otherEvent;
			auto curr = sweepLine.find(event);
			next = prev = curr;

			if (next != sweepLine.end()) {
				++next;

				if (prev != begin && next != sweepLine.end()) {
					--prev;
					possible_intersection(**prev, **next, queue, eventOwner);
				}

				sweepLine.erase(curr);
			}
		}
	}
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

static void compute_fields(SweepEvent& event, SweepEvent* prev, BooleanOperation operation) {
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

static int possible_intersection(SweepEvent& a, SweepEvent& b, EventQueue& queue,
		std::vector<std::unique_ptr<SweepEvent>>& eventOwner) {
	auto va = a.otherEvent->point - a.point;
	auto vb = b.otherEvent->point - b.point;
	auto e = b.point - a.point;

	if (auto cr = cross(va, vb); cr * cr > 0.f) {
		auto s = cross(e, vb) / cr;

		// Not on line segment A
		if (s < 0.f || s > 1.f) {
			return 0;
		}

		auto t = cross(e, va) / cr;

		// Not on line segment B
		if (t < 0.f || t > 1.f) {
			return 0;
		}

		// The line segments intersect at an end point of both line segments. For the purposes of this
		// algorithm it is discarded
		if ((s == 0.f || s == 1.f) && (t == 0.f || t == 1.f)) {
			return 0;
		}

		// On an endpoint of line segment A
		if (s == 0.f || s == 1.f) {
			divide_segment(queue, eventOwner, b, b.point + vb * t);
		}
		// On an endpoint of line segment B
		else if (t == 0.f || t == 1.f) {
			divide_segment(queue, eventOwner, a, a.point + va * s);
		}
		else {
			divide_segment(queue, eventOwner, a, a.point + va * s);
			divide_segment(queue, eventOwner, b, b.point + vb * t);
		}

		return 1;
	}

	// Lines are either parallel or the same, but the segments could overlap partially, fully, or not at all.
	// If (b.point - a.point) is parallel with the line, then the two lines are the same and there will be
	// overlap.

	// Lines are parallel, no overlap
	if (auto cr = cross(e, va); cr * cr > 0.f) {
		return 0;
	}

	auto sqrLenA = glm::dot(va, va);
	auto sa = glm::dot(va, e) / sqrLenA;
	auto sb = sa + glm::dot(va, vb) / sqrLenA;
	auto sMin = std::fminf(sa, sb);
	auto sMax = std::fmaxf(sa, sb);

	if (sMin <= 1.f && sMax >= 0.f) {
		// Overlap on an end point
		if (sMin == 1.f) {
			return 0;
		}

		// Overlap on an end point
		if (sMax == 0.f) {
			return 0;
		}

		if (sMin == 0.f && sMax == 1.f) {
			return 0;
		}

		// There's an overlap on a segment - 2 points of intersection
		SweepEvent* events[4]{};
		SweepEvent** ppEvents = events;
		bool leftCoincide = false;
		bool rightCoincide = false;

		if (a.point == b.point) {
			leftCoincide = true; // linked
		}
		else if (!CompareEvents{}(&a, &b)) {
			*(ppEvents++) = &b;
			*(ppEvents++) = &a;
		}
		else {
			*(ppEvents++) = &a;
			*(ppEvents++) = &b;
		}

		if (a.otherEvent->point == b.otherEvent->point) {
			rightCoincide = true;
		}
		else if (!CompareEvents{}(a.otherEvent, b.otherEvent)) {
			*(ppEvents++) = b.otherEvent;
			*(ppEvents++) = a.otherEvent;
		}
		else {
			*(ppEvents++) = a.otherEvent;
			*(ppEvents++) = b.otherEvent;
		}

		// Both line segments are equal or share the left endpoint
		if ((leftCoincide && rightCoincide) || leftCoincide) {
			b.edgeType = EdgeType::NON_CONTRIBUTING;
			a.edgeType = (b.inOut == a.inOut) ? EdgeType::SAME_TRANSITION : EdgeType::DIFFERENT_TRANSITION;

			// Honestly no idea, but changing events sselection from [2, 1] to [0, 1] fixes the overlapping
			// self-intersecting polygons issue
			if (leftCoincide && !rightCoincide) {
				divide_segment(queue, eventOwner, *events[1]->otherEvent, events[0]->point);
			}

			return 2;
		}

		// The line segments share the right endpoint
		if (rightCoincide) {
			divide_segment(queue, eventOwner, *events[0], events[1]->point);
			return 3;
		}

		// No line segment fully includes the other one
		if (events[0] != events[3]->otherEvent) {
			divide_segment(queue, eventOwner, *events[0], events[1]->point);
			divide_segment(queue, eventOwner, *events[1], events[2]->point);
			return 3;
		}

		// One line segment includes the other one
		divide_segment(queue, eventOwner, *events[0], events[1]->point);
		divide_segment(queue, eventOwner, *events[3]->otherEvent, events[2]->point);
		return 3;
	}

	return 0;
}

static void divide_segment(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		SweepEvent& event, const glm::vec2& p) {
	auto r = std::make_unique<SweepEvent>(p, &event, event.contourID, EdgeType::NORMAL, false, event.isSubject);
	auto l = std::make_unique<SweepEvent>(p, event.otherEvent, event.contourID, EdgeType::NORMAL, true,
			event.isSubject);

	if (event.point == event.otherEvent->point) {
		printf("Warning: maybe collapsed segment? %u\n", event.contourID);
	}

	// Avoid a rounding error. The left event would be processed after the right event
	if (!CompareEvents{}(l.get(), event.otherEvent)) {
		event.otherEvent->left = true;
		l->left = false;
	}

	event.otherEvent->otherEvent = l.get();
	event.otherEvent = r.get();

	queue.push(l.get());
	queue.push(r.get());

	eventOwner.emplace_back(std::move(l));
	eventOwner.emplace_back(std::move(r));
}

static void connect_edges(std::vector<SweepEvent*>& sortedEvents, std::vector<Contour>& contours,
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

			if (pos < resultEvents.size() && resultEvents[pos]) {
				resultEvents[pos]->outputContourID = contourID;
			}

			pos = resultEvents[pos]->otherPos;

			processed[pos] = true;

			if (pos < resultEvents.size() && resultEvents[pos]) {
				resultEvents[pos]->outputContourID = contourID;
			}

			contour.points.emplace_back(resultEvents[pos]->point);

			pos = next_pos(pos, resultEvents, processed, origPos);

			if (pos == origPos || pos >= resultEvents.size() || !resultEvents[pos]) {
				break;
			}
		}
	}
}

static void order_events(const std::vector<SweepEvent*>& sortedEvents, std::vector<SweepEvent*>& resultEvents) {
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
			// FIXME: compare() == 1, improve comparison
			if ((i + 1) < resultEvents.size() && !CompareEvents{}(resultEvents[i], resultEvents[i + 1])) {
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

static size_t next_pos(size_t pos, const std::vector<SweepEvent*>& resultEvents,
		const std::vector<bool>& processed, size_t origPos) {
	auto newPos = pos + 1;
	auto p = resultEvents[pos]->point;

	if (newPos < resultEvents.size()) {
		auto p1 = resultEvents[newPos]->point;

		while (newPos < resultEvents.size() && p == p1) {
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

static float cross(const glm::vec2& a, const glm::vec2& b) {
	return a.x * b.y - a.y * b.x;
}

static void deep_copy_shape(Polygon& dst, const Polygon& src) {
	for (auto& subshape : src.subShapes) {
		dst.subShapes.emplace_back();
		auto& dstSubshape = dst.subShapes.back();
		dstSubshape.points = subshape.points;
	}
}

static float signed_area(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2) {
	return (p0.x - p2.x) * (p1.y - p2.y) - (p1.x - p2.x) * (p0.y - p2.y);
}

bool AABB::intersects(const AABB& other) const {
	return maxX >= other.minX && maxY >= other.minY && minX <= other.maxX && minY <= other.maxY;
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
	return point.x == otherEvent->point.x;
}

bool SweepEvent::is_in_result() const {
	return resultTransition != ResultTransition::NOT_IN_RESULT;
}

bool CompareEvents::operator()(const SweepEvent* a, const SweepEvent* b) const {
	// Different x coordinate
	if (a->point.x > b->point.x) {
		return false;
	}
	else if (a->point.x < b->point.x) {
		return true;
	}

	// Different points, but same x coordinate
	if (a->point.y > b->point.y) {
		return false;
	}
	else if (a->point.y < b->point.y) {
		return true;
	}

	// Special cases:
	// 1. same coordinates, but one is a left endpoint and the other is a right endpoint. The right
	// endpoint is processed first
	if (a->left != b->left) {
		return a->left;
	}

	// Same coordinates, both events are left endpoints or right endpoints, but not collinear
	if (signed_area(a->point, a->otherEvent->point, b->otherEvent->point) != 0) {
		// The event associated with the bottom segment is processed first
		return a->is_below(b->otherEvent->point);
	}

	return a->isSubject || !b->isSubject;
}

bool CompareSegments::operator()(const SweepEvent* a, const SweepEvent* b) const {
	if (a == b) {
		return false;
	}

	// Segments are not collinear
	if (signed_area(a->point, a->otherEvent->point, b->point) != 0
			|| signed_area(a->point, a->otherEvent->point, b->otherEvent->point) != 0) {
		// If they share their left endpoint use the right endpoint to sort
		if (a->point == b->point) {
			return a->is_below(b->otherEvent->point);
		}

		// Different left endpoint: use the left endpoint to sort
		if (a->point.x == b->point.x) {
			return a->point.y < b->point.y;
		}

		// Has the line segment associated with a been inserted into S after the line segment associated
		// with b?
		if (!CompareEvents{}(a, b)) {
			return b->is_above(a->point);
		}

		// The line segment associated with b has been inserted into S after the line segment associated
		// with a
		return a->is_below(b->point);
	}

	// Same polygon
	if (a->isSubject == b->isSubject) {
		if (a->point == b->point) {
			if (a->otherEvent->point == b->otherEvent->point) {
				return false;
			}

			return a->contourID < b->contourID;
		}
	}
	// Segments are collinear but belong to separate polygons
	else {
		return a->isSubject;
	}

	return CompareEvents{}(a, b);
}

bool Contour::is_exterior() const {
	return holeOf == ~0u;
}
