#pragma once

#include "bezier.hpp"
#include "martinez_impl.hpp"

#include <compare>
#include <memory>
#include <queue>

struct CPUQuadraticShape;

namespace BezierMartinez {

using real_t = double;
using Martinez::ResultTransition;
using Martinez::EdgeType;

struct SweepEvent {
	glm::vec<2, real_t> point;
	real_t t;
	QuadraticBezier<real_t> bezier;
	SweepEvent* otherEvent{nullptr};
	uint32_t contourID{};
	// Is the left endpoint?
	bool left{};
	// Belongs to the source or clipping polygon
	bool isSubject{};

	EdgeType edgeType{EdgeType::NORMAL};
	bool inOut{false};
	bool otherInOut{false};

	SweepEvent* prevInResult{nullptr};
	ResultTransition resultTransition{ResultTransition::NOT_IN_RESULT};

	bool is_below(const glm::vec2& p) const;
	bool is_above(const glm::vec2& p) const;

	bool is_vertical() const;
};

struct CompareEvents {
	std::strong_ordering operator()(const SweepEvent* a, const SweepEvent* b) const;
};

struct CompareEventsGreater {
	bool operator()(const SweepEvent* a, const SweepEvent* b) const;
};

struct CompareSegments {
	std::strong_ordering operator()(const SweepEvent* a, const SweepEvent* b) const;
};

struct CompareSegmentsLess {
	bool operator()(const SweepEvent* a, const SweepEvent* b) const;
};

using EventQueue = std::priority_queue<SweepEvent*, std::vector<SweepEvent*>, CompareEventsGreater>;

void fill_queue(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const CPUQuadraticShape& subject, const CPUQuadraticShape& clipping, BooleanOperation operation);
void subdivide_segments(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		std::vector<SweepEvent*>& sortedEvents, const CPUQuadraticShape& subject,
		const CPUQuadraticShape& clipping, BooleanOperation operation);

void compute_fields(SweepEvent& event, SweepEvent* prev, BooleanOperation operation);

}

void martinez_boolean_bezier(const CPUQuadraticShape& subject, const CPUQuadraticShape& clipping,
		CPUQuadraticShape& result, BooleanOperation operation);
