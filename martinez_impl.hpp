#pragma once

#include "martinez.hpp"

#include <compare>
#include <memory>
#include <queue>

namespace Martinez {

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

	bool operator==(const AABB& other) const;

	bool intersects(const AABB& other) const;
};

struct SweepEvent {
	glm::vec2 point;
	SweepEvent* otherEvent{nullptr};
	uint32_t contourID{};
	// Edge contribution type
	EdgeType edgeType{EdgeType::NORMAL};
	// Is the left endpoint?
	bool left{};
	// Belongs to the source or clipping polygon
	bool isSubject{};
	// FIXME: Potentially unused?
	bool isExteriorRing{};

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

struct Contour {
	std::vector<glm::vec2> points;
	std::vector<uint32_t> holeIDs;
	uint32_t holeOf{~0u};
	uint32_t depth{};

	bool is_exterior() const;
};

void fill_queue(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		const Polygon& subject, const Polygon& clipping, AABB& sbbox, AABB& cbbox, BooleanOperation operation);
void subdivide_segments(EventQueue& queue, std::vector<std::unique_ptr<SweepEvent>>& eventOwner,
		std::vector<SweepEvent*>& sortedEvents, const Polygon& subject, const Polygon& clipping,
		const AABB& sbbox, const AABB& cbbox, BooleanOperation operation);
void connect_edges(std::vector<SweepEvent*>& sortedEvents, std::vector<Contour>& contours,
		BooleanOperation operation);
void order_events(const std::vector<SweepEvent*>& sortedEvents, std::vector<SweepEvent*>& resultEvents);
size_t next_pos(size_t pos, const std::vector<SweepEvent*>& resultEvents, const std::vector<bool>& processed,
		size_t origPos);

float signed_area(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2);

}
