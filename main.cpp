#include <cstdio>
#include <cstring>
#include <cfloat>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <optional>
#include <string>

#include "svg.hpp"

#define NANOVG_GL3_IMPLEMENTATION
#include <nanovg.h>
#include <nanovg_gl.h>

#define NANOSVG_ALL_COLOR_KEYWORDS
#define NANOSVG_IMPLEMENTATION
#include <nanosvg.h>

#include "shader.hpp"
#include "svg_shader.hpp"
#include "hatch_shader.hpp"

#include "hatch.hpp"
#include "martinez.hpp"

#include <algorithm>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <glm/geometric.hpp>
#include "hatch_impl.hpp"
#include "martinez_impl.hpp"
#include "bezier_martinez_impl.hpp"
#include "pair.hpp"

#include <glm/mat2x2.hpp>

static constexpr const char* g_lineVertex = R"(
#version 460

layout (location = 0) uniform vec2 u_p0;
layout (location = 1) uniform vec2 u_p1;
layout (location = 2) uniform vec2 u_invScreenSize;

void main() {
	vec2 p = (gl_VertexID == 0 ? u_p0 : u_p1) * u_invScreenSize;
	gl_Position = vec4(vec2(p.x, 1 - p.y) * 2 - 1, 0, 1);
}
)";

static constexpr const char* g_lineFragment = R"(
#version 460

layout (location = 3) uniform vec4 u_color;

layout (location = 0) out vec4 o_color;

void main() {
	o_color = u_color;
}
)";

static void GLAPIENTRY gl_message_callback(GLenum source, GLenum type, GLuint id, GLenum severity,
		GLsizei length, const GLchar* message, const void* userParam);

static std::optional<std::string> file_read_bytes(const char* fileName);

static constexpr const int g_width = 640;
static constexpr const int g_height = 480;

struct GPUPathData {
	glm::vec2 points[65536];
	uint32_t pointCounts[1024];
	uint32_t pathCounts[1024];
	uint32_t fillColors[1024];
	uint32_t strokeColors[1024];
	float strokeWidths[1024];
	uint32_t pathFlags[1024];
};

static void convert_cpu_path_data_to_gpu(const std::vector<CPUQuadraticShape>& shapes, GPUPathData& pathData,
		uint32_t& shaderShapeCount);

static void nvg_draw(NVGcontext* vg, NSVGimage* nsvg);
static void debug_draw_lines(NVGcontext* vg, GPUPathData& shaderPathData, uint32_t shaderShapeCount);

static void draw_polygon(NVGcontext* vg, const Polygon& polygon, const glm::vec2& offset,
		NVGcolor color);

static void add_polygon_svg(std::vector<CPUQuadraticShape>& svgShapes, const Polygon& polygon,
		uint32_t fillColor, const glm::vec2& polygonOffset) {
	svgShapes.emplace_back();
	auto& shape = svgShapes.back();
	shape.fillColor = fillColor;

	for (auto& subshape : polygon.subShapes) {
		shape.paths.emplace_back();
		auto& path = shape.paths.back();

		path.points.emplace_back(subshape.points[0] + polygonOffset);

		for (size_t i = 1; i < subshape.points.size(); ++i) {
			path.points.emplace_back((subshape.points[i - 1] + subshape.points[i]) * 0.5f + polygonOffset);
			//path.points.emplace_back(subshape.points[i] + polygonOffset);
			path.points.emplace_back(subshape.points[i] + polygonOffset);
		}
	}
}

static glm::vec<2, double> controlPoints[] = {
	{50, 100}, {300, 150}, {50, 200},
	{100, 50}, {200, 100}, {150, 350},
};
static glm::vec<2, double>* selectedPoint = nullptr;

static void on_mouse_button_event(GLFWwindow* window, int button, int action, int mods);
static void on_mouse_move_event(GLFWwindow* window, double xPos, double yPos);
static void on_key_event(GLFWwindow* window, int key, int scancode, int action, int mods);

int g_step = 0;

struct SegmentComp {
	bool operator()(const BezierMartinez::SweepEvent* a, const BezierMartinez::SweepEvent* b) const {
		Hatch::Segment segA{.originalBezier = &a->bezier, .tStart = a->t, .tEnd = a->otherEvent->t};
		Hatch::Segment segB{.originalBezier = &b->bezier, .tStart = b->t, .tEnd = b->otherEvent->t};

		return Hatch::SegmentLess{}(segA, segB);
	}
};

static Polygon quad_shape_to_polygon(const CPUQuadraticShape& shape) {
	Polygon poly;

	for (auto& path : shape.paths) {
		poly.subShapes.emplace_back();
		auto& subshape = poly.subShapes.back();

		for (size_t i = 0; i < path.points.size() - 1; i += 2) {
			QuadraticBezier<Hatch::real_t> bezier{path.points[i], path.points[i + 1], path.points[i + 2]};
			auto quadratic = QuadraticCurve<double>::from_bezier(bezier.P0, bezier.P1, bezier.P2);
			auto t = -quadratic.B / (2.0 * quadratic.A);

			if (i == 0) {
				subshape.points.emplace_back(path.points[i]);
			}

			if (t.x > 0 && t.x < 1.0 && (t.x == t.y || t.y <= 0 || t.y > 1.0)) {
				subshape.points.emplace_back(bezier.evaluate(t.x));
			}

			if (t.y > 0 && t.y < 1.0 && t.x != t.y) {
				subshape.points.emplace_back(bezier.evaluate(t.y));
			}

			subshape.points.emplace_back(path.points[i + 2]);
		}
	}

	return poly;
}

struct MartinezDebug {
	std::vector<std::unique_ptr<Martinez::SweepEvent>> ownedEvents;
	Martinez::EventQueue queue;
	Martinez::AABB sbbox{FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};
	Martinez::AABB cbbox{FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};
	std::vector<Martinez::SweepEvent*> sortedEvents;

	void build(const Polygon& subj, const Polygon& clip, BooleanOperation op) {
		Martinez::fill_queue(queue, ownedEvents, subj, clip, sbbox, cbbox, op);
		Martinez::subdivide_segments(queue, ownedEvents, sortedEvents, subj, clip, sbbox, cbbox, op);
	}

	void draw(NVGcontext* vg) {
		for (auto* event : sortedEvents) {
			if (!event->left) {
				continue;
			}

			bool inResult = event->is_in_result();

			nvgBeginPath(vg);
			nvgMoveTo(vg, event->point.x, event->point.y);
			nvgLineTo(vg, event->otherEvent->point.x, event->otherEvent->point.y);
			nvgStrokeWidth(vg, 2.f);
			nvgStrokeColor(vg, inResult ? nvgRGBA(0, 255, 0, 255) : nvgRGBA(128, 128, 128, 255));
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, event->point.x, event->point.y, 3);
			nvgFillColor(vg, nvgRGBA(255, 0, 0, 255));
			nvgFill(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, event->otherEvent->point.x, event->otherEvent->point.y, 3);
			nvgFillColor(vg, nvgRGBA(0, 0, 255, 255));
			nvgFill(vg);
		}
	}
};

template <typename float_t>
static void bez_bez_fast(const QuadraticBezier<float_t>& bezierA, const QuadraticBezier<float_t>& bezierB);

struct BezierMartinezDebug {
	std::vector<std::unique_ptr<BezierMartinez::SweepEvent>> ownedEvents;
	BezierMartinez::EventQueue queue;
	Martinez::AABB sbbox{FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};
	Martinez::AABB cbbox{FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX};
	std::vector<BezierMartinez::SweepEvent*> sortedEvents;
	std::vector<glm::vec<2, BezierMartinez::real_t>> unfoundIntersections;
	std::vector<glm::vec<2, BezierMartinez::real_t>> fromOrthoUnfound;

	void build(const CPUQuadraticShape& subj, const CPUQuadraticShape& clip, BooleanOperation op) {
		int major = static_cast<int>(Hatch::SELECTED_MAJOR_AXIS);
		int minor = 1 - major;

		BezierMartinez::fill_queue(queue, ownedEvents, subj, clip, sbbox, cbbox, op);
		BezierMartinez::subdivide_segments(queue, ownedEvents, sortedEvents, subj, clip, sbbox, cbbox, op);

		for (auto* event : sortedEvents) {
			if (!event->left) {
				continue;
			}

			assert(event->t < event->otherEvent->t);
		}

		for (size_t i = 0; i < sortedEvents.size(); ++i) {
			if (!sortedEvents[i]->left /*|| sortedEvents[i]->is_in_result()*/) {
				continue;
			}

			Hatch::Segment segA{&sortedEvents[i]->bezier, sortedEvents[i]->t, sortedEvents[i]->otherEvent->t};

			for (size_t j = i + 1; j < sortedEvents.size(); ++j) {
				if (!sortedEvents[j]->left /*|| sortedEvents[j]->is_in_result()*/) {
					continue;
				}

				Hatch::Segment segB{&sortedEvents[j]->bezier, sortedEvents[j]->t,
						sortedEvents[j]->otherEvent->t};

				auto sects = segA.intersect(segB);

				bez_bez_fast(*segA.originalBezier, *segB.originalBezier);

				for (auto t : sects) {
					if (std::isnan(t) || t <= segB.tStart || t >= segB.tEnd) {
						continue;
					}

					auto point = segB.originalBezier->evaluate(t);
					auto s = Hatch::intersect_ortho(*segA.originalBezier, point[major], major);

					if (!std::isnan(s) && s > segA.tStart && s < segA.tEnd) {
						auto ptA = segA.originalBezier->evaluate(s);
						unfoundIntersections.emplace_back(point);
						fromOrthoUnfound.emplace_back(ptA);
					}
				}
			}
		}
	}

	void draw(NVGcontext* vg) {
		for (auto* event : sortedEvents) {
			if (!event->left) {
				continue;
			}

			Hatch::Segment seg{&event->bezier, event->t, event->otherEvent->t};

			bool inResult = event->is_in_result();
			bool linear = Hatch::is_line_segment(event->bezier);
			bool slcm = seg.is_straight_line_constant_major();

			auto bezier = event->bezier.split_from_min_to_max(event->t, event->otherEvent->t);

			nvgBeginPath(vg);
			nvgMoveTo(vg, event->point.x, event->point.y);
			//nvgLineTo(vg, event->otherEvent->point.x, event->otherEvent->point.y);
			//nvgQuadTo(vg, bezier.P1.x, bezier.P1.y, bezier.P2.x, bezier.P2.y);
			nvgQuadTo(vg, bezier.P1.x, bezier.P1.y, event->otherEvent->point.x, event->otherEvent->point.y);
			nvgStrokeWidth(vg, 2.f);
			nvgStrokeColor(vg, inResult ? nvgRGBA(0, 255, 0, 255) : nvgRGBA(128, 128, 128, 255));
			//nvgStrokeColor(vg, linear ? nvgRGBA(255, 255, 0, 255) : nvgRGBA(0, 0, 255, 255));
			
			auto gradGreen = nvgLinearGradient(vg, event->point.x, event->point.y, event->otherEvent->point.x,
					event->otherEvent->point.y, nvgRGBA(0, 64, 0, 255), nvgRGBA(0, 255, 0, 255));
			auto gradGrey = nvgLinearGradient(vg, event->point.x, event->point.y, event->otherEvent->point.x,
					event->otherEvent->point.y, nvgRGBA(64, 64, 64, 255), nvgRGBA(128, 128, 128, 255));

			nvgStrokePaint(vg, inResult ? gradGreen : gradGrey);
			nvgStroke(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, event->point.x, event->point.y, 3);
			nvgFillColor(vg, nvgRGBA(255, 0, 0, 255));
			//nvgFill(vg);

			nvgBeginPath(vg);
			nvgCircle(vg, event->otherEvent->point.x, event->otherEvent->point.y, 3);
			nvgFillColor(vg, nvgRGBA(0, 0, 255, 255));
			//nvgFill(vg);
		}

		for (auto pt : fromOrthoUnfound) {
			nvgBeginPath(vg);
			nvgCircle(vg, pt.x, pt.y, 2);
			nvgFillColor(vg, nvgRGBA(255, 255, 0, 255));
			nvgFill(vg);
		}

		for (auto pt : unfoundIntersections) {
			nvgBeginPath(vg);
			nvgCircle(vg, pt.x, pt.y, 2);
			nvgFillColor(vg, nvgRGBA(0, 255, 255, 255));
			nvgFill(vg);
		}
	}
};

namespace {

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
			Pair<float_t, float_t> inFRange, Pair<float_t, float_t> inGRange)
		: F(inF)
		, G(inG)
		, fRange(inFRange)
		, gRange(inGRange) {}

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

}

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
		const glm::vec<2, float_t>& L0, const glm::vec<2, float_t>& L1, float_t lineConstant) {
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

	//if (roots.x >= float_t(0.0) && roots.x <= float_t(1.0)) {
		result[resultCount++] = roots.x;
	//}

	//if (roots.y >= float_t(0.0) && roots.y <= float_t(1.0)) {
		result[resultCount++] = roots.y;
	//}

	return result;
}

template <typename float_t>
static int check_intersection_in_ranges(const Iteration<float_t>& iter,
		std::array<Iteration<float_t>, 2>& newIters) {
	auto& F = *iter.F;
	auto& G = *iter.G;
	auto& fRange = iter.fRange;
	auto& gRange = iter.gRange;

	auto ftMin = fRange.first;
	auto ftMax = fRange.second;
	auto gtMin = gRange.first;
	auto gtMax = gRange.second;

	//auto F_ = from_to_including_error_bound(F, ftMin, ftMax);
	//auto G_ = from_to_including_error_bound(G, gtMin, gtMax);
	auto F_ = F.split_from_min_to_max(ftMin, ftMax);
	auto G_ = G.split_from_min_to_max(gtMin, gtMax);

	// Q will be fat line bounded. Get start and endpoint of curve
	auto& FS = F_.P0;
	auto& FE = F_.P2;

	auto dF0 = distance_to_line(FS, FE, F_.P0);
	auto dF1 = distance_to_line(FS, FE, F_.P1);
	auto dF2 = distance_to_line(FS, FE, F_.P2);

	return 0;
}

template <typename float_t>
static void bez_bez_fast(const QuadraticBezier<float_t>& bezierA, const QuadraticBezier<float_t>& bezierB) {
	const float_t tMinimumAccuracy = std::pow(2.0, -33);
	const float_t finalTMinimumAccuracy = std::pow(2.0, -43);

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
		int results = check_intersection_in_ranges(iter, newIters);

		if (results == 1) {
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
						fRange.first = std::max(0.0, fRange.first - finalTMinimumAccuracy);
						fRange.second = std::min(1.0, fRange.second + finalTMinimumAccuracy);
					}

					newIter.last = std::make_unique<Iteration<float_t>>(newIter.F, newIter.G, newIter.fRange,
							newIter.gRange);
				}

				// Push the (possibly) final iteration
				stack.emplace_back(std::move(newIter));
			}
		}
		else if (results == 2) {
			stack.emplace_back(std::move(newIters[0]));
			stack.emplace_back(std::move(newIters[1]));
		}
	}
	
	// Check for possible duplicate intersections at split points
	std::sort(xs.begin(), xs.end(), [](const auto& a, const auto& b) {
		return a.first.first < b.first.first; 
	});
}

template <typename float_t>
static int clip_curve_by_fat_line(const QuadraticBezier<float_t>& bezier, const glm::vec<2, float_t>& L0,
		const glm::vec<2, float_t>& L1, float_t lineOffset, const Pair<float_t, float_t>& gRange,
		std::array<Pair<float_t, float_t>, 2>& results) {
	auto resLow = intersect_bez_line(bezier, L0, L1, float_t(0.0));
	auto resHigh = intersect_bez_line(bezier, L0, L1, -lineOffset);

	bool hasRootsLo = !(std::isnan(resLow[0]) && std::isnan(resLow[1]));
	bool hasRootsHi = !(std::isnan(resHigh[0]) && std::isnan(resHigh[1]));

	if (!hasRootsLo && !hasRootsHi) {
		// If neither the high or low bound of the fat line intersect the other curve,
		// then they cannot intersect
		return 0;
	}

	if (hasRootsLo && !hasRootsHi) {
		// 1 solution means the line F.P0->F.P2 tangentially touches G. This either means no intersection,
		// an exact intersection of F.P0/F.P2 with G, or F is a line segment and touches G
		if (std::isnan(resLow[1]) || resLow[0] == resLow[1]) {
			// Intersection is outside of the range of G, so no intersections
			if (resLow[0] < gRange.first || resLow[0] > gRange.second) {
				return 0;
			}

			// FIXME: Check if G(resLow[0]) lies on F.
		}

		auto tMin = std::min(gRange.second, std::max(gRange.first, resLow[0]));
		auto tMax = std::min(gRange.second, std::max(gRange.first, resLow[1]));

		// Return clipped result with swapped parameters.
		results[0] = {tMin, tMax};
		return 1;
	}
	else if (hasRootsHi && !hasRootsLo) {
		// 1 solution means that the line F.P0->F.P2 + offset tangentially touches G. This can only mean no
		// intersection, since if F was a line, it would have found low roots.
		if (std::isnan(resHigh[1]) || resHigh[0] == resHigh[1]) {
			return 0;
		}

		auto tMin = std::min(gRange.second, std::max(gRange.first, resHigh[0]));
		auto tMax = std::min(gRange.second, std::max(gRange.first, resHigh[1]));

		// Return clipped result with swapped parameters.
		results[0] = {tMin, tMax};
		return 1;
	}
	else {
		auto tMin0 = std::min(gRange.second, std::max(gRange.first, resLow[0]));
		auto tMax0 = std::min(gRange.second, std::max(gRange.first, resHigh[0]));

		auto tMin1 = std::min(gRange.second, std::max(gRange.first, resLow[1]));
		auto tMax1 = std::min(gRange.second, std::max(gRange.first, resHigh[1]));

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
}

template <typename float_t>
static int check_intersection_in_ranges2(const Iteration<float_t>& iter,
	std::array<Iteration<float_t>, 2>& newIters) {
	static constexpr const float_t maxClipTSpan = float_t(0.7);

	auto subF = iter.F->split_from_min_to_max(iter.fRange.first, iter.fRange.second);
	auto lineOffset = distance_to_line(subF.P0, subF.P2, subF.P1);
	std::array<Pair<float_t, float_t>, 2> intersections;
	int intersectionCount = clip_curve_by_fat_line(*iter.G, subF.P0, subF.P2, lineOffset, iter.gRange,
			intersections);

	if (intersectionCount == 0) {
		return 0;
	}
	else if (intersectionCount == 2) {
		newIters[0] = {iter.G, iter.F, intersections[0], iter.fRange};
		newIters[1] = {iter.G, iter.F, intersections[1], iter.fRange};
		return 2;
	}

	auto [tMin, tMax] = intersections[0];
	auto tScale = float_t(1.0);// / (iter.gRange.second - iter.gRange.first);

	// If the range is too large and a solution hasn't been found yet, try a perpendicular line.
	// This is important for efficiency especially in cases where Bezier curves meet (or almost meet)
	// with nearly the same tangent and curvature.
	if (!iter.last && (tMax - tMin) * tScale > maxClipTSpan) {
		auto perp = subF.P0 + glm::vec<2, float_t>(subF.P0.y - subF.P2.y, subF.P2.x - subF.P0.x);
		// FIXME: Compare and swap by the absolute value of these 2
		lineOffset = std::max(distance_to_line(subF.P0, perp, subF.P1),
				distance_to_line(subF.P0, perp, subF.P2));
		intersectionCount = clip_curve_by_fat_line(*iter.G, subF.P0, perp, lineOffset, iter.gRange,
				intersections);

		if (intersectionCount == 0) {
			return 0;
		}
		else if (intersectionCount == 2) {
			newIters[0] = {iter.G, iter.F, intersections[0], iter.fRange};
			newIters[1] = {iter.G, iter.F, intersections[1], iter.fRange};
			return 2;
		}

		tMin = intersections[0].first;
		tMax = intersections[0].second;
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

	newIters[0] = {iter.G, iter.F, {tMin, tMax}, iter.fRange};
	return 1;
}

template <typename float_t>
struct BezBezDebugEntry {
	std::array<Iteration<float_t>, 2> iters;
	int count;
};

template <typename float_t>
struct BezBezDebug {
	std::vector<BezBezDebugEntry<float_t>> steps;
};

template <typename float_t>
static void bez_bez_fast2(const QuadraticBezier<float_t>& bezierA, const QuadraticBezier<float_t>& bezierB,
		BezBezDebug<float_t>& debug) {
	int iterations = 0;
	int maxIterations = 30;

	std::vector<Iteration<float_t>> stack;
	stack.emplace_back(&bezierA, &bezierB, Pair<float_t, float_t>{0, 1}, Pair<float_t, float_t>{0, 1});

	while (!stack.empty() && iterations < maxIterations) {
		++iterations;

		auto iter = std::move(stack.back());
		stack.pop_back();

		std::array<Iteration<float_t>, 2> newIters;
		int resultCount = check_intersection_in_ranges2(iter, newIters);

		debug.steps.emplace_back(newIters, resultCount);

		if (resultCount == 1) {
			stack.emplace_back(std::move(newIters[0]));
		}
		else if (resultCount == 2) {
			stack.emplace_back(std::move(newIters[0]));
			stack.emplace_back(std::move(newIters[1]));
		}
	}
}

int main() {
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
	auto* window = glfwCreateWindow(g_width, g_height, "Jong", nullptr, nullptr);

	glfwSetMouseButtonCallback(window, on_mouse_button_event);
	glfwSetCursorPosCallback(window, on_mouse_move_event);
	glfwSetKeyCallback(window, on_key_event);

	glfwMakeContextCurrent(window);
	gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress));

	glEnable(GL_DEBUG_OUTPUT);
	glDebugMessageCallback(gl_message_callback, 0);

	auto* vg = nvgCreateGL3(NVG_DEBUG);

	auto svgData = file_read_bytes("../third_party/riichi-mahjong-tiles/Regular/Chun.svg");
	Scene svg;

	if (svgData) {
		svg_parse(svgData->data(), svgData->size(), svg);
	}
	else {
		perror("");
	}

	NSVGimage* nsvg = nsvgParseFromFile("../third_party/riichi-mahjong-tiles/Regular/Ton.svg", "px", 96);

	unsigned vao;
	glCreateVertexArrays(1, &vao);

	auto fstShader = Shader::load(g_svgVertex, g_svgFragment);
	auto hatchShader = Shader::load(g_hatchVertex, g_hatchFragment);

	unsigned curveBuffer;
	glCreateBuffers(1, &curveBuffer);

	GPUPathData shaderPathData;
	uint32_t shaderShapeCount = 0;

	std::vector<CPUQuadraticShape> svgShapes;
	svg_convert_to_quadratic_paths(svg, svgShapes);

	float res[] = {g_width, g_height};

	float counter = 0.f;

	Polygon subject{
		.subShapes = {
			{.points = {{6, 43}, {-1.5f, -198}, {248, -199.5f}, {252.5f, 42.5f}, {6, 43}}},
			{.points = {{57.5f, 1.5f}, {50.5f, -149}, {210, -150.5f}, {213, 3}, {57.5f, 1.5f}}},
		}
	};

	Polygon clipping{
		.subShapes = {
			{.points = {{18.5f, 89.5f}, {90, -141.5f}, {178.5f, -181.5f}, {184, 79.5f}, {18.5f, 89.5f}}},
			{.points = {{112.5f, -108.5f}, {72, 57.5f}, {162, 59.5f}, {158, -132}, {112.5f, -108.5f}}},
		}
	};

	Polygon result{};
	martinez_boolean(subject, clipping, result, BooleanOperation::XOR);

	glm::vec2 polygonOffset{200, 300};

	//add_polygon_svg(svgShapes, subject, 0xFF008000u, polygonOffset);
	//add_polygon_svg(svgShapes, clipping, 0xFF008000u, polygonOffset);
	//add_polygon_svg(svgShapes, result, 0xFF0000FFu, polygonOffset);
	
	CPUQuadraticShape subj{
		.paths = {
			{.points = {
				{100, 100}, {150, 75}, {200, 100}, {225, 150}, {200, 200},
				{150, 225}, {100, 200}, {75, 150}, {100, 100},
			}}
		},
	};

	CPUQuadraticShape clip{
		.paths = {
			{.points = {
				{100 + 25, 100 + 25}, {150 + 25, 75 + 25}, {200 + 25, 100 + 25},
				{225 + 25, 150 + 25}, {200 + 25, 200 + 25},
				{150 + 25, 225 + 25}, {100 + 25, 200 + 25}, {75 + 25, 150 + 25},
				{100 + 25, 100 + 25},
			}}
		},
	};

	/*Polygon polySubj{
		.subShapes = {
			{.points = {{200, 200}, {300, 200}, {300, 250}, {200, 250}, {200, 200}}},
			{.points = {{210, 210}, {290, 210}, {290, 240}, {210, 240}, {210, 210}}},
		},
	};

	Polygon polyClip{
		.subShapes = {
			{.points = {{225, 175}, {275, 175}, {275.001f, 275}, {225, 275}, {225, 175}}},
		},
	};*/

	auto polySubj = quad_shape_to_polygon(svgShapes[0]);
	auto polyClip = quad_shape_to_polygon(svgShapes[1]);
	Polygon polyRes;
	martinez_boolean(polySubj, polyClip, polyRes, BooleanOperation::UNION);

	CPUQuadraticShape resShape;
	martinez_boolean_bezier(subj, clip, resShape, BooleanOperation::XOR);

	std::vector<CurveHatchBox> hatchBoxes;

	//generate_hatch_boxes(resShape, hatchBoxes);

	std::vector<CPUQuadraticShape> unionStack;

	for (auto& shape : svgShapes) {
		unionStack.emplace_back(shape);
	}

	while (unionStack.size() > 1) {
		auto& subj = unionStack.back();
		auto& clip = unionStack[unionStack.size() - 2];

		CPUQuadraticShape result;
		martinez_boolean_bezier(subj, clip, result, BooleanOperation::UNION);

		printf("Union stack size before popping: %llu\n", unionStack.size());

		unionStack.pop_back();
		unionStack.pop_back();

		unionStack.emplace_back(std::move(result));

		break;
	}

	//generate_hatch_boxes(unionStack.back(), hatchBoxes);
	//generate_hatch_boxes(unionStack[unionStack.size() - 2], hatchBoxes);
	//generate_hatch_boxes(unionStack[unionStack.size() - 3], hatchBoxes);

	for (auto& shape : svgShapes) {
		//generate_hatch_boxes(shape, hatchBoxes);
	}

	printf("Generated %llu hatch boxes\n", hatchBoxes.size());

	//convert_cpu_path_data_to_gpu(svgShapes, shaderPathData, shaderShapeCount);
	glNamedBufferStorage(curveBuffer, sizeof(shaderPathData), &shaderPathData, GL_DYNAMIC_STORAGE_BIT);

	auto lastTime = glfwGetTime();
	int step = 0;

	BezierMartinezDebug bmd;
	bmd.build(svgShapes[0], svgShapes[1], BooleanOperation::UNION);

	auto cross = [](const auto& a, const auto& b) {
		return a.x * b.y - a.y * b.x;
	};

	while (!glfwWindowShouldClose(window)) {
		auto currTime = glfwGetTime();
		auto deltaTime = static_cast<float>(currTime - lastTime);
		lastTime = currTime;

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

		glBindVertexArray(vao);

		if (shaderShapeCount > 0) {
			fstShader.bind();

			glUniform1f(0, static_cast<float>(glfwGetTime()));
			glUniform2fv(1, 1, res);
			glUniform1ui(2, shaderShapeCount);

			glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, curveBuffer);

			glDrawArrays(GL_TRIANGLES, 0, 3);
		}

		float invScreenSize[] = {1.f / g_width, 1.f / g_height};

		nvgBeginFrame(vg, g_width, g_height, 1.0);

		bmd.draw(vg);

		//draw_polygon(vg, polyRes, {}, nvgRGBA(0, 255, 0, 255));

		for (auto& box : hatchBoxes) {
			hatchShader.bind();

			float extents[] = {box.aabbMin.x, box.aabbMin.y, box.aabbMax.x - box.aabbMin.x,
					box.aabbMax.y - box.aabbMin.y};
			glUniform2fv(0, 1, invScreenSize);
			glUniform4fv(1, 1, extents);
			glUniform2fv(2, 3, reinterpret_cast<const float*>(&box.curveMin[0]));
			glUniform2fv(5, 3, reinterpret_cast<const float*>(&box.curveMax[0]));
			//glDrawArrays(GL_TRIANGLES, 0, 6);

			/*nvgBeginPath(vg);
			nvgMoveTo(vg, box.aabbMin.x, box.aabbMin.y);
			nvgLineTo(vg, box.aabbMax.x, box.aabbMin.y);
			nvgLineTo(vg, box.aabbMax.x, box.aabbMax.y);
			nvgLineTo(vg, box.aabbMin.x, box.aabbMax.y);
			nvgClosePath(vg);

			nvgFillColor(vg, nvgRGBA(255, 0, 0, 128));
			nvgFill(vg);*/

			auto p0 = box.curveMin[0] * (box.aabbMax - box.aabbMin) + box.aabbMin;
			auto p1 = box.curveMin[1] * (box.aabbMax - box.aabbMin) + box.aabbMin;
			auto p2 = box.curveMin[2] * (box.aabbMax - box.aabbMin) + box.aabbMin;

			auto cross = [](const auto& a, const auto& b) {
				return a.x * b.y - a.y * b.x;
			};

			auto cr = cross(p1 - p0, p2 - p0);
			auto sqLen = glm::dot(p1 - p0, p1 - p0) + glm::dot(p2 - p0, p2 - p0);
			bool linear = cr * cr < 1e-4 * sqLen;

			nvgBeginPath(vg);
			nvgMoveTo(vg, p0.x, p0.y);
			nvgQuadTo(vg, p1.x, p1.y, p2.x, p2.y);
			nvgStrokeColor(vg, linear ? nvgRGBA(255, 0, 0, 255) : nvgRGBA(0, 0, 255, 255));
			nvgStrokeWidth(vg, 2.f);
			nvgStroke(vg);

			p0 = box.curveMax[0] * (box.aabbMax - box.aabbMin) + box.aabbMin;
			p1 = box.curveMax[1] * (box.aabbMax - box.aabbMin) + box.aabbMin;
			p2 = box.curveMax[2] * (box.aabbMax - box.aabbMin) + box.aabbMin;

			cr = cross(p1 - p0, p2 - p0);
			sqLen = glm::dot(p1 - p0, p1 - p0) + glm::dot(p2 - p0, p2 - p0);
			linear = cr * cr < 1e-4 * sqLen;

			nvgBeginPath(vg);
			nvgMoveTo(vg, p0.x, p0.y);
			nvgQuadTo(vg, p1.x, p1.y, p2.x, p2.y);
			nvgStrokeColor(vg, linear ? nvgRGBA(255, 0, 0, 255) : nvgRGBA(0, 0, 255, 255));
			nvgStrokeWidth(vg, 2.f);
			nvgStroke(vg);
		}

		for (size_t i = 0; i < sizeof(controlPoints) / sizeof(controlPoints[0]); i += 3) {
			nvgBeginPath(vg);
			nvgMoveTo(vg, controlPoints[i].x, controlPoints[i].y);
			nvgQuadTo(vg, controlPoints[i + 1].x, controlPoints[i + 1].y, controlPoints[i + 2].x,
					controlPoints[i + 2].y);
			nvgStrokeWidth(vg, 2.f);
			nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 255));
			nvgStroke(vg);
		}

		for (size_t i = 0; i < sizeof(controlPoints) / sizeof(controlPoints[0]); ++i) {
			nvgBeginPath(vg);
			nvgCircle(vg, controlPoints[i].x, controlPoints[i].y, 3.f);
			nvgFillColor(vg, nvgRGBA(255, 0, 0, 255));
			nvgFill(vg);
		}

		{
			QuadraticBezier<double> bez0{controlPoints[0], controlPoints[1], controlPoints[2]};
			QuadraticBezier<double> bez1{controlPoints[3], controlPoints[4], controlPoints[5]};
			BezBezDebug<double> debug;
			bez_bez_fast2(bez0, bez1, debug);

			int stepIndex = g_step <= debug.steps.size() - 1 ? g_step : debug.steps.size() - 1;
			auto& step = debug.steps[stepIndex];

			auto drawCurve = [&](const QuadraticBezier<double>& fullBez, const Pair<double, double>& range) {
				if (range.second - range.first < 0.0001) {
					auto p = fullBez.evaluate((range.first + range.second) * 0.5);
					nvgBeginPath(vg);
					nvgCircle(vg, p.x, p.y, 4);
					nvgFillColor(vg, nvgRGBA(0, 255, 255, 255));
					nvgFill(vg);
				}
				else {
					auto bez = fullBez.split_from_min_to_max(range.first, range.second);

					nvgBeginPath(vg);
					nvgMoveTo(vg, bez.P0.x, bez.P0.y);
					nvgQuadTo(vg, bez.P1.x, bez.P1.y, bez.P2.x, bez.P2.y);
					nvgStrokeWidth(vg, 3.f);
					nvgStrokeColor(vg, nvgRGBA(0, 0, 255, 255));
					nvgStroke(vg);
				}
			};

			for (int i = 0; i < step.count; ++i) {
				auto& iter = step.iters[i];
				drawCurve(*iter.F, iter.fRange);
				drawCurve(*iter.G, iter.gRange);
			}
		}

		nvgEndFrame(vg);

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	nsvgDelete(nsvg);

	glDeleteBuffers(1, &curveBuffer);
	glDeleteVertexArrays(1, &vao);

	nvgDeleteGL3(vg);

	glfwDestroyWindow(window);
	glfwTerminate();
}

static void on_mouse_button_event(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_PRESS) {
		double mouseX, mouseY;
		glfwGetCursorPos(window, &mouseX, &mouseY);

		for (auto& controlPoint : controlPoints) {
			if (glm::distance(controlPoint, glm::vec<2, double>{mouseX, mouseY}) < 3.f) {
				selectedPoint = &controlPoint;
				break;
			}
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_1 && action == GLFW_RELEASE) {
		selectedPoint = nullptr;
	}
}

static void on_mouse_move_event(GLFWwindow* window, double xPos, double yPos) {
	if (selectedPoint) {
		*selectedPoint = glm::vec2{std::fminf(std::max(xPos, 0.0), g_width),
				std::fminf(std::max(yPos, 0.0), g_height)};
	}
}

static void on_key_event(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_LEFT && action == GLFW_PRESS) {
		if (g_step > 0) {
			--g_step;
		}
	}
	else if (key == GLFW_KEY_RIGHT && action == GLFW_PRESS) {
		++g_step;
	}

	if (key == GLFW_KEY_S && action == GLFW_PRESS) {
		if (auto* file = fopen("curves.bin", "wb")) {
			fwrite(controlPoints, sizeof(controlPoints), 1, file);
			fclose(file);
		}
	}
	else if (key == GLFW_KEY_R && action == GLFW_PRESS) {
		if (auto* file = fopen("curves.bin", "rb")) {
			fread(controlPoints, sizeof(controlPoints), 1, file);
			fclose(file);
		}
	}
}

static void draw_polygon(NVGcontext* vg, const Polygon& polygon, const glm::vec2& offset,
		NVGcolor color) {
	nvgBeginPath(vg);

	for (auto& subshape : polygon.subShapes) {
		nvgMoveTo(vg, subshape.points[0].x + offset.x, subshape.points[0].y + offset.y);

		for (size_t i = 1; i < subshape.points.size(); ++i) {
			nvgLineTo(vg, subshape.points[i].x + offset.x, subshape.points[i].y + offset.y);
		}
	}

	nvgStrokeColor(vg, color);
	nvgStrokeWidth(vg, 1.f);
	nvgStroke(vg);
}

static void convert_cpu_path_data_to_gpu(const std::vector<CPUQuadraticShape>& shapes, GPUPathData& pathData,
		uint32_t& shaderShapeCount) {
	uint32_t pointCount = 0;
	uint32_t pathCount = 0;
	shaderShapeCount = 0;

	for (auto& shape : shapes) {
		for (auto& path : shape.paths) {
			std::memcpy(&pathData.points[pointCount], path.points.data(),
					path.points.size() * sizeof(glm::vec2));
			pointCount += static_cast<uint32_t>(path.points.size());

			pathData.pathFlags[pathCount] = path.flags;
			pathData.pointCounts[pathCount] = pointCount;
			++pathCount;
		}

		pathData.pathCounts[shaderShapeCount] = pathCount;
		pathData.fillColors[shaderShapeCount] = shape.fillColor;
		pathData.strokeColors[shaderShapeCount] = shape.strokeColor;
		pathData.strokeWidths[shaderShapeCount] = shape.strokeWidth;
		++shaderShapeCount;
	}

	printf("converted %u points, %u paths, %u shapes\n", pointCount, pathCount, shaderShapeCount);
}

static void debug_draw_lines(NVGcontext* vg, GPUPathData& shaderPathData, uint32_t shaderShapeCount) {
	for (uint32_t ishape = 0, ipath = 0, i = 0; ishape < shaderShapeCount; ++ishape) {
		for (; ipath < shaderPathData.pathCounts[ishape]; ++ipath) {
			nvgBeginPath(vg);
			nvgMoveTo(vg, shaderPathData.points[i].x, shaderPathData.points[i].y);

			for (; i < shaderPathData.pointCounts[ipath] - 1; i += 2) {
				//nvgLineTo(vg, shaderPathData.points[i + 2].x, shaderPathData.points[i + 2].y);
				nvgQuadTo(vg, shaderPathData.points[i + 1].x, shaderPathData.points[i + 1].y,
						shaderPathData.points[i + 2].x, shaderPathData.points[i + 2].y);
			}

			nvgStrokeColor(vg, nvgRGBA(0, 255, 0, 255));
			nvgStroke(vg);

			i = shaderPathData.pointCounts[ipath];
		}
	}
}

static void nvg_draw(NVGcontext* vg, NSVGimage* nsvg) {
	for (auto* shape = nsvg->shapes; shape; shape = shape->next) {
		for (auto* path = shape->paths; path; path = path->next) {
			nvgBeginPath(vg);
			nvgMoveTo(vg, path->pts[0] + 300, path->pts[1]);

			for (int i = 0; i < path->npts - 1; i += 3) {
				auto* p = &path->pts[2 * i];
				nvgBezierTo(vg, p[2] + 300, p[3], p[4] + 300, p[5], p[6] + 300, p[7]);
			}

			//nvgStrokeColor(vg, nvgRGBA(0, 255, 0, 255));
			//nvgStroke(vg);

			if (path->closed) {
				nvgClosePath(vg);
			}

			for (int i = 0; i < 3; ++i) {
				auto paintOrder = (shape->paintOrder >> (2 * i)) & 0x2U;

				if (paintOrder == NSVG_PAINT_STROKE && shape->stroke.type != NSVG_PAINT_NONE) {
					auto col = nvgRGBA(shape->stroke.color & 0xFF, (shape->stroke.color >> 8) & 0xFF,
							(shape->stroke.color >> 16) & 0xFF, (shape->stroke.color >> 24) & 0xFF);
					nvgLineCap(vg, NVG_ROUND);
					nvgStrokeColor(vg, col);
					nvgStrokeWidth(vg, shape->strokeWidth);
					nvgStroke(vg);
				}
				else if (paintOrder == NSVG_PAINT_FILL && shape->fill.type != NSVG_PAINT_NONE) {
					auto col = nvgRGBA(shape->fill.color & 0xFF, (shape->fill.color >> 8) & 0xFF,
							(shape->fill.color >> 16) & 0xFF, (shape->fill.color >> 24) & 0xFF);
					nvgFillColor(vg, col);
					nvgFill(vg);
				}
			}
		}
	}
}

static std::optional<std::string> file_read_bytes(const char* fileName) {
	FILE* file = std::fopen(fileName, "rb");

	if (!file) {
		return std::nullopt;
	}

	std::fseek(file, 0, SEEK_END);
	std::string result(std::ftell(file), '\0');
	std::rewind(file);

	std::fread(result.data(), 1, result.size(), file);
	std::fclose(file);

	return result;
}

static void GLAPIENTRY gl_message_callback(GLenum source, GLenum type, GLuint id, GLenum severity,
		GLsizei length, const GLchar* message, const void* userParam) {
	if (type == GL_DEBUG_TYPE_ERROR) {
		fprintf(stderr, "GL CALLBACK: **ERROR** type = 0x%x, severity = 0x%x, message = %s\n", type, severity,
				message);
	}
}
