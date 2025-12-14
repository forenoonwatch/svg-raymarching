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
#include "bezier_clip.hpp"

#include <glm/mat2x2.hpp>

#include <boost/multiprecision/cpp_bin_float.hpp>

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
		
		/*for (auto& pEvent : ownedEvents) {
			sortedEvents.emplace_back(pEvent.get());
		}*/

		for (auto* event : sortedEvents) {
			if (!event->left) {
				continue;
			}

			//assert(event->t < event->otherEvent->t);
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

				for (auto t : sects) {
					if (isnan(t) || segB.tStart >= t || t >= segB.tEnd) {
						continue;
					}

					auto point = segB.originalBezier->evaluate(t);
					auto s = Hatch::intersect_ortho(*segA.originalBezier, point[major], major);

					if (!isnan(s) && s > segA.tStart && s < segA.tEnd) {
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
			using std::abs;
			using std::isnan;

			using f_t = long double;//boost::multiprecision::cpp_bin_float_oct;
			QuadraticBezier<f_t> bez0{controlPoints[0], controlPoints[1], controlPoints[2]};
			QuadraticBezier<f_t> bez1{controlPoints[3], controlPoints[4], controlPoints[5]};
			BezBezDebug<f_t> debug;

			std::array<Pair<f_t, f_t>, 2> results;
			int nresults = bez_bez_intersection_clipping(bez0, bez1, results, debug);

			int stepIndex = g_step <= debug.steps.size() - 1 ? g_step : debug.steps.size() - 1;
			auto& step = debug.steps[stepIndex];

			auto drawCurve = [&](const QuadraticBezier<f_t>& fullBez, const Pair<f_t, f_t>& range,
					NVGcolor color, bool isF) {
				/*if (range.second - range.first < 0.0001) {
					auto p = fullBez.evaluate((range.first + range.second) * 0.5);
					nvgBeginPath(vg);
					nvgCircle(vg, p.x, p.y, 4);
					nvgFillColor(vg, nvgRGBA(0, 255, 255, 255));
					nvgFill(vg);
				}
				else*/ {
					auto bez = fullBez.split_from_min_to_max(range.first, range.second);
					auto lineOffset = distance_to_line(bez.P0, bez.P2, bez.P1);
					auto rawPerp = glm::vec<2, f_t>(bez.P0.y - bez.P2.y, bez.P2.x - bez.P0.x);
					auto perp = glm::normalize(rawPerp);

					auto offs0 = bez.P0 + perp * -lineOffset;
					auto offs2 = bez.P2 + perp * -lineOffset;

					auto perpOffs2 = distance_to_line(bez.P0, bez.P0 + rawPerp, bez.P2);
					auto perpOffs1 = distance_to_line(bez.P0, bez.P0 + rawPerp, bez.P1);
					auto perpOffsDist = abs(perpOffs2) > abs(perpOffs1) ? perpOffs2 : perpOffs1;

					auto otherPerpStart = bez.P0 + glm::normalize(bez.P2 - bez.P0) * perpOffsDist;

					if (isF) {
						nvgBeginPath(vg);
						nvgMoveTo(vg, (float)bez.P0.x, (float)bez.P0.y);
						nvgLineTo(vg, (float)bez.P2.x, (float)bez.P2.y);
						nvgStrokeWidth(vg, 2.f);
						nvgStrokeColor(vg, nvgRGBA(255, 0, 255, 255));
						nvgStroke(vg);

						nvgBeginPath(vg);
						nvgMoveTo(vg, (float)offs0.x, (float)offs0.y);
						nvgLineTo(vg, (float)offs2.x, (float)offs2.y);
						nvgStrokeWidth(vg, 2.f);
						nvgStrokeColor(vg, nvgRGBA(255, 0, 255, 255));
						nvgStroke(vg);

						nvgBeginPath(vg);
						nvgMoveTo(vg, (float)bez.P0.x, (float)bez.P0.y);
						nvgLineTo(vg, float(bez.P0.x - rawPerp.x), float(bez.P0.y - rawPerp.y));
						nvgStrokeWidth(vg, 2.f);
						nvgStrokeColor(vg, nvgRGBA(255, 255, 0, 255));
						nvgStroke(vg);

						nvgBeginPath(vg);
						nvgMoveTo(vg, (float)otherPerpStart.x, (float)otherPerpStart.y);
						nvgLineTo(vg, float(otherPerpStart.x - rawPerp.x),
								float(otherPerpStart.y - rawPerp.y));
						nvgStrokeWidth(vg, 2.f);
						nvgStrokeColor(vg, nvgRGBA(255, 255, 0, 255));
						nvgStroke(vg);
					}

					nvgBeginPath(vg);
					nvgMoveTo(vg, (float)bez.P0.x, (float)bez.P0.y);
					nvgQuadTo(vg, (float)bez.P1.x, (float)bez.P1.y, (float)bez.P2.x, (float)bez.P2.y);
					nvgStrokeWidth(vg, 3.f);
					nvgStrokeColor(vg, color);
					nvgStroke(vg);
				}
			};

			//drawCurve(*step.F, step.fRange, nvgRGBA(0, 0, 255, 255), true);
			//drawCurve(*step.G, step.gRange, nvgRGBA(0, 0, 128, 255), false);

			auto subF = step.F->split_from_min_to_max(step.fRange.first, step.fRange.second);

			auto perp = subF.P0 + glm::vec<2, f_t>(subF.P0.y - subF.P2.y, subF.P2.x - subF.P0.x);
			auto offs2 = distance_to_line(subF.P0, perp, subF.P2);
			auto lineOffset = distance_to_line(subF.P0, perp, subF.P1);

			if (abs(offs2) > abs(lineOffset)) {
				lineOffset = offs2;
			}

			auto resLow = intersect_bez_line(*step.G, subF.P0, perp, f_t{0.0}, true);
			auto resHigh = intersect_bez_line(*step.G, subF.P0, perp, -lineOffset, true);

			for (auto t : resLow) {
				if (!isnan(t)) {
					auto p = step.G->evaluate(t);
					nvgBeginPath(vg);
					nvgCircle(vg, (float)p.x, (float)p.y, 3.f);
					nvgFillColor(vg, nvgRGBA(255, 255, 0, 255));
					nvgFill(vg);
				}
			}

			for (auto t : resHigh) {
				if (!isnan(t)) {
					auto p = step.G->evaluate(t);
					nvgBeginPath(vg);
					nvgCircle(vg, (float)p.x, (float)p.y, 3.f);
					nvgFillColor(vg, nvgRGBA(255, 255, 0, 255));
					nvgFill(vg);
				}
			}

			for (int i = 0; i < nresults; ++i) {
				auto p = bez0.evaluate(results[i].first);
				nvgBeginPath(vg);
				nvgCircle(vg, (float)p.x, (float)p.y, 3.f);
				nvgFillColor(vg, nvgRGBA(0, 255, 0, 255));
				nvgFill(vg);

				auto p2 = bez0.evaluate(results[i].second);
				nvgBeginPath(vg);
				nvgCircle(vg, (float)p2.x, (float)p2.y, 3.f);
				nvgFillColor(vg, nvgRGBA(0, 255, 0, 255));
				nvgFill(vg);
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
