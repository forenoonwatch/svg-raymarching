#include <cstdio>
#include <cstring>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <optional>
#include <string>
//#include <cmath>
//#include <numbers>

#include "svg.hpp"

#define NANOVG_GL3_IMPLEMENTATION
#include <nanovg.h>
#include <nanovg_gl.h>

#define NANOSVG_ALL_COLOR_KEYWORDS
#define NANOSVG_IMPLEMENTATION
#include <nanosvg.h>

#include "shader.hpp"
#include "svg_shader.hpp"

#include "martinez.hpp"

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

int main() {
	glfwInit();

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
	auto* window = glfwCreateWindow(g_width, g_height, "Jong", nullptr, nullptr);

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

	NSVGimage* nsvg = nsvgParseFromFile("../third_party/riichi-mahjong-tiles/Regular/Chun.svg", "px", 96);

	unsigned vao;
	glCreateVertexArrays(1, &vao);

	auto fstShader = Shader::load(g_svgVertex, g_svgFragment);

	unsigned curveBuffer;
	glCreateBuffers(1, &curveBuffer);

	GPUPathData shaderPathData;
	uint32_t shaderShapeCount = 0;

	std::vector<CPUQuadraticShape> svgShapes;
	svg_convert_to_quadratic_paths(svg, svgShapes);
	convert_cpu_path_data_to_gpu(svgShapes, shaderPathData, shaderShapeCount);

	glNamedBufferStorage(curveBuffer, sizeof(shaderPathData), &shaderPathData, GL_DYNAMIC_STORAGE_BIT);

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

	while (!glfwWindowShouldClose(window)) {
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
		
		nvgBeginFrame(vg, g_width, g_height, 1.0);

		draw_polygon(vg, subject, {200, 300}, nvgRGBA(0, 255, 0, 255));
		draw_polygon(vg, clipping, {200, 300}, nvgRGBA(0, 255, 0, 255));
		draw_polygon(vg, result, {200, 300}, nvgRGBA(255, 0, 0, 255));

		//debug_draw_lines(vg, shaderPathData, shaderShapeCount);

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
