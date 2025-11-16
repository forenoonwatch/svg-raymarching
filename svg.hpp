#pragma once

#include <cstddef>
#include <cstdint>

#include <vector>

#include <glm/vec2.hpp>

enum class PathCommand : uint32_t {
	MOVE_TO,
	LINE_TO,
	QUAD_BEZIER_TO,
	BEZIER_TO,
	ARC_TO,
	CLOSE,
};

struct MoveToCommand {
	glm::vec2 pos;
};

struct LineToCommand {
	glm::vec2 pos;
};

struct QuadBezierToCommand {
	glm::vec2 c0;
	glm::vec2 pos;
};

struct BezierToCommand {
	glm::vec2 c0;
	glm::vec2 c1;
	glm::vec2 pos;
};

struct ArcToCommand {
	glm::vec2 majorAxis;
	glm::vec2 center;
	glm::vec2 angleBounds;
	glm::vec2 pos;
	float eccentricity;
};

struct CloseCommand {};

struct Path {
	std::vector<uint32_t> commands;

	void push_move_to(float x, float y);
	void push_move_to(const glm::vec2&);
	void push_line_to(float x, float y);
	void push_line_to(const glm::vec2&);
	void push_quad_bezier_to(float x1, float y1, float x, float y);
	void push_bezier_to(float x1, float y1, float x2, float y2, float x, float y);
	void push_arc_to(const glm::vec2& p0, const glm::vec2& p1, float rx, float ry, float xAxisRotation,
			uint32_t largeArcFlag, uint32_t sweepFlag);
	void push_arc_to(const glm::vec2& majorAxis, const glm::vec2& center, const glm::vec2& angleBounds,
			float eccentricity, const glm::vec2& pos);
	void push_close();

	template <typename Visitor>
	void for_each_command(Visitor&& visitor) {
		for (size_t i = 0; i < commands.size();) {
			switch (static_cast<PathCommand>(commands[i])) {
				case PathCommand::MOVE_TO:
					visitor(*reinterpret_cast<MoveToCommand*>(&commands[i + 1]));
					i += 3;
					break;
				case PathCommand::LINE_TO:
					visitor(*reinterpret_cast<LineToCommand*>(&commands[i + 1]));
					i += 3;
					break;
				case PathCommand::QUAD_BEZIER_TO:
					visitor(*reinterpret_cast<QuadBezierToCommand*>(&commands[i + 1]));
					i += 5;
					break;
				case PathCommand::BEZIER_TO:
					visitor(*reinterpret_cast<BezierToCommand*>(&commands[i + 1]));
					i += 7;
					break;
				case PathCommand::ARC_TO:
					visitor(*reinterpret_cast<ArcToCommand*>(&commands[i + 1]));
					i += 10;
					break;
				case PathCommand::CLOSE:
					visitor(CloseCommand{});
					++i;
					break;
				default:
					++i;
					break;
			}
		}
	}

	template <typename Visitor>
	void for_each_command(Visitor&& visitor) const {
		return const_cast<Path*>(this)->for_each_command(visitor);
	}

	void print() const;
};

enum class SceneCmdType : uint32_t {
	PATH,
	ELLIPSE,
};

struct SceneCommand {
	SceneCmdType type;
	uint32_t fillColor;
	uint32_t strokeColor;
	float strokeWidth;
};

struct ScenePathCommand : public SceneCommand {
	uint32_t path;
};

struct SceneEllipseCommand : public SceneCommand {
	glm::vec2 center;
	glm::vec2 majorAxis;
	float eccentricity;
};

struct Scene {
	std::vector<uint32_t> commands;
	std::vector<Path> paths;

	void push_path_command(uint32_t fillColor, uint32_t strokeColor, float strokeWidth);
	void push_ellipse_command(uint32_t fillColor, uint32_t strokeColor, float strokeWidth,
			const glm::vec2& center, const glm::vec2& majorAxis, float eccentricity);

	template <typename Visitor>
	void for_each_command(Visitor&& visitor) {
		for (size_t i = 0; i < commands.size();) {
			auto* pCmd = reinterpret_cast<SceneCommand*>(&commands[i]);

			switch (pCmd->type) {
				case SceneCmdType::PATH:
					visitor(*static_cast<ScenePathCommand*>(pCmd));
					i += sizeof(ScenePathCommand) / sizeof(uint32_t);
					break;
				case SceneCmdType::ELLIPSE:
					visitor(*static_cast<SceneEllipseCommand*>(pCmd));
					i += sizeof(SceneEllipseCommand) / sizeof(uint32_t);
					break;
				default:
					++i;
					break;
			}
		}
	}

	template <typename Visitor>
	void for_each_command(Visitor&& visitor) const {
		const_cast<Scene*>(this)->for_each_command(std::forward<Visitor>(visitor));
	}
};

enum PathFlags : uint32_t {
	PATH_FLAG_OPEN = 1,
};

struct CPUQuadraticPath {
	std::vector<glm::vec2> points;
	uint32_t flags;
};

struct CPUQuadraticShape {
	std::vector<CPUQuadraticPath> paths;
	uint32_t fillColor;
	uint32_t strokeColor;
	float strokeWidth;
};

bool svg_parse(const char* str, size_t size, Scene& result);
void svg_convert_to_quadratic_paths(const Scene& scene, std::vector<CPUQuadraticShape>& outShapes,
		float maxErrorTolerance = 1.f);
