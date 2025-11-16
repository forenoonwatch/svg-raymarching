#include "svg.hpp"

#include "visitor_base.hpp"
#include "subdivision.hpp"

#include <glm/mat2x2.hpp>

using namespace ZN;

static void subdiv_path(const Path& path, const ScenePathCommand& cmdPath,
		std::vector<CPUQuadraticShape>& outShapes, float maxErrorTolerance);
static void subdiv_ellipse(const SceneEllipseCommand& c,
		std::vector<CPUQuadraticShape>& outShapes, float maxErrorTolerance);

void svg_convert_to_quadratic_paths(const Scene& scene, std::vector<CPUQuadraticShape>& outShapes,
		float maxErrorTolerance) {
	scene.for_each_command(VisitorBase {
		[&](const ScenePathCommand& cmdPath) {
			subdiv_path(scene.paths[cmdPath.path], cmdPath, outShapes, maxErrorTolerance);
		},
		[&](const SceneEllipseCommand& c) {
			subdiv_ellipse(c, outShapes, maxErrorTolerance);
		},
	});
}

static void subdiv_path(const Path& path, const ScenePathCommand& cmdPath,
		std::vector<CPUQuadraticShape>& outShapes, float maxErrorTolerance) {
	Subdivision<float> subdiv;
	uint32_t subpathPointCount = 0;
	glm::vec2 initialPoint{};
	glm::vec2 cursor{};

	outShapes.emplace_back();
	auto& shape = outShapes.back();
	shape.fillColor = cmdPath.fillColor;
	shape.strokeColor = cmdPath.strokeColor;
	shape.strokeWidth = cmdPath.strokeWidth * 0.5f;

	path.for_each_command(VisitorBase {
		[&](const MoveToCommand& c) {
			if (subpathPointCount == 0) {
				initialPoint = c.pos;
				shape.paths.emplace_back();
				shape.paths.back().points.emplace_back(c.pos);
			}
			else {
				// https://svgwg.org/svg2-draft/paths.html#PathDataMovetoCommands
				// If a moveto is followed by multiple pairs of coordinates, the subsequent pairs are
				// treated as implicit lineto commands
				shape.paths.back().points.emplace_back(c.pos);
				shape.paths.back().points.emplace_back(c.pos);
			}

			cursor = c.pos;
			++subpathPointCount;
		},
		[&](const LineToCommand& c) {
			if (subpathPointCount == 0) {
				initialPoint = cursor;
				shape.paths.emplace_back();
				shape.paths.back().points.emplace_back(cursor);
			}

			shape.paths.back().points.emplace_back(c.pos);
			shape.paths.back().points.emplace_back(c.pos);

			cursor = c.pos;
			++subpathPointCount;
		},
		[&](const QuadBezierToCommand& c) {
			if (subpathPointCount == 0) {
				initialPoint = cursor;
				shape.paths.emplace_back();
				shape.paths.back().points.emplace_back(cursor);
			}

			shape.paths.back().points.emplace_back(c.c0);
			shape.paths.back().points.emplace_back(c.pos);

			cursor = c.pos;
			++subpathPointCount;
		},
		[&](const BezierToCommand& c) {
			if (subpathPointCount == 0) {
				initialPoint = cursor;
				shape.paths.emplace_back();
				shape.paths.back().points.emplace_back(cursor);
			}

			auto curve = CubicCurve<float>::from_bezier(cursor, c.c0, c.c1, c.pos);

			subdiv.adaptive(curve, 0, 1, maxErrorTolerance, [&](auto&& quadCurve) {
				shape.paths.back().points.emplace_back(quadCurve.P1);
				shape.paths.back().points.emplace_back(quadCurve.P2);
			});

			cursor = c.pos;
			++subpathPointCount;
		},
		[&](const ArcToCommand& c) {
			if (subpathPointCount == 0) {
				initialPoint = cursor;
				shape.paths.emplace_back();
				shape.paths.back().points.emplace_back(cursor);
			}

			// https://www.w3.org/TR/SVG11/implnote.html#ArcImplementationNotes
			// F.6.6 Correction of out-of-range radii
			// If rx = 0 or ry = 0, then treat this as a straight line from (x1, y1) to (x2, y2)
			// and stop
			if (c.eccentricity == 0.f) {
				shape.paths.back().points.emplace_back(c.pos);
				shape.paths.back().points.emplace_back(c.pos);
			}
			else {
				EllipticalArcInfo<float> arc{
					.majorAxis = c.majorAxis,
					.center = c.center,
					.angleBounds = c.angleBounds,
					.eccentricity = c.eccentricity,
				};

				AxisAlignedEllipse<float> aaEllipse(glm::length(arc.majorAxis),
						glm::length(arc.majorAxis) * arc.eccentricity, arc.angleBounds.x,
						arc.angleBounds.y);
				auto nm = glm::normalize(arc.majorAxis);
				glm::mat2 rotate{glm::vec2(nm.x, -nm.y), glm::vec2(nm.y, nm.x)};

				auto p0 = glm::transpose(rotate) * aaEllipse.compute_position(0.f) + arc.center;
				auto p1 = glm::transpose(rotate) * aaEllipse.compute_position(1.f) + arc.center;

				subdiv.adaptive(arc, 0, 1, maxErrorTolerance, [&](auto&& quadCurve) {
					shape.paths.back().points.emplace_back(quadCurve.P1);
					shape.paths.back().points.emplace_back(quadCurve.P2);
				});
			}

			cursor = c.pos;
			++subpathPointCount;
		},
		[&](CloseCommand) {
			if (subpathPointCount > 0) {
				if (glm::distance(cursor, initialPoint) > 1e-3f) {
					shape.paths.back().points.emplace_back(initialPoint);
					shape.paths.back().points.emplace_back(initialPoint);
				}
			}

			subpathPointCount = 0;
		},
	});

	if (subpathPointCount > 0) {
		shape.paths.back().flags |= PATH_FLAG_OPEN;
	}
}

static void subdiv_ellipse(const SceneEllipseCommand& c,
		std::vector<CPUQuadraticShape>& outShapes, float maxErrorTolerance) {
	Subdivision<float> subdiv;
	EllipticalArcInfo<float> ellipse{
		.majorAxis = c.majorAxis,
		.center = c.center,
		.angleBounds = glm::vec2(0.f, 2.f * std::numbers::pi_v<float>),
		.eccentricity = c.eccentricity,
	};
	bool firstPoint = true;

	outShapes.emplace_back();
	auto& shape = outShapes.back();
	shape.fillColor = c.fillColor;
	shape.strokeColor = c.strokeColor;
	shape.strokeWidth = c.strokeWidth * 0.5f;

	shape.paths.emplace_back();

	subdiv.adaptive(ellipse, 0, 1, 1.f, [&](auto&& quadCurve) {
		if (firstPoint) {
			firstPoint = false;
			shape.paths.back().points.emplace_back(quadCurve.P0);
		}

		shape.paths.back().points.emplace_back(quadCurve.P1);
		shape.paths.back().points.emplace_back(quadCurve.P2);
	});
}
