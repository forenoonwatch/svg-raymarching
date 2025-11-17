#pragma once

#include <glm/vec2.hpp>

#include <vector>

struct PolygonSubShape {
	std::vector<glm::vec2> points;
};

struct Polygon {
	std::vector<PolygonSubShape> subShapes;
};

enum class BooleanOperation {
	INTERSECTION,
	DIFFERENCE,
	UNION,
	XOR,
};

void martinez_boolean(const Polygon& subject, const Polygon& clipping, Polygon& result,
		BooleanOperation operation);
