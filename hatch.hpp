#pragma once

#include <glm/vec2.hpp>

#include <vector>

struct CPUQuadraticShape;

struct CurveHatchBox {
	glm::vec2 aabbMin;
	glm::vec2 aabbMax;
	glm::vec2 curveMin[3];
	glm::vec2 curveMax[3];
};

void generate_hatch_boxes(const CPUQuadraticShape& shape, std::vector<CurveHatchBox>& result);
