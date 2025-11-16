#pragma once

#include <cmath>
#include <numbers>

#include <glm/mat2x2.hpp>
#include <glm/vec2.hpp>
#include <glm/geometric.hpp>

template <typename float_t>
float angle_between_vectors(const glm::vec<2, float_t>& u, const glm::vec<2, float_t>& v) {
	auto len = glm::length(u) * glm::length(v);
	return std::acos(glm::dot(u, v) / len) * ((u.x * v.y - u.y * v.x) < 0 ? float_t(-1.0) : float_t(1.0));
}

template <typename float_t>
void ellipse_arc(const glm::vec<2, float_t>& p0, const glm::vec<2, float_t>& p1, float_t rx, float_t ry,
		float_t xAxisRotation, uint32_t largeArcFlag, uint32_t sweepFlag, glm::vec<2, float_t>& outMajorAxis,
		glm::vec<2, float_t>& outCenter, glm::vec<2, float_t>& outAngleBounds, float_t& outEccentricity) {
	using vec2 = glm::vec<2, float_t>;

	// https://www.w3.org/TR/SVG11/implnote.html#ArcImplementationNotes
	auto cosPhi = std::cos(xAxisRotation);
	auto sinPhi = std::sin(xAxisRotation);
	glm::mat<2, 2, float_t> rot(vec2(cosPhi, sinPhi), vec2(-sinPhi, cosPhi));
	auto invRot = glm::transpose(rot);

	auto pPrime = rot * ((p0 - p1) * float_t(0.5));
	auto xPrime2 = pPrime.x * pPrime.x;
	auto yPrime2 = pPrime.y * pPrime.y;

	auto lambda = xPrime2 / (rx * rx) + yPrime2 / (ry * ry);

	float_t d = float_t(0.0);

	if (lambda > float_t(1.0)) {
		lambda = std::sqrt(lambda);
		rx *= lambda;
		ry *= lambda;
	}
	else {
		auto rx2 = rx * rx;
		auto ry2 = ry * ry;
		auto numer = rx2 * ry2 - rx2 * yPrime2 - ry2 * xPrime2;
		auto denom = rx2 * yPrime2 + ry2 * xPrime2;

		d = std::sqrt(numer / denom) * (largeArcFlag == sweepFlag ? float_t(-1.0) : float_t(1.0));
	}

	auto cPrime = vec2(rx * pPrime.y / ry, -ry * pPrime.x / rx) * d;

	outCenter = invRot * cPrime + (p0 + p1) * float_t(0.5);

	auto scaledDPrime = (pPrime - cPrime) / glm::vec2(rx, ry);
	auto dTheta = angle_between_vectors(scaledDPrime, (-pPrime - cPrime) / vec2(rx, ry));

	if (!sweepFlag && dTheta > 0) {
		dTheta -= float_t(2.0) * std::numbers::pi_v<float_t>;
	}
	else if (sweepFlag && dTheta < 0) {
		dTheta += float_t(2.0) * std::numbers::pi_v<float_t>;
	}

	outAngleBounds.x = angle_between_vectors(vec2(1, 0), scaledDPrime);
	outAngleBounds.y = outAngleBounds.x + dTheta;

	outMajorAxis = rot[0] * rx;
	outEccentricity = ry / rx;
}
