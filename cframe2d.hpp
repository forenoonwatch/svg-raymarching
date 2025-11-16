#pragma once

#include <cassert>

#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/mat2x2.hpp>

namespace ZN {

class CFrame2D {
	public:
		CFrame2D() = default;
		CFrame2D(float diagonal)
				: m_columns{{diagonal, 0.f}, {0.f, diagonal}, {}} {}

		CFrame2D(float x, float y)
				: m_columns{{1.f, 0.f}, {0.f, 1.f}, {x, y}} {}
		CFrame2D(glm::vec2 translation)
				: m_columns{{1.f, 0.f}, {0.f, 1.f}, std::move(translation)} {}

		 CFrame2D& set_translation(const glm::vec2& translation) {
			m_columns[2] = translation;

			return *this;
		}

		 CFrame2D& set_translation(float x, float y) {
			m_columns[2][0] = x;
			m_columns[2][1] = y;

			return *this;
		}

		 CFrame2D& set_rotation(float rotation) {
			float s = sin(rotation);
			float c = cos(rotation);

			m_columns[0][0] = c;
			m_columns[0][1] = s;
			m_columns[1][0] = -s;
			m_columns[1][1] = c;

			return *this;
		}

		 CFrame2D& apply_scale(float scale) {
			m_columns[0] *= scale;
			m_columns[1] *= scale;

			return *this;
		}

		 CFrame2D& apply_scale_local(const glm::vec2& scale) {
			m_columns[0] *= scale.x;
			m_columns[1] *= scale.y;

			return *this;
		}

		 CFrame2D& apply_scale_global(const glm::vec2& scale) {
			m_columns[0] *= scale;
			m_columns[1] *= scale;

			return *this;
		}

		CFrame2D& fast_inverse_self() {
			get_rotation_matrix() = transpose(get_rotation_matrix());
			m_columns[2] *= -1.f;

			return *this;
		}

		CFrame2D fast_inverse() const {
			return CFrame2D(*this).fast_inverse_self();
		}

		glm::vec2 point_to_local_space(const glm::vec2& point) const {
			return transpose(get_rotation_matrix()) * (point - m_columns[2]);
		}

		glm::vec2& operator[](size_t index) {
			return m_columns[index];
		}

		const glm::vec2& operator[](size_t index) const {
			return m_columns[index];
		}

		CFrame2D& operator+=(const glm::vec2& c) {
			m_columns[2] += c;

			return *this;
		}

		CFrame2D& operator-=(const glm::vec2& c) {
			m_columns[2] -= c;

			return *this;
		}

		CFrame2D& operator*=(const CFrame2D& c) {
			m_columns[2] += get_rotation_matrix() * c.m_columns[2];
			get_rotation_matrix() *= c.get_rotation_matrix();

			return *this;
		}

		glm::vec3 row(size_t index) const {
			switch (index) {
				case 0:
					return glm::vec3(m_columns[0][0], m_columns[1][0], m_columns[2][0]);
				case 1:
					return glm::vec3(m_columns[0][1], m_columns[1][1], m_columns[2][1]);
				default:
					return glm::vec3();
			}
		}

		glm::mat2& get_rotation_matrix() {
			return *reinterpret_cast<glm::mat2*>(this);
		}

		const glm::mat2& get_rotation_matrix() const {
			return *reinterpret_cast<const glm::mat2*>(this);
		}

		glm::vec2 get_scale() const {
			return {glm::length(m_columns[0]), glm::length(m_columns[1])};
		}
	private:
		glm::vec2 m_columns[3];
};

inline CFrame2D operator*(const CFrame2D& a, const CFrame2D& b) {
	auto result = a;
	result *= b;

	return result;
}

inline glm::vec2 operator*(const CFrame2D& c, const glm::vec3& v) {
	return glm::vec2(dot(c.row(0), v), dot(c.row(1), v));
}

inline glm::vec2 operator*(const CFrame2D& c, const glm::vec2& v) {
	return c.get_rotation_matrix() * v + c[2];
}

inline CFrame2D operator+(const CFrame2D& c, const glm::vec2& v) {
	auto result = c;
	result += v;

	return result;
}

inline CFrame2D operator-(const CFrame2D& c, const glm::vec2& v) {
	auto result = c;
	result -= v;

	return result;
}

}
