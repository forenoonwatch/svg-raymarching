#pragma once

#include "common.hpp"

class Shader {
	public:
		static Shader load(const char* vertexSource, const char* fragmentSource);

		constexpr Shader() = default;
		constexpr explicit Shader(unsigned program)
				: m_program(program) {}

		~Shader();

		NULL_COPY_AND_ASSIGN(Shader);

		void bind();
	private:
		unsigned m_program{};
};
