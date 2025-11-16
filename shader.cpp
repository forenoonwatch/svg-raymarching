#include "shader.hpp"

#include <cstdio>

#include <glad/glad.h>

static bool try_compile_program(const char* vertexSource, const char* fragmentSource, unsigned& outProgram);
static bool try_compile_shader(const char* source, GLenum type, unsigned& outShader);

Shader Shader::load(const char* vertexSource, const char* fragmentSource) {
	unsigned program;

	if (!try_compile_program(vertexSource, fragmentSource, program)) {
		return {};
	}

	return Shader{program};
}

Shader::~Shader() {
	if (m_program != 0) {
		glDeleteProgram(m_program);
	}
}

void Shader::bind() {
	glUseProgram(m_program);
}

static bool try_compile_program(const char* vertexSource, const char* fragmentSource, unsigned& outProgram) {
	outProgram = 0;

	if (!vertexSource) {
		return false;
	}

	unsigned vertexShader{};
	unsigned fragmentShader{};

	if (!try_compile_shader(vertexSource, GL_VERTEX_SHADER, vertexShader)) {
		return false;
	}

	if (fragmentSource && !try_compile_shader(fragmentSource, GL_FRAGMENT_SHADER, fragmentShader)) {
		glDeleteShader(vertexShader);
		return false;
	}

	unsigned shaderProgram = glCreateProgram();

	glAttachShader(shaderProgram, vertexShader);

	if (fragmentShader) {
		glAttachShader(shaderProgram, fragmentShader);
	}

	glLinkProgram(shaderProgram);

	int success;
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);

	if (success) {
		outProgram = shaderProgram;
	}
	else {
		char infoLog[512]{};
		glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
		printf("[GL] %s\n", infoLog);

		glDeleteProgram(shaderProgram);
	}

	glDeleteShader(vertexShader);

	if (fragmentShader) {
		glDeleteShader(fragmentShader);
	}

	return success;
}

static bool try_compile_shader(const char* source, GLenum type, unsigned& outShader) {
	outShader = 0;
	unsigned shader = glCreateShader(type);
	glShaderSource(shader, 1, &source, nullptr);
	glCompileShader(shader);

	int success{};
	glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
	if (!success) {
		char infoLog[512]{};
		glGetShaderInfoLog(shader, 512, nullptr, infoLog);
		printf("[GL] %s\n", infoLog);

		glDeleteShader(shader);
		return false;
	}

	outShader = shader;
	return true;
}
