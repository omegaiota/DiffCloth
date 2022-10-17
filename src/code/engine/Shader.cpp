//
// Created by liyifei@csail.mit.edu on 7/28/18.
//
#include "Shader.h"

Shader::Shader(const GLchar* vertexName, const GLchar* fragmentName){
  // 1. retrieve the vertex/fragment source code from filePath
  std::string vertexCode;
  std::string fragmentCode;
  std::string vertexPathStr = Shader::getPath(vertexName);
  std::string fragmentPathStr = Shader::getPath(fragmentName);
  std::ifstream vShaderFile(vertexPathStr);
  std::ifstream fShaderFile(fragmentPathStr);
  std::printf("loading shader from %s %s...", vertexName, fragmentName);
  // ensure ifstream objects can throw exceptions:
  vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  try {
    DEBUG_PRINTF(("Current path is\n"));
    std::string pwdStr("pwd");
    const char *command = pwdStr.c_str();
    system(command);
    DEBUG_PRINTF(("SHADER::TRYING TO OPEN SHADERS...\n"));
    vertexCode = std::string((std::istreambuf_iterator<char>(vShaderFile)), std::istreambuf_iterator<char>());
    fragmentCode = std::string((std::istreambuf_iterator<char>(fShaderFile)), std::istreambuf_iterator<char>());
  }
	catch (std::ifstream::failure e)
	{
    DEBUG_PRINTF( ("ERROR::SHADER::FILE_NOT_SUCCESFULLY_READ\n") );
    DEBUG_PRINTF( ("ERROR::SHADER::%s\n", e.what()) );
	}
	const char* vShaderCode = vertexCode.c_str();
	const char * fShaderCode = fragmentCode.c_str();
	// 2. compile shaders
	unsigned int vertex, fragment;
	// vertex shader
	vertex = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex, 1, &vShaderCode, NULL);
	glCompileShader(vertex);
	checkCompileErrors(vertex, "VERTEX");
	// fragment Shader
	fragment = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment, 1, &fShaderCode, NULL);
	glCompileShader(fragment);
	checkCompileErrors(fragment, "FRAGMENT");
	// shader Program
	ID = glCreateProgram();
	glAttachShader(ID, vertex);
	glAttachShader(ID, fragment);
	glLinkProgram(ID);
	checkCompileErrors(ID, "PROGRAM");
	// delete the shaders as they're linked into our program now and no longer necessary
	glDeleteShader(vertex);
	glDeleteShader(fragment);
//	std::printf("success!\n");
}

