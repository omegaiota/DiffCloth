//
// Created by liyifei@csail.mit.edu on 7/28/18.
//

#ifndef OMEGAENGINE_SHADER_H
#define OMEGAENGINE_SHADER_H

#include "Macros.h"
#include <nanogui/nanogui.h>
#include <fstream>
#include <sstream>
class Shader {

public:
    // the program ID
    unsigned int ID;

    // constructor reads and builds the shader
		Shader(const GLchar* vertexName, const GLchar* fragmentName);
    // use/activate the shader
    // utility uniform functions
	void setBool(const std::string &name, bool value) const
	{
		glUniform1i(glGetUniformLocation(ID, name.c_str()), (int)value);
	}
	// ------------------------------------------------------------------------
	void setInt(const std::string &name, int value) const
	{
		glUniform1i(glGetUniformLocation(ID, name.c_str()), value);
	}
	// ------------------------------------------------------------------------
	void setFloat(const std::string &name, float value) const
	{
		glUniform1f(glGetUniformLocation(ID, name.c_str()), value);
	}
	// ------------------------------------------------------------------------
	void setVec2(const std::string &name, const glm::vec2 &value) const
	{
		glUniform2fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
	}

    void setVec2(const std::string &name, float x, float y) const {
      glUniform2f(glGetUniformLocation(ID, name.c_str()), x, y);
    }


    // ------------------------------------------------------------------------
    void setVec3(const std::string &name, const glm::vec3 &value) const {
      glUniform3fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }

    void setVec3(const std::string &name, const nanogui::Color &value) const {
      glUniform3f(glGetUniformLocation(ID, name.c_str()), value[0], value[1], value[2]);
    }


    void setVec3(const std::string &name, float x, float y, float z) const {
      glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
    }

    void setVec3(const std::string &name, double x, double y, double z) const {
      glUniform3f(glGetUniformLocation(ID, name.c_str()), x, y, z);
    }

    void setVec3(const std::string &name, const Vec3d &v) const {
      glUniform3f(glGetUniformLocation(ID, name.c_str()), v[0], v[1], v[2]);
    }

    // ------------------------------------------------------------------------
    void setVec4(const std::string &name, const glm::vec4 &value) const {
      glUniform4fv(glGetUniformLocation(ID, name.c_str()), 1, &value[0]);
    }

    void setVec4(const std::string &name, float x, float y, float z, float w) {
      glUniform4f(glGetUniformLocation(ID, name.c_str()), x, y, z, w);
    }

    // ------------------------------------------------------------------------
	void setMat2(const std::string &name, const glm::mat2 &mat) const
	{
		glUniformMatrix2fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
	}
	// ------------------------------------------------------------------------
	void setMat3(const std::string &name, const glm::mat3 &mat) const
	{
		glUniformMatrix3fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
	}
	// ------------------------------------------------------------------------
	void setMat4(const std::string &name, const glm::mat4 &mat) const
	{
		glUniformMatrix4fv(glGetUniformLocation(ID, name.c_str()), 1, GL_FALSE, &mat[0][0]);
	}
	 
    void use()
    {
		glUseProgram(ID);
    }

	const static std::string getPath(const std::string& vertexPath) {
		std::string shaderFolder;
    shaderFolder = std::string(SOURCE_PATH) + "/src/gpu/shaders/";
		shaderFolder.append(vertexPath).append(".glsl");
		return shaderFolder;
	}
private:

		void checkCompileErrors(unsigned int shader, std::string type)
		{
			int success;
			char infoLog[1024];
			if (type != "PROGRAM")
			{
				glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
				if (!success)
				{
					glGetShaderInfoLog(shader, 1024, NULL, infoLog);
					std::cout << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
				}
			}
			else
			{
				glGetProgramiv(shader, GL_LINK_STATUS, &success);
				if (!success)
				{
					glGetProgramInfoLog(shader, 1024, NULL, infoLog);
					std::cout << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n" << infoLog << "\n -- --------------------------------------------------- -- " << std::endl;
				}
			}
		}
};


#endif //OMEGAENGINE_SHADER_H
