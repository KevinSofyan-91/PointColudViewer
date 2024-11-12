#pragma once
#include <string>
#include <iostream>
#include <fstream>

#include <GL\glew.h>

class ShaderUtils
{
	unsigned int shader;
public:
	std::string readShaderFile(const char*);
	unsigned int shaderCompile(unsigned int, const std::string&);
	unsigned int createShaders(const std::string&, const std::string&);
};

