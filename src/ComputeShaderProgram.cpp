#include "ComputeShaderProgram.hpp"
#include "Exceptions.hpp"

using namespace gmu;

GLuint ComputeShaderProgram::compile() {
    GLint linkStatus;
    GLuint renderProgram;

    if (computeShader == 0) {
        throw Exception("Could not compile compute program: Compute shader missing.");
    }

    renderProgram = glCreateProgram();

    glAttachShader(renderProgram, computeShader);

    glLinkProgram(renderProgram);

    glGetProgramiv(renderProgram, GL_LINK_STATUS, &linkStatus);

    if (linkStatus != GL_TRUE) {

        string info = getProgramInfo(renderProgram);

        glDeleteProgram(renderProgram);
        throw GLException(info);
    }

    return renderProgram;
}


