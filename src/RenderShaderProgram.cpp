#include "RenderShaderProgram.hpp"
#include "Exceptions.hpp"

using namespace gmu;

GLuint RenderShaderProgram::compile() {

    GLint linkStatus;
    GLuint renderProgram;

    if (vertexShader == 0) {
        throw Exception("Could not compile render program: Vertex shader missing.");
    } else if (fragmentShader == 0) {
        throw Exception("Could not compile render program: Fragment shader missing.");
    }


    renderProgram = glCreateProgram();

    glAttachShader(renderProgram, vertexShader);
    glAttachShader(renderProgram, fragmentShader);

    glLinkProgram(renderProgram);

    glGetProgramiv(renderProgram, GL_LINK_STATUS, &linkStatus);

    if (linkStatus != GL_TRUE) {

        string info = getProgramInfo(renderProgram);

        glDeleteProgram(renderProgram);
        throw GLException(info);
    }

    return renderProgram;

}

