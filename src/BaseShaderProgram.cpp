#include <GL/glew.h>
#include <fstream>
#include <iterator>

#include "BaseShaderProgram.hpp"
#include "Exceptions.hpp"

using namespace gmu;
using namespace std;

BaseShaderProgram::BaseShaderProgram() : program(0) {

}

GLuint BaseShaderProgram::getProgram() {
    if (program == 0) {
        program = compile();
    }

    return program;
}

string BaseShaderProgram::getFileContents(string &filename) {
    ifstream fileStream(filename.c_str());

    if (!fileStream) {
        throw string("Could not load file '" + filename + "'.");
    }

    return string(istream_iterator<char>(fileStream >> noskipws), istream_iterator<char>());
}

GLuint BaseShaderProgram::createShaderFromSource(string &source, GLenum type) {
    GLint compileStatus;
    GLuint shader = glCreateShader(type);
    const char *cSource = source.c_str();

    glShaderSource(shader, 1, &cSource, NULL);

    glCompileShader(shader);

    glGetShaderiv(shader, GL_COMPILE_STATUS, &compileStatus);

    if (compileStatus != GL_TRUE) {

        string info = getShaderInfo(shader);

        glDeleteShader(shader);
        throw GLException(info);
    }

    return shader;
}

GLuint BaseShaderProgram::createShaderFromFile(string &filename, GLenum type) {
    string source = getFileContents(filename);

    return createShaderFromSource(source, type);
}

string BaseShaderProgram::getShaderInfo(GLuint shader) {
    GLint infoLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &infoLength);

    string info;
    info.resize(infoLength);

    glGetShaderInfoLog(shader, infoLength, &infoLength, &info[0]);

    return info;
}

string BaseShaderProgram::getProgramInfo(GLuint program) {
    GLint infoLength = 0;
    glGetProgramiv(program, GL_INFO_LOG_LENGTH, &infoLength);

    string info;
    info.resize(infoLength);

    glGetProgramInfoLog(program, infoLength, &infoLength, &info[0]);

    return info;
}
