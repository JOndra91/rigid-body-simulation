#pragma once

#include <GL/gl.h>
#include <string>

namespace gmu {

    using std::string;

    class BaseShaderProgram {
    private:
        GLuint program;
    public:
        BaseShaderProgram();

        ~BaseShaderProgram() {
            deleteProgram();
        }
        GLuint getProgram();

    protected:
        virtual GLuint compile() = 0;

        inline void deleteProgram() {
            if (program) {
                glDeleteProgram(program);
                program = 0;
            }
        }

        inline void deleteShader(GLuint &shader) {

            deleteProgram();

            if (shader) {
                glDeleteShader(shader);
                shader = 0;
            }
        }

        static string getFileContents(string &filename);
        static GLuint createShaderFromSource(string &source, GLenum shaderType);
        static GLuint createShaderFromFile(string &filename, GLenum shaderType);
        static string getShaderInfo(GLuint shader);
        static string getProgramInfo(GLuint program);
    };

}