#pragma once

#include <GL/glew.h>

#include "BaseShaderProgram.hpp"

namespace gmu {

    class RenderShaderProgram : public BaseShaderProgram {
    protected:
        GLuint vertexShader;
        GLuint fragmentShader;
    public:

        RenderShaderProgram() : vertexShader(0), fragmentShader(0) {
        }

        ~RenderShaderProgram() {
            deleteShader(vertexShader);
            deleteShader(fragmentShader);
        }

        inline void setVertexShaderFromFile(string &filename) {
            string source = getFileContents(filename);
            setVertexShaderFromSource(source);
        }

        inline void setFragmenShaderFromFile(string &filename) {
            string source = getFileContents(filename);
            setFragmentShaderFromSource(source);
        }

        inline void setVertexShaderFromSource(string &source) {
            deleteShader(vertexShader);
            vertexShader = createShaderFromSource(source, GL_VERTEX_SHADER);
        }

        inline void setFragmentShaderFromSource(string &source) {
            deleteShader(fragmentShader);
            fragmentShader = createShaderFromSource(source, GL_FRAGMENT_SHADER);
        }
    protected:
        virtual GLuint compile();
    };

}