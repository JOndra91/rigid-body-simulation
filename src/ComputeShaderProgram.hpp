#pragma once

#include <GL/glew.h>

#include "BaseShaderProgram.hpp"

namespace gmu {

    class ComputeShaderProgram : public BaseShaderProgram {
    protected:
        GLuint computeShader;
    public:

        ComputeShaderProgram() : computeShader(0) {
        }

        virtual ~ComputeShaderProgram() {
            deleteShader(computeShader);
        }

        inline void setComputeShaderFromFile(string &filename) {
            string source = getFileContents(filename);
            setComputeShaderFromSource(source);
        }

        inline void setComputeShaderFromSource(string &source) {
            deleteShader(computeShader);
            computeShader = createShaderFromSource(source, GL_COMPUTE_SHADER);
        }
    private:

        virtual GLuint compile();

    };

}


