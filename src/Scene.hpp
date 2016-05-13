#pragma once

#include "Camera.hpp"
#include "IRenderer.hpp"
#include "IProcessor.hpp"
#include "RenderShaderProgram.hpp"
#include "RegistrablesContainer.hpp"

#include <vector>

#include <q3.h>

namespace gmu {

    using glm::vec3;
    using glm::mat4;

    class Scene : public IRenderer, public IProcessor, public IEventListener, public RegistrablesContainer {
    private:
        struct Vertex {
            vec3 position;
            vec3 normal;
            u8vec3 color;
        };

        const static u8vec3 colors[];

        Camera *camera;
        RenderShaderProgram renderProgram;
        RenderShaderProgram renderDebugProgram;
        q3Scene scene;
        std::vector<const q3BoxRef*> boxes;
        GLuint vao, vbo, ebo;
        GLint uView, uProjection;
        GLint aPosition, aNormal, aColor;
        GLint uSunPosition, uSunColor;
        GLenum polygonMode;

        void prepareScene();
        static void prepareBuffers(unsigned &index, const q3BoxRef *b, Vertex *vert, GLuint *elem);
    public:
        int loop;
        bool pause;
        bool debug;
        Scene(q3OpenCLDevice device);
        Scene(Camera *camera, q3OpenCLDevice device);

        ~Scene();

        mat4 getProjectionMatrix();
        mat4 getViewMatrix();

        virtual void render();

        virtual void step(float time, float delta);

        virtual IEventListener::EventResponse onEvent(SDL_Event* evt);

    };

}
