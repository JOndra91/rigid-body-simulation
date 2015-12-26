#pragma once

#include "Camera.hpp"
#include "IRenderer.hpp"
#include "IProcessor.hpp"
#include "RenderShaderProgram.hpp"
#include "RegistrablesContainer.hpp"

namespace gmu {

    using glm::vec3;
    using glm::mat4;

    class Scene : public IRenderer, public IProcessor, public IEventListener, public RegistrablesContainer {
    private:
        Camera *camera;
        RenderShaderProgram renderProgram;
        GLuint vao, vbo, ebo;
        GLint uModel, uView, uProjection;
        GLint uColor;
        GLint aPosition, aNormal;
        GLint uSunPosition, uSunColor;
        GLenum polygonMode;
    public:
        Scene(Camera *camera);

        ~Scene();

        mat4 getProjectionMatrix();
        mat4 getViewMatrix();

        virtual void render();

        virtual void step(float time, float delta);

        virtual IEventListener::EventResponse onEvent(SDL_Event* evt);

    };

}
