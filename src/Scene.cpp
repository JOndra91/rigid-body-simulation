#include <climits>
#include <glm/glm.hpp>
#include <string>
#include <glm/gtc/matrix_transform.hpp>

#include "Scene.hpp"

#define LANDSCAPE_SIZE 350
#define LANDSCAPE_SIZEF float(LANDSCAPE_SIZE)
#define INDEX_COUNT (2 * (LANDSCAPE_SIZE + 1) * LANDSCAPE_SIZE + LANDSCAPE_SIZE)
#define BASE_FREQUENCY 100
#define RESOLUTION 0.85

using namespace gmu;

using glm::u8vec3;
using glm::vec3;

typedef struct {
    vec3 position;
    vec3 normal;
} Vertex;

Scene::Scene(Camera *_camera) : camera(_camera), vao(0), vbo(0), ebo(0), polygonMode(GL_FILL) {
    string vertexShaderFile("./shaders/shader.vert");
    string fragmentShaderFile("./shaders/shader.frag");

    registerRenderer(camera);

    renderProgram.setVertexShaderFromFile(vertexShaderFile);
    renderProgram.setFragmenShaderFromFile(fragmentShaderFile);

    GLuint program = renderProgram.getProgram();
    if (program == 0) {
        throw string("Could not compile render program.");
    }


    uProjection = glGetUniformLocation(program, "projection");
    uView = glGetUniformLocation(program, "view");
    uModel = glGetUniformLocation(program, "model");

    uColor = glGetUniformLocation(program, "color");

    uSunPosition = glGetUniformLocation(program, "sunPosition");
    uSunColor = glGetUniformLocation(program, "sunColor");

    aPosition = glGetAttribLocation(program, "position");
    aNormal = glGetAttribLocation(program, "normal");


    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    glEnableVertexAttribArray(aPosition);
    glVertexAttribPointer(aPosition, 3, GL_FLOAT, GL_FALSE, sizeof (Vertex), (GLvoid*) offsetof(Vertex, position));

    glEnableVertexAttribArray(aNormal);
    glVertexAttribPointer(aNormal, 3, GL_FLOAT, GL_FALSE, sizeof (Vertex), (GLvoid*) offsetof(Vertex, normal));

    glGenBuffers(1, &ebo);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);

    glBindVertexArray(0);
}

Scene::~Scene() {

    glDeleteBuffers(1, &vbo);
    glDeleteBuffers(1, &ebo);

    glDeleteVertexArrays(1, &vao);

}

mat4 Scene::getProjectionMatrix() {

      vec2 windowSize = camera->getWindowSize();
      float fov = radians(75.0);
      float aspect = windowSize.x / windowSize.y;
      float near = 0.001;
      float far = 1e5;

      return glm::perspective(fov, aspect, near, far);
}

mat4 Scene::getViewMatrix() {

      vec3 cameraPosition = camera->getPosition();
      vec3 atPosition = cameraPosition + camera->getViewVector();

      return glm::lookAt(cameraPosition, atPosition, vec3(0, 1, 0));
}

void Scene::render() {

    runRenderers();

    glUseProgram(renderProgram.getProgram());

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    glClearColor(0.0, 0.7, 0.8, 1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    mat4 viewMat = getViewMatrix();
    mat4 projMat = getProjectionMatrix();

    vec3 camPos = camera->getPosition();

    // std::cout << "Eye: (" << camPos.x << ", " << camPos.y << ", " << camPos.z << ")" << std::endl;
    // std::cout << "At: (" << atPosition.x << ", " << atPosition.y << ", " << atPosition.z << ")" << std::endl;

    glUniformMatrix4fv(uView, 1, GL_FALSE, (GLfloat*) & viewMat);
    glUniformMatrix4fv(uProjection, 1, GL_FALSE, (GLfloat*) & projMat);

    glBindVertexArray(vao);

    glPolygonMode(GL_FRONT_AND_BACK, polygonMode);

    glEnable(GL_PRIMITIVE_RESTART_FIXED_INDEX);
    glDrawElements(GL_TRIANGLE_STRIP, INDEX_COUNT, GL_UNSIGNED_INT, 0);
    glDisable(GL_PRIMITIVE_RESTART_FIXED_INDEX);

    glBindVertexArray(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void Scene::step(float time, float delta) {

}

IEventListener::EventResponse Scene::onEvent(SDL_Event* evt) {

    if (evt->type == SDL_KEYDOWN) {
        SDL_KeyboardEvent *e = &evt->key;

        if (e->keysym.sym == SDLK_p) {
            switch (polygonMode) {
                case GL_FILL:
                    polygonMode = GL_LINE;
                    break;
                case GL_LINE:
                    polygonMode = GL_POINT;
                    break;
                case GL_POINT:
                    polygonMode = GL_FILL;
                    break;
            }
            return EVT_PROCESSED;
        }
    }

    return EVT_IGNORED;
}
