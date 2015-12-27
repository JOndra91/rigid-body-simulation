#include "Camera.hpp"

#include <glm/ext.hpp>
#include <GL/glew.h>

#define PI_HALF_CLAMP (1.57079632679f - 0.0001f)

using namespace gmu;
using namespace glm;

Camera::Camera(SDL_Window *_window) : position(0.0, 20.0, -25.0), rotation(0, 0) {
    window = _window;
    SDL_GetWindowSize(window, &(windowSize.x), &(windowSize.y));
}

vec3 Camera::getViewVector() {
    vec3 direction = vec3(0, 0, 1);

    direction = rotateX(direction, rotation.x);
    direction = rotateY(direction, rotation.y);

    return normalize(direction);
}

void Camera::setRotation(vec2 r) {
    rotation = r;
}

IEventListener::EventResponse Camera::onEvent(SDL_Event* evt) {
    if (evt->type == SDL_WINDOWEVENT) {
        SDL_WindowEvent *e = &evt->window;

        if (e->event == SDL_WINDOWEVENT_RESIZED
                || e->event == SDL_WINDOWEVENT_SIZE_CHANGED) {

            windowSize.x = e->data1;
            windowSize.y = e->data2;

            return EVT_PROCESSED;
        }
    } else if (evt->type == SDL_KEYDOWN || evt->type == SDL_KEYUP) {
        ivec3 positive, negative;
        SDL_KeyboardEvent *e = &evt->key;

        positive = max(ivec3(0, 0, 0), movement);
        negative = min(ivec3(0, 0, 0), movement);

        int active = (evt->type == SDL_KEYDOWN) ? 1 : 0;

        switch (e->keysym.sym) {
            case SDLK_w:
                positive.z = active;
                break;
            case SDLK_s:
                negative.z = active;
                break;
            case SDLK_a:
                positive.x = active;
                break;
            case SDLK_d:
                negative.x = active;
                break;
            case SDLK_LSHIFT:
                positive.y = active;
                break;
            case SDLK_LCTRL:
                negative.y = active;
                break;
            default:
                return EVT_IGNORED;
        }

        movement = positive - negative;

        return EVT_PROCESSED;

    } else if (evt->type == SDL_MOUSEMOTION) {
        SDL_MouseMotionEvent *e = &evt->motion;

        if (SDL_BUTTON_LEFT & e->state) {
            // TODO: Adjust mouse sensitivity coeficient
            rotation.y += e->xrel * 0.002;
            rotation.x -= e->yrel * 0.002;

            // To prevent horizontal turn
            rotation.x = clamp(rotation.x, -PI_HALF_CLAMP, PI_HALF_CLAMP);
        }

        return EVT_PROCESSED;
    }
    return EVT_IGNORED;
}

void Camera::step(float time, float delta) {
    vec3 direction;
    if (movement.x == 0 && movement.z == 0) {
        direction = movement;
    } else {
        direction = vec3(movement.x, 0, movement.z);
        direction = rotateX(direction, rotation.x);
        direction = rotateY(direction, rotation.y);

        direction.y += movement.y;

        direction = normalize(direction);
    }

    // TODO: Add speed coeficient
    position += direction * delta * 15.0f;
}

void Camera::render() {
    glViewport(0, 0, windowSize.x, windowSize.y);
}
