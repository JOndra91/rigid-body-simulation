#pragma once

#define S_OK 0
#define S_SDL_ERROR 1

#include <SDL2/SDL.h>

#include "RegistrablesContainer.hpp"
#include "Camera.hpp"
#include "Scene.hpp"

#include <q3.h>

namespace gmu {

    class Main : public RegistrablesContainer, public IEventListener {
    protected:
        SDL_Window *sdlWindow;
        SDL_GLContext context;
        bool quitFlag = false;
        Camera *camera;
        Scene *scene;
        q3OpenCLDevice clDev;
        bool renderingEnabled;

    public:
        Main();
        ~Main();
        void run();
        void init(bool renderingEnabled);
        void onQuit();
        void release();

        inline void setDevice(q3OpenCLDevice device) {
            clDev = device;
        };

        inline void quit() {
            quitFlag = true;
        };

        // Event listener interface
        virtual IEventListener::EventResponse onEvent(SDL_Event* evt);

    };
}

int main(int argc, char **argv);
