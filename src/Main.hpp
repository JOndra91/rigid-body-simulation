#pragma once

#define S_OK 0
#define S_SDL_ERROR 1

#include <SDL.h>

#include "RegistrablesContainer.hpp"
#include "Camera.hpp"

namespace gmu {

    class Main : public RegistrablesContainer, public IEventListener {
    protected:
        SDL_Window *sdlWindow;
        SDL_GLContext context;
        bool quitFlag = false;
        Camera *camera;

    public:
        Main();
        ~Main();
        void run();
        void init();
        void onQuit();
        void release();

        inline void quit() {
            quitFlag = true;
        };

        // Event listener interface
        virtual IEventListener::EventResponse onEvent(SDL_Event* evt);

    };
}

int main(int argc, char **argv);
