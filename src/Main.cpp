#include <iostream>
#include <csignal>

#include "Main.hpp"
#include "Exceptions.hpp"

using namespace std;
using namespace gmu;

Main program;

void sigintHandler(int signal) {
    program.quit();
}

int main(int argc, char **argv) {
    signal(SIGINT, sigintHandler);
    bool render = true;

    for(int i = 1; i < argc; i++) {
        string arg(argv[i]);

        if(arg == "--none") {
            program.setDevice(q3OpenCLDevice::NONE);
        } else if(arg == "--cpu") {
            program.setDevice(q3OpenCLDevice::CPU);
        } else if(arg == "--gpu") {
            program.setDevice(q3OpenCLDevice::GPU);
        }
        else if(arg == "--no-render") {
            render = false;
        }
        else {
            fprintf(stderr, "%s: Invalid argument\n", arg.c_str());
            exit(1);
        }
    }

    try {
        program.init(render);

        program.run();
    } catch (string &str) {
        cerr << str << endl;
        return 2;
    } catch (Exception &e) {
        cerr << "Exception: " << e.getMessage() << endl;
        return 1;
    }

    return 0;
}

void glDebugCallback(GLenum source, GLenum type, GLuint id, GLenum severity, GLsizei length, const GLchar * message, void * userParam) {
    bool throwE;
    cerr << "Debug: " << message << endl;

    throwE = !((type == GL_DEBUG_TYPE_PERFORMANCE) || (severity == GL_DEBUG_SEVERITY_NOTIFICATION));

    if (throwE) {
        throw GLException(string(message));
    }
}

Main::Main() : sdlWindow(NULL), context(NULL), camera(NULL), scene(NULL), clDev(q3OpenCLDevice::NONE){
}

Main::~Main() {
    release();
}

void Main::run() {
    uint32_t ticks = SDL_GetTicks();
    uint32_t initTicks = ticks;
    uint32_t lastFrameTicks = ticks;
    float dt = 1e-10;
    float t = 0.0;

    #ifdef FPS_CTR
        float ft = t;
        unsigned int frameCounter = 0;
    #endif

    while (!quitFlag) {
        SDL_Event evt;
        while (SDL_PollEvent(&evt)) {
            int processed = 0;
            IEventListener::EventResponse r;
            for (IEventListener *listener : eventListenerList) {
                r = listener->onEvent(&evt);

                switch (r) {
                    case IEventListener::EVT_IGNORED:
                        break;
                    case IEventListener::EVT_PROCESSED:
                        processed++;
                        break;
                    case IEventListener::EVT_DROPPED:
                        goto drop;
                }
            }
drop:
            if (quitFlag) {
                goto quit;
            }
        }

        runProcessors(t, dt);

        runRenderers();

        if(renderingEnabled) {
            glFinish();
            SDL_GL_SwapWindow(sdlWindow);

            ticks = SDL_GetTicks();
            // cout << "Ticks: " << ticks << endl;
            t = (ticks - initTicks)/1000.0f;
            dt = (ticks - lastFrameTicks)/1000.0f;
            // cout << "DT: " << dt << endl;
            lastFrameTicks = ticks;

            #ifdef FPS_CTR
            frameCounter++;
            if(frameCounter > 60) {
                cout << "FPS: " << (frameCounter/(t-ft)) << endl;
                frameCounter = 0;
                ft = t;
            }
            #endif
        }

       if(scene->loop > 500) {
         quit();
       }

    }
quit:
    onQuit();
}

void Main::init(bool _renderingEnabled) {

    renderingEnabled = _renderingEnabled;

    if (SDL_Init(SDL_INIT_EVENTS | SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        throw string("SDL_Init failed.");
    }

    sdlWindow = NULL;

    if(renderingEnabled) {

        sdlWindow = SDL_CreateWindow(
                "Rigid body simulation on GPU (sort of) Stable",
                SDL_WINDOWPOS_UNDEFINED,
                SDL_WINDOWPOS_UNDEFINED,
                1200,
                800,
                SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE
                );

        if (sdlWindow == NULL) {
            throw string(SDL_GetError());
        }

        context = SDL_GL_CreateContext(sdlWindow);

        glewInit();

        glEnable(GL_DEBUG_OUTPUT);
        glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

        glDebugMessageControl(GL_DONT_CARE, GL_DONT_CARE, GL_DEBUG_SEVERITY_NOTIFICATION, 0, NULL, GL_FALSE);
        glDebugMessageControl(GL_DONT_CARE, GL_DEBUG_TYPE_PERFORMANCE, GL_DONT_CARE, 0, NULL, GL_FALSE);

        glDebugMessageCallback((GLDEBUGPROC) glDebugCallback, NULL);

    }

    if(renderingEnabled) {
        camera = new Camera(sdlWindow);
        scene = new Scene(camera, clDev);

        registerEventListener(this);

        registerEventListener(camera);
        registerProcessor(camera);

        autoregister(scene);
    }
    else {
        camera = new Camera();
        scene = new Scene(clDev);

        registerProcessor(scene);
    }
}

void Main::onQuit() {
  release();
}

void Main::release() {
    delete camera;

    camera = NULL;

    if(renderingEnabled) {
        SDL_DestroyWindow(sdlWindow);
        sdlWindow = NULL;

        SDL_GL_DeleteContext(context);
        context = NULL;

        SDL_Quit();
    }
}

// Event listener interface implementation

IEventListener::EventResponse Main::onEvent(SDL_Event* evt) {
    if (evt->type == SDL_WINDOWEVENT) {
        SDL_WindowEvent *e = &evt->window;

        if (e->event == SDL_WINDOWEVENT_CLOSE) {
            quit();
            return EVT_DROPPED;
        }
    } else if (evt->type == SDL_KEYDOWN) {
        SDL_KeyboardEvent *e = &evt->key;

        if (e->keysym.sym == SDLK_ESCAPE) {
            quit();
            return EVT_DROPPED;
        }
    }

    return EVT_IGNORED;
}
