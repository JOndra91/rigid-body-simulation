#pragma once

#include <SDL2/SDL.h>
#include "IRegisterable.hpp"

namespace gmu {

    class IEventListener : virtual public IRegisterable {
    public:

        enum EventResponse {
            EVT_IGNORED,
            EVT_PROCESSED,
            EVT_DROPPED
        };

        virtual EventResponse onEvent(SDL_Event *evt) = 0;

    };

}
