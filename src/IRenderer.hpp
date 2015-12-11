#pragma once

#include "IRegisterable.hpp"

namespace gmu {

    class IRenderer : virtual public IRegisterable {
    public:
        virtual void render() = 0;
    };
}
