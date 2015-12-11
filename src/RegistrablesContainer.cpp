#include "RegistrablesContainer.hpp"

using namespace gmu;

RegistrablesContainer::RegistrablesContainer() {
}

void RegistrablesContainer::autoregister(IRegisterable *registerable) {
    IEventListener *listener = dynamic_cast<IEventListener*> (registerable);
    if (listener) {
        registerEventListener(listener);
    }

    IProcessor *processor = dynamic_cast<IProcessor*> (registerable);
    if (processor) {
        registerProcessor(processor);
    }

    IRenderer *renderer = dynamic_cast<IRenderer*> (registerable);
    if (renderer) {
        registerRenderer(renderer);
    }
}

