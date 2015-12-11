#pragma once

#include <vector>

#include "IRegisterable.hpp"
#include "IEventListener.hpp"
#include "IRenderer.hpp"
#include "IProcessor.hpp"

namespace gmu {

    using std::vector;

    class RegistrablesContainer {
    protected:
        vector<IEventListener*> eventListenerList;
        vector<IRenderer*> rendererList;
        vector<IProcessor*> processorList;
    public:
        RegistrablesContainer();

        inline void registerEventListener(IEventListener *listener) {
            eventListenerList.push_back(listener);
        }

        inline void registerRenderer(IRenderer *renderer) {
            rendererList.push_back(renderer);
        }

        inline void registerProcessor(IProcessor *processor) {
            processorList.push_back(processor);
        }

        inline void runProcessors(float time, float delta) {
            for (IProcessor *processor : processorList) {
                processor->step(time, delta);
            }
        }

        inline void runRenderers() {
            for (IRenderer *renderer : rendererList) {
                renderer->render();
            }
        }

        void autoregister(IRegisterable *registerable);
    };

}
