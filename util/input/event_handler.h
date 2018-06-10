#pragma once

#include <SDL.h>

#include <functional>
#include <mutex>

namespace hebi {
namespace util {

using SDLEventCallback = std::function<void(const SDL_Event&)>;
  
void register_event(SDL_EventType event, SDLEventCallback callback);
void initialize_event_handler();

}
}
