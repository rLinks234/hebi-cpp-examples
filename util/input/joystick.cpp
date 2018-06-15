#include "util/input/joystick.h"

#include <cassert>
#include <condition_variable>
#include <mutex>

namespace hebi {
namespace util {
  
struct SDL2_InputSubsystemInitializer {
  SDL2_InputSubsystemInitializer() {
    SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
    SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
    SDL_JoystickEventState(SDL_ENABLE);
  }

  ~SDL2_InputSubsystemInitializer() {
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER | SDL_INIT_EVENTS);
  }
};

void load_joystick_system() {
  static SDL2_InputSubsystemInitializer loader;
}

static std::mutex sJoystickLock;
static std::vector<std::shared_ptr<Joystick>> sJoysticks;

//------------------------------------------------------------------------------

Joystick::Joystick(size_t index, SDL_Joystick* joystick,
    SDL_GameController* game_controller, ctor_key /*unused*/)
  : index_(index), joystick_(joystick), game_controller_(game_controller) {
  num_axes_ = SDL_JoystickNumAxes(joystick);
  num_hats_ = SDL_JoystickNumHats(joystick);
  num_buttons_ = SDL_JoystickNumButtons(joystick);

  axis_events_.reserve(num_axes_);
  hat_events_.reserve(num_hats_);
  button_events_.reserve(num_buttons_);

  for (size_t i = 0; i < num_axes_; i++) {
    axis_events_.emplace_back("");
  }
  for (size_t i = 0; i < num_hats_; i++) {
    hat_events_.emplace_back("");
  }
  for (size_t i = 0; i < num_buttons_; i++) {
    button_events_.emplace_back("");
  }

  name_ = SDL_GameControllerName(game_controller);

  {
    guid_.reserve(32);
    char buffer[33];
    SDL_JoystickGetGUIDString(SDL_JoystickGetGUID(joystick),
                              buffer, 33);
    guid_.append(buffer);
  }

}

Joystick::~Joystick() {
  if (joystick_) {
    SDL_JoystickClose(joystick_);
  }
  if (game_controller_) {
    SDL_GameControllerClose(game_controller_);
  }
}

// in joystick_mapper.cpp
void map_joystick(std::shared_ptr<Joystick>  const& joy);

void Joystick::set_at(size_t index, SDL_Joystick* joystick, SDL_GameController* game_controller) {
  std::lock_guard<std::mutex> lock(sJoystickLock);
  if (sJoysticks.size() < (index + 1)) {
    size_t insert_count = (index + 1) - sJoysticks.size();
    for (size_t i = 0; i < insert_count; i++) {
      sJoysticks.push_back(std::shared_ptr<Joystick>(nullptr));
    }
  }
  auto joy = std::make_shared<Joystick>(index, joystick, game_controller, ctor_key{});
  map_joystick(joy);
  sJoysticks[index] = joy;
}

void Joystick::add_axis_alias(const char* alias, size_t axis) {
  axis_aliases_[std::string(alias)] = axis;
}

void Joystick::add_button_alias(const char* alias, size_t button) {
  button_aliases_[std::string(alias)] = button;
}

//------------------------------------------------------------------------------

size_t Joystick::joystick_count() {
  return static_cast<size_t>(SDL_NumJoysticks());
}

std::shared_ptr<Joystick> Joystick::at_index(size_t index) {
  if (index >= Joystick::joystick_count()) {
    return std::shared_ptr<Joystick>(nullptr);
  }
  std::lock_guard<std::mutex> lock(sJoystickLock);
  return sJoysticks[index];
}

std::vector<std::shared_ptr<Joystick>> Joystick::available_joysticks() {
  std::vector<std::shared_ptr<Joystick>> ret;
  std::lock_guard<std::mutex> lock(sJoystickLock);

  ret.reserve(sJoysticks.size());
  for (const auto& joystick : sJoysticks) {
    ret.push_back(joystick);
  }
  return ret;
}

//------------------------------------------------------------------------------

std::string Joystick::get_button_name(size_t button) const {
  if (button >= num_buttons_) {
    // TODO: either throw exception, or return invalid name
    return "Invalid";
  }
  return button_events_[button].name();
}

std::string Joystick::get_axis_name(size_t axis) const {
  if (axis >= num_axes_) {
    // TODO: either throw exception, or return invalid name
    return "Invalid";
  }
  return axis_events_[axis].name();
}

std::string Joystick::name() const {
  return name_;
}

std::string Joystick::guid() const {
  return guid_;
}

size_t Joystick::index() const {
  return index_;
}

//------------------------------------------------------------------------------

void Joystick::on_axis_event(uint32_t ts, size_t axis, float value) {
  assert(axis < num_axes_);
  axis_events_[axis].update(ts, value);
}

void Joystick::on_hat_event(uint32_t ts, size_t hat, HatValue value) {
  assert(hat < num_hats_);
  hat_events_[hat].update(ts, value);
}

void Joystick::on_button_event(uint32_t ts, size_t button, bool value) {
  assert(button < num_buttons_);
  button_events_[button].update(ts, value);
}

//------------------------------------------------------------------------------

void Joystick::add_axis_event_handler(size_t axis, AxisEventHandler handler) {
  axis_events_[axis].add_event_handler(handler);
}

void Joystick::add_axis_event_handler(const std::string& axis, AxisEventHandler handler) {
  axis_events_[axis_aliases_.at(axis)].add_event_handler(handler);
}

void Joystick::add_hat_event_handler(size_t hat, HatEventHandler handler) {
  hat_events_[hat].add_event_handler(handler);
}

void Joystick::add_button_event_handler(size_t button, ButtonEventHandler handler) {
  button_events_[button].add_event_handler(handler);
}

void Joystick::add_button_event_handler(const std::string& button, ButtonEventHandler handler) {
  button_events_[button_aliases_.at(button)].add_event_handler(handler);
}

//------------------------------------------------------------------------------

float Joystick::get_current_axis_state(size_t axis) {
  return axis_events_[axis].get();
}

float Joystick::get_current_axis_state(const std::string& axis) {
  return get_current_axis_state(axis_aliases_.at(axis));
}

HatValue Joystick::get_current_hat_state(size_t hat) {
  return hat_events_[hat].get();
}

bool Joystick::get_current_button_state(size_t button) {
  return button_events_[button].get();
}

bool Joystick::get_current_button_state(const std::string& button) {
  return get_current_button_state(button_aliases_.at(button));
}


//------------------------------------------------------------------------------

float Joystick::get_next_axis_state(size_t axis, uint32_t timeout) {
  if (axis >= num_axes_) {
    // TODO: throw exception?
  }
  return axis_events_[axis].get_next(timeout);
}

HatValue Joystick::get_next_hat_state(size_t hat, uint32_t timeout) {
  if (hat >= num_hats_) {
    // TODO: throw exception?
  }
  return hat_events_[hat].get_next(timeout);
}

bool Joystick::get_next_button_state(size_t button, uint32_t timeout) {
  if (button >= num_buttons_) {
    // TODO: throw exception?
  }
  return button_events_[button].get_next(timeout);
}


}
}