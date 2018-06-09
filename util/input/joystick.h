#pragma once

#include <SDL.h>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace hebi {
namespace util {

struct HatValue {
  int8_t x, y;
};

template <typename ValueT>
class JoystickElement {

private:

  ValueT value_;
  uint32_t timestamp_;
  std::string name_;
  std::vector<EventHandler> callbacks_;
  std::mutex val_lock_;
  std::condition_variable cv_;

  void wait_for_next(uint32_t timeout_ms=0) {
    // TODO: do timeout
    std::unique_lock<std::mutex> lock(val_lock_);
    cv_.wait(val_lock_);
  }

public:

  using EventHandler = std::function<void(uint32_t, ValueT)>;

  JoystickElement() = delete;
  JoystickElement(const std::string& name="") : name_(name) {
    ValueT val{};
    update(0, val);
  }

  void update(uint32_t ts, ValueT value) {
    std::unique_lock<std::mutex> lock(val_lock_);

    value_ = value;
    timestamp_ = ts;

    for (auto& callback : callbacks_) {
      callback(ts, value);
    }

    cv_.notify_all();    
  }

  void set_name(const std::string& name) {
    name_ = name;
  }

  const std::string name() const {
    return name_;
  }

  ValueT get() const {
    return value_;
  }

  uint32_t timestamp() const {
    return timestamp_;
  }

  ValueT get_next(uint32_t timeout_ms=0) {
    wait_for_next(timeout_ms);
    return value_;
  }

  ValueT get_next(uint32_t& timestamp, uint32_t timeout_ms=0) {
    wait_for_next(timeout_ms);
    timestamp = timestamp_;
    return value_;
  }

  void add_event_handler(EventHandler callback) {
    std::lock_guard<std::mutex> lock(val_lock_);
    callbacks_.push_back(callback);
  }

};

using AxisEventHandler = std::function<void(uint32_t, float)>;
using HatEventHandler = std::function<void(uint32_t, HatValue)>;
using ButtonEventHandler = std::function<void(uint32_t, bool)>;

class Joystick {

private:

  friend class JoystickDispatcher;

  uint32_t num_axes_;
  uint32_t num_hats_;
  uint32_t num_buttons_;
  size_t index_;
  std::string name_;
  std::string guid_;
  SDL_Joystick* joystick_
  SDL_GameController* game_controller_;

  std::vector<JoystickElement<float>> axis_events_;
  std::vector<JoystickElement<HatValue>> hat_events_;
  std::vector<JoystickElement<bool>> button_events_;

  Joystick(SDL_Joystick* joystick, SDL_GameController* game_controller);

  static void set_at(size_t index, SDL_Joystick* joystick, SDL_GameController* game_controller);

  void on_axis_event(uint32_t ts, size_t axis, float value);
  void on_hat_event(uint32_t ts, size_t hat, HatValue value);
  void on_button_event(uint32_t ts, size_t axis, bool value);

public:

//------------------------------------------------------------------------------

  Joystick() = delete;
  Joystick(const Joystick&) = delete;
  Joystick(Joystick&&) = delete;
  ~Joystick();

//------------------------------------------------------------------------------

  static size_t joystick_count();
  static std::shared_ptr<Joystick> at_index(size_t index);
  static std::vector<std::shared_ptr<Joystick>> available_joysticks();

//------------------------------------------------------------------------------

  std::string get_button_name(size_t button) const;
  std::string get_axis_name(size_t axis) const;
  std::string name() const;
  std::string guid() const;
  size_t index() const;

//------------------------------------------------------------------------------

  void add_axis_event_handler(size_t axis, AxisEventHandler handler);
  void add_hat_event_handler(size_t hat, HatEventHandler handler);
  void add_button_event_handler(size_t button, ButtonEventHandler handler);

//------------------------------------------------------------------------------

  float get_current_axis_state(size_t axis);
  HatValue get_current_hat_state(size_t hat);
  bool get_current_button_state(size_t button);

//------------------------------------------------------------------------------

  float get_next_axis_state(size_t axis, uint32_t timeout=0);
  HatValue get_next_hat_state(size_t hat, uint32_t timeout=0);
  bool get_next_button_state(size_t button, uint32_t timeout=0);

};


}
}