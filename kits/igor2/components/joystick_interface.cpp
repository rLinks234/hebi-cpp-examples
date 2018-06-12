#include "components/joystick_interface.h"
#include "components/igor.h"

#include "util/input/joystick.h"

#if !defined(NDEBUG)
  #define debug_printf(str, ...) printf("%s" str, __func__, __VA_ARGS__)
#else
  #define debug_printf(str, ...)
#endif

namespace hebi {

static float sign(float v);

const std::string LEFT_SHOULDER = "LEFT_SHOULDER";
const std::string RIGHT_SHOULDER = "RIGHT_SHOULDER";
const std::string LEFT_TRIGGER = "LEFT_TRIGGER";
const std::string RIGHT_TRIGGER = "RIGHT_TRIGGER";
const std::string OPTIONS = "OPTIONS";

//------------------------------------------------------------------------------
// Arm Event Handlers
//------------------------------------------------------------------------------

/**
 * Event handler when left stick Y-Axis motion occurs.
 * This event handler will set the given arm's velocity in the X axis to
 * a value proportional to the given input `axis_value` when outside the
 * joystick deadzone. When the value is within the joystick deadzone, the
 * arm's velocity in the X axis is set to zero.
 * 
 * (Left Stick Y-Axis event handler)
 */
template<typename Functor>
static void arm_x_vel_event(Igor& igor, Functor const& in_deadzone, uint32_t ts, float axis_value) {
  if (std::isnan(axis_value)) {
    return;
  } else if (in_deadzone(axis_value)) {
    debug_printf("(axis_value: %.2f, deadzone: yes)\n", axis_value);
    igor.left_arm().set_x_velocity(0.0);
    igor.right_arm().set_x_velocity(0.0);
  } else {
    double velocity = -0.4*static_cast<double>(axis_value);
    debug_printf("(axis_value: %.2f, deadzone: no, velocity: %.2f)\n", axis_value, velocity);
    igor.left_arm().set_x_velocity(velocity);
    igor.right_arm().set_x_velocity(velocity);
  }
}

/**
 * Event handler when left stick X-Axis motion occurs.
 * 
 * This event handler will set the given arm's velocity in the Y axis to
 * a value proportional to the given input `axis_value` when outside the
 * joystick deadzone. When the value is within the joystick deadzone, the
 * arm's velocity in the Y axis is set to zero.
 * 
 * (Left Stick X-Axis event handler)
 */
template<typename Functor>
static void arm_y_vel_event(Igor& igor, Functor const& in_deadzone, uint32_t ts, float axis_value) {
  if (std::isnan(axis_value)) {
    return;
  } else if (in_deadzone(axis_value)) {
    debug_printf("(axis_value: %.2f, deadzone: yes)\n", axis_value);
    igor.left_arm().set_y_velocity(0.0);
    igor.right_arm().set_y_velocity(0.0);
  } else {
    double velocity = -0.4*static_cast<double>(axis_value);
    debug_printf("(axis_value: %.2f, deadzone: no, velocity: %.2f)\n", axis_value, velocity);
    igor.left_arm().set_y_velocity(velocity);
    igor.right_arm().set_y_velocity(velocity);
  }
}

/**
 * Event handler when left or right trigger of joystick is pressed or released.
 * This event handler will zero the Z axis velocity of the arms, if both joysticks
 * are not being pressed. If this condition is not satisfied, then this function
 * does nothing.
 */
static void zero_arm_z_event(Igor& igor, uint32_t ts, bool pressed) {
  auto joy = igor.joystick();

  bool left = joy->get_current_button_state(LEFT_SHOULDER);
  bool right = joy->get_current_button_state(RIGHT_SHOULDER);

  if (!(left xor right)) /* both released or both pressed */ {
    igor.left_arm().set_z_velocity(0.0);
    igor.right_arm().set_z_velocity(0.0);
  } else if (right) {
    igor.left_arm().set_z_velocity(0.2);
    igor.right_arm().set_z_velocity(0.2);
  } else if (left) {
    igor.left_arm().set_z_velocity(-0.2);
    igor.right_arm().set_z_velocity(-0.2);
  }
}

/**
 * Event handler when d-pad is pressed.
 * 
 * This event handler will set the velocity of the wrists.
 */
static void wrist_vel_event(Igor& igor, uint32_t ts, int hx, int hy) {
  if (hy == 0) {
    igor.left_arm().set_wrist_velocity(0.0);
    igor.right_arm().set_wrist_velocity(0.0);
  } else {
    const double joy_low_pass = 0.95;
    const double wrist_vel = igor.left_arm().user_commanded_wrist_velocity();
    const auto hat_dir = static_cast<double>(hy);
    double velocity = (joy_low_pass*wrist_vel)+hat_dir*(1.0-joy_low_pass)*0.25;
    igor.left_arm().set_wrist_velocity(velocity);
    igor.right_arm().set_wrist_velocity(velocity);
  }
}

//------------------------------------------------------------------------------
// Chassis Event Handlers
//------------------------------------------------------------------------------

/**
 * Event handler when right stick Y-Axis motion occurs.
 */
template <typename Functor>
static void chassis_velocity_event(Igor& igor, Functor const& in_deadzone, uint32_t ts, float axis_value) {
  if (std::isnan(axis_value)) {
    return;
  } else if (in_deadzone(axis_value)) {
    debug_printf("(axis_value: %.2f, deadzone: yes)\n", axis_value);
    igor.chassis().set_directional_velocity(0.0);
  } else {
    const double dead_zone = igor.joystick_dead_zone();
    double velocity = -0.5*(axis_value-(dead_zone*sign(axis_value)));
    debug_printf("(axis_value: %.2f, deadzone: no, velocity: %.2f)\n", axis_value, velocity);
    igor.chassis().set_directional_velocity(velocity);
  }
}

/**
 * Event handler when right stick X-Axis motion occurs.
 */
template <typename Functor>
static void chassis_yaw_event(Igor& igor, Functor const& in_deadzone, uint32_t ts, float axis_value) {
  if (std::isnan(axis_value)) {
    return;
  } else if (in_deadzone(axis_value)) {
    debug_printf("(axis_value: %.2f, deadzone: yes)\n", axis_value);
    igor.chassis().set_yaw_velocity(0.0);
  } else {
    const double scale = 25.0;
    const double dead_zone = igor.joystick_dead_zone();
    const double wheel_radius = igor.wheel_radius();
    const double wheel_base = igor.wheel_base();
    double velocity = (scale * wheel_radius / wheel_base) *
                      (axis_value - (dead_zone * sign(axis_value)));
    debug_printf("(axis_value: %.2f, deadzone: no, velocity: %.2f)\n", axis_value, velocity);
    igor.chassis().set_yaw_velocity(velocity);
  }
}

//------------------------------------------------------------------------------
// Misc Event Handlers
//------------------------------------------------------------------------------

/**
 * 
 */
template <typename Functor>
static void stance_height_triggers_event(Igor& igor, Functor const& vel_calc, uint32_t ts, float axis_value) {
  // Ignore this if `OPTIONS` is pressed
  auto joy = igor.joystick();
  if (joy->get_current_button_state(OPTIONS))
    return;

  auto velocity = vel_calc(joy);
  debug_printf("(axis_value: %.2f, velocity: %.2f)\n", axis_value, velocity);
  igor.left_leg().set_knee_velocity(velocity);
  igor.right_leg().set_knee_velocity(velocity);
}

/**
 * Event handler when `OPTIONS` button is pressed or released.
 */
template <typename Functor>
static void stance_height_event(Igor& igor, Functor const& vel_calc, uint32_t ts, bool pressed) {
  if (pressed) {
    debug_printf("(pressed: yes, velocity: %.2f)\n", 1.0);
    igor.left_leg().set_knee_velocity(1.0);
    igor.right_leg().set_knee_velocity(1.0);
    if (igor.left_leg().knee_angle() > 2.5) {
      // TODO: implement restart
      //igor.request_restart()
      //igor.request_stop()
    }
  } else {
    auto velocity = vel_calc(igor.joystick());
    debug_printf("(pressed: no, velocity: %.2f)\n", velocity);
    igor.left_leg().set_knee_velocity(velocity);
    igor.right_leg().set_knee_velocity(velocity);
  }
}

/**
 * Event handler when `TOUCHPAD` button is pressed or released.
 */
static void balance_controller_event(Igor& igor, uint32_t ts, bool pressed) {
  igor.set_balance_controller_state(!pressed);
}

/**
 * Event handler when `SHARE` button is pressed or released.
 * When the button is pressed, and the robot is ready (in the "started" state),
 * the session will begin to quit.
 */
static void quit_session_event(Igor& igor, uint32_t ts, bool pressed) {
  if (pressed && igor.started())
    igor.request_stop();
}

//------------------------------------------------------------------------------
// Helper Functions
//------------------------------------------------------------------------------

static float sign(float v) {
  if (v < -0.0f) {
    return -1.0f;
  } else if (v > 0.0f) {
    return 1.0f;
  }
  return 0.0f;
}

//------------------------------------------------------------------------------
// Functors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------------

void register_igor_event_handlers(Igor& igor_) {

  Igor* igor = &igor_;
  auto joystick = igor->joystick();

  // ----------------------------------------------
  // Functions to be passed to event handlers below

  auto arm_x_deadzone = [=] (float val) -> bool {
    return static_cast<double>(std::abs(val)) <= igor->joystick_dead_zone()*3.0; };

  auto arm_y_deadzone = [=] (float val) -> bool {
    return static_cast<double>(std::abs(val)) <= igor->joystick_dead_zone(); };

  auto stance_height_calc = [=] (std::shared_ptr<util::Joystick> const& joystick) -> double {
    auto l_val = static_cast<double>(joystick->get_current_axis_state(LEFT_TRIGGER));
    auto r_val = static_cast<double>(joystick->get_current_axis_state(RIGHT_TRIGGER));
    double d_ax = l_val-r_val;
    if (std::abs(d_ax) > igor->joystick_dead_zone())
      return 0.5*d_ax;
    return 0.0; };

  //  The current joystick used is not a global state, so we need to wrap it here

  // ----------------------------------------------------------------------
  // Functions which have bound parameters, in order to have right function
  // signature for event handlers

  // ------------
  // Quit Session

  auto quit_session = [=] (uint32_t ts, bool pressed) {
    quit_session_event(*igor, ts, pressed); };
  joystick->add_button_event_handler(std::string("SHARE"), quit_session);

  // ---------------
  // Toggle Balancer

  auto balance_controller = [=] (uint32_t ts, bool pressed) {
    balance_controller_event(*igor, ts, pressed); };
  joystick->add_button_event_handler(std::string("TOUCHPAD"), balance_controller);

  // -----------------------
  // Left Arm event handlers

  // Reacts to left stick Y-axis
  auto arm_x_vel = [=] (uint32_t ts, float value) {
    arm_x_vel_event(*igor, arm_x_deadzone, ts, value); };
  joystick->add_axis_event_handler(std::string("LEFT_STICK_Y"), arm_x_vel);

  // Reacts to left stick X-axis
  auto arm_y_vel = [=] (uint32_t ts, float value) {
    arm_y_vel_event(*igor, arm_y_deadzone, ts, value); };
  joystick->add_axis_event_handler(std::string("LEFT_STICK_X"), arm_y_vel);

  // ------------------------
  // Both Arms event handlers

  // Reacts to triggers pressed/released
  auto zero_arm_z = [=] (uint32_t ts, bool value) {
    zero_arm_z_event(*igor, ts, value); };
  joystick->add_button_event_handler(std::string("LEFT_SHOULDER"), zero_arm_z);
  joystick->add_button_event_handler(std::string("RIGHT_SHOULDER"), zero_arm_z);

  // Reacts to D-Pad pressed/released
  //auto wrist_vel = funpart(wrist_vel_event, igor)
  //joystick.add_dpad_event_handler(wrist_vel)

  // ----------------------
  // Chassis event handlers

  // Reacts to right stick Y-axis
  auto chassis_velocity = [=] (uint32_t ts, float value) {
    chassis_velocity_event(*igor, arm_y_deadzone, ts, value); };
  joystick->add_axis_event_handler(std::string("RIGHT_STICK_Y"), chassis_velocity);

  // Reacts to right stick X-axis
  auto chassis_yaw = [=] (uint32_t ts, float value) {
    chassis_yaw_event(*igor, arm_y_deadzone, ts, value); };
  joystick->add_axis_event_handler(std::string("RIGHT_STICK_X"), chassis_yaw);

  // -------------
  // Stance height

  auto stance_height_trigger = [=] (uint32_t ts, float value) {
    stance_height_triggers_event(*igor, stance_height_calc, ts, value); };
  joystick->add_axis_event_handler(LEFT_TRIGGER, stance_height_trigger);
  joystick->add_axis_event_handler(RIGHT_TRIGGER, stance_height_trigger);

  auto stance_height = [=] (uint32_t ts, bool value) {
    stance_height_event(*igor, stance_height_calc, ts, value); };
  joystick->add_button_event_handler(std::string("OPTIONS"), stance_height);
}

}
