#include "components/joystick_interface.h"
#include "components/igor.h"

#include "util/input/joystick.h"

namespace hebi {

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
static void arm_x_vel_event(Igor& igor, const Functor& in_deadzone, uint64_t ts, float axis_value) {
  if (in_deadzone(axis_value)) {
    igor.left_arm().set_x_velocity(0.0);
    igor.right_arm().set_x_velocity(0.0);
  } else {
    double velocity = -0.4*static_cast<double>(axis_value);
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
static void arm_y_vel_event(Igor& igor, const Functor& in_deadzone, uint64_t ts, float axis_value) {
  if (in_deadzone(axis_value)) {
    igor.left_arm().set_y_velocity(0.0);
    igor.right_arm().set_y_velocity(0.0);
  } else {
    double velocity = -0.4*static_cast<double>(axis_value);
    igor.left_arm().set_y_velocity(velocity);
    igor.right_arm().set_y_velocity(velocity);
  }
}

/**
 * Event handler when left trigger of joystick has its axis value changed
 * 
 * (Left Trigger Axis event handler)
 */
static void arm_z_vel_event_l(Igor& igor, uint64_t ts, float axis_value) {
  // Left Trigger
  double velocity = -0.1*(static_cast<double>(axis_value)+1.0);
  set_arm_vel_z(igor.left_arm(), velocity);
  set_arm_vel_z(igor.right_arm(), velocity);
}

/**
 * Event handler when right trigger of joystick has its axis value changed
 * 
 * (Right Trigger Axis event handler)
 */
static void arm_z_vel_event_r(Igor& igor, uint64_t ts, float axis_value) {
  // Right Trigger
  double velocity = 0.1*(static_cast<double>(axis_value)+1.0);
  set_arm_vel_z(igor.left_arm(), velocity);
  set_arm_vel_z(igor.right_arm(), velocity);
}

/**
 * Event handler when left or right trigger of joystick is pressed or released.
 * This event handler will zero the Z axis velocity of the arms, if both joysticks
 * are not being pressed. If this condition is not satisfied, then this function
 * does nothing.
 */
template <typename Functor>
static void zero_arm_z_event(Igor& igor, const Functor& both_triggers_released, uint64_t ts, bool pressed) {
  if (both_triggers_released()) {
    igor.left_arm().set_z_velocity(0.0);
    igor.right_arm().set_z_velocity(0.0);
  }
}

/**
 * Event handler when d-pad is pressed.
 * 
 * This event handler will set the velocity of the wrists.
 */
static void wrist_vel_event(Igor& igor, uint64_t ts, int hx, int hy) {
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
static void chassis_velocity_event(Igor& igor, const Functor& in_deadzone, uint64_t ts, float axis_value) {
  if (in_deadzone(axis_value)) {
    igor.chassis().set_directional_velocity(0.0);
  } else {
    const double dead_zone = igor.joystick_dead_zone();
    double velocity = 0.5*(axis_value-(dead_zone*math_utils.sign(axis_value)));
    igor.chassis().set_directional_velocity(velocity);
  }
}

/**
 * Event handler when right stick X-Axis motion occurs.
 */
template <typename Functor>
static void chassis_yaw_event(Igor& igor, const Functor& in_deadzone, uint64_t ts, float axis_value) {
  if (in_deadzone(axis_value)) {
    igor.chassis().set_yaw_velocity(0.0);
  } else {
    const double scale = 25.0;
    const double dead_zone = igor.joystick_dead_zone();
    const double wheel_radius = igor.wheel_radius();
    const double wheel_base = igor.wheel_base();
    double velocity = (scale * wheel_radius / wheel_base) *
                      (axis_value - dead_zone * math_utils.sign(axis_value));
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
static void stance_height_triggers_event(Igor& igor, util::Joystick& joy, const Functor& vel_calc, uint64_t ts, float axis_value) {
  // Ignore this if `OPTIONS` is pressed
  if (joy.get_button('OPTIONS'))
    return;

  auto velocity = vel_calc();
  igor.left_leg().set_knee_velocity(velocity);
  igor.right_leg().set_knee_velocity(velocity);
}

/**
 * Event handler when `OPTIONS` button is pressed or released.
 */
template <typename Functor>
static void stance_height_event(Igor& igor, const Functor& vel_calc, uint64_t ts, bool pressed) {
  if (pressed) {
    igor.left_leg().set_knee_velocity(1.0);
    igor.right_leg().set_knee_velocity(1.0);
    if (igor.left_leg().knee_angle() > 2.5) {
      // TODO: implement restart
      //igor.request_restart()
      //igor.request_stop()
    }
  } else {
    auto velocity = vel_calc();
    igor.left_leg().set_knee_velocity(velocity);
    igor.right_leg().set_knee_velocity(velocity);
  }
}

/**
 * Event handler when `TOUCHPAD` button is pressed or released.
 */
static void balance_controller_event(Igor& igor, uint64_t ts, bool pressed) {
  igor.set_balance_controller_state(!pressed);
}

/**
 * Event handler when `SHARE` button is pressed or released.
 * When the button is pressed, and the robot is ready (in the "started" state),
 * the session will begin to quit.
 */
static void quit_session_event(Igor& igor, uint64_t ts, bool pressed) { 
  if (pressed && igor.started())
    igor.request_stop();
}

//------------------------------------------------------------------------------
// Helper Functions
//------------------------------------------------------------------------------

/**
 * Set the Z axis velocity of the given arm.
 * 
 * If the velocity is less than 1e-4, this function does nothing.
 */
static void set_arm_vel_z(Arm& arm, float val) {
  // Ignore small values from being set (noise)
  if (std::abs(val) > 1e-4f) {
    arm.set_z_velocity(static_cast<double>(val));
  }
}

/**
 * @return true if both the left and right triggers are not being pressed
 */
static bool both_triggers_released(Joystick& joy) {
  bool left = joy.get_button('LEFT_TRIGGER');
  bool right = joy.get_button('RIGHT_TRIGGER');
  return !(left || right);
}

//------------------------------------------------------------------------------
// Functors
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// Public API
//------------------------------------------------------------------------------

void register_igor_event_handlers(Igor& igor) {
  // TODO
}

}
