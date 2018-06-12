#include "components/igor.h"
#include "util/input/event_handler.h"

#include <thread>

namespace hebi {

class IgorAccessor {

private:

  Igor* igor;

public:

  void print_state() {
    auto mass = igor->mass_;
    auto roll_angle = igor->roll_angle_;
    auto pitch_angle = igor->pitch_angle_;
    auto feedback_lean_angle = igor->feedback_lean_angle_;
    auto feedback_lean_angle_velocity = igor->feedback_lean_angle_velocity_;
    auto height_com = igor->height_com_;

    Eigen::Vector3d com = igor->com_;
    Eigen::Vector3d line_com = igor->line_com_;
    Eigen::Vector3d ground_point = igor->ground_point_;

    auto calculated_knee_velocity = igor->chassis().calculated_knee_velocity();

    printf(
        "mass: %.3f\n"
        "roll_angle: %.3f\n"
        "pitch_angle: %.3f\n"
        "feedback_lean_angle: %.3f\n"
        "feedback_lean_angle_velocity: %.3f\n"
        "height_com: %.3f\n"
        "com: %.3f, %.3f, %.3f\n"
        "line_com: %.3f, %.3f, %.3f\n"
        "ground_point: %.3f, %.3f, %.3f\n"
        "--- Chassis ---\n"
        "chassis.calculated_knee_velocity: %.3f\n"
        "\n"
        "=================================================\n\n",
        mass, roll_angle, pitch_angle,
        feedback_lean_angle, feedback_lean_angle_velocity,
        height_com,
        com[0], com[1], com[2],
        line_com[0], line_com[1], line_com[2],
        ground_point[0], ground_point[1], ground_point[2],
        calculated_knee_velocity);
  }

  explicit IgorAccessor(Igor* igor_) : igor(igor_) {}

};

}

static void igor_printer(hebi::Igor* igor) {
  hebi::IgorAccessor accessor(igor);

  while (true) {
    //puts("\\x1B[2J\\x1B[H");
    accessor.print_state();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}

int main(int argc, char** argv) {

  try {
    hebi::util::initialize_event_handler();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    using Igor = hebi::Igor;
    Igor* igor = new Igor();

    igor->start();

    std::thread printer_thd(igor_printer, igor);
    printer_thd.detach();

    igor->wait_for();
  } catch (...) {
    puts("Caught unknown exception");
  }

  return 0;
}
