#include "util/input/joystick.h"

namespace hebi {
namespace util {

struct JoystickMapping {
  std::vector<std::pair<std::string, size_t>> axis;
  std::vector<std::pair<std::string, size_t>> button;
};

static JoystickMapping sPS3 = {
  {
    {"LEFT_STICK_X", 0},
    {"LEFT_STICK_Y", 1},
    {"LEFT_TRIGGER", 2},
    {"RIGHT_STICK_X", 3},
    {"RIGHT_STICK_Y", 4},
    {"RIGHT_TRIGGER", 5}
  },
  {
    {"X", 0},
    {"CIRCLE", 1},
    {"TRIANGLE", 2},
    {"SQUARE", 3},
    {"LEFT_SHOULDER", 4},
    {"RIGHT_SHOULDER", 5},
    {"LEFT_TRIGGER", 6},
    {"RIGHT_TRIGGER", 7},
    {"SHARE", 8}, {"SELECT", 8},
    {"OPTIONS", 9}, {"START", 9},
    {"HOME", 10}, {"TOUCHPAD", 10},
    {"LEFT_STICK", 11},
    {"RIGHT_STICK", 12}
  }
};

static JoystickMapping sPS4 = {
  {
    {"LEFT_STICK_X", 0},
    {"LEFT_STICK_Y", 1},
    {"LEFT_TRIGGER", 2},
    {"RIGHT_STICK_X", 3},
    {"RIGHT_STICK_Y", 4},
    {"RIGHT_TRIGGER", 5}
  },
  {
    {"X", 0},
    {"CIRCLE", 1},
    {"TRIANGLE", 2},
    {"SQUARE", 3},
    {"LEFT_SHOULDER", 4},
    {"RIGHT_SHOULDER", 5},
    {"LEFT_TRIGGER", 6},
    {"RIGHT_TRIGGER", 7},
    {"SHARE", 8}, {"SELECT", 8},
    {"OPTIONS", 9}, {"START", 9},
    {"HOME", 10}, {"TOUCHPAD", 10},
    {"LEFT_STICK", 11},
    {"RIGHT_STICK", 12}
  }
};

static std::map<std::string, JoystickMapping*> controllers = {
    {"030000004c0500006802000000000000", &sPS3},
    {"030000004c0500006802000011810000", &sPS3},
    {"030000004c050000cc09000011010000", &sPS4},
    {"030000004c050000cc09000011810000", &sPS4},
    {"030000004c050000a00b000011810000", &sPS4}
};

class JoystickMapper {

public:

  static void map_joystick(std::shared_ptr<Joystick> joy) {
    const std::string guid = joy->guid();
    auto itr = controllers.find(guid);
    if (itr == controllers.end()) {
      printf("Could not find joystick mapping for %s (guid %s)\n",
        joy->name().c_str(), guid.c_str());
      return;
    }

    JoystickMapping* mapping = itr->second;

    for (auto& entry : mapping->axis) {
      joy->add_axis_alias(entry.first.c_str(), entry.second);
    }

    for (auto& entry : mapping->button) {
      joy->add_button_alias(entry.first.c_str(), entry.second);
    }
  }

};

void map_joystick(std::shared_ptr<Joystick> joy) {
  JoystickMapper::map_joystick(joy);
}

}
}