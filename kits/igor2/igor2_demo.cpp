#include "components/igor.h"
#include "util/input/event_handler.h"

#include <thread>

int main(int argc, char** argv) {

  try {
    hebi::util::initialize_event_handler();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    using Igor = hebi::Igor;
    Igor* igor = new Igor();

    igor->start();

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    std::this_thread::sleep_for(std::chrono::milliseconds(50000));
  } catch (...) {
    puts("Caught unknown exception");
  }

  return 0;
}
