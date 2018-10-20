#ifndef PATH_PLANNING_CLOCK_H
#define PATH_PLANNING_CLOCK_H

#include <chrono>

namespace common {

/**
 * Utility class to get echo in milli seconds.
 */
class Clock {
 public:
  static int64_t NowMillis() {
    auto epoch = std::chrono::high_resolution_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::milliseconds>(epoch).count();
  }
};

}  // namespace common


#endif //PATH_PLANNING_CLOCK_H
