#ifndef PATH_PLANNING_TIMER_H
#define PATH_PLANNING_TIMER_H

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include "Clock.h"

namespace common {

struct TimerTask {
  std::function<void()> callback;
  int64_t execution_millis;
};

/**
 * Utility class for scheduling periodic task.
 */
class Timer {
 public:
  Timer() : shutdown_(false) {
    timer_thread_ = std::make_shared<std::thread>([this]() {
      while (true) {
        std::vector<TimerTask> tasks_to_execute;
        int num_tasks = 0;
        {
          std::lock_guard<std::recursive_mutex> guard(mutex_);
          std::vector<TimerTask> tasks_remaining;
          int64_t now_millis = Clock::NowMillis();
          for (auto &task : tasks_) {
            if (task.execution_millis <= now_millis) {
              tasks_to_execute.emplace_back(task);
            } else {
              tasks_remaining.emplace_back(task);
              ++num_tasks;
            }
          }
          tasks_.swap(tasks_remaining);
        }
        for (auto &task : tasks_to_execute) {
          task.callback();
        }
        if (shutdown_ && num_tasks == 0) {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    });
  }

  ~Timer() {
    Shutdown();
  }

  void ExecuteAtPeriodicInterval(std::function<void()> callback,
                                 std::chrono::milliseconds period) {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    std::function<void()> wrapped = [this, callback, period]() {
      callback();
      if (!shutdown_) {
        ExecuteAtPeriodicInterval(callback, period);
      }
    };
    TimerTask task;
    task.callback = wrapped;
    task.execution_millis = Clock::NowMillis() + period.count();
    tasks_.emplace_back(task);
  }

  void Shutdown() {
    shutdown_ = true;
    timer_thread_->join();
  }

 private:
  std::vector<TimerTask> tasks_;
  std::recursive_mutex mutex_;
  std::atomic<bool> shutdown_;
  std::shared_ptr<std::thread> timer_thread_;
};

}  // namespace common

#endif //PATH_PLANNING_TIMER_H
