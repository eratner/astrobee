// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#ifndef ELLIS_PLANNER_CLOCK_H_
#define ELLIS_PLANNER_CLOCK_H_

#include <chrono>

namespace ellis_planner {

class Clock {
 public:
  Clock();

  virtual ~Clock();

  void Start();

  void Stop();

  double GetElapsedTimeSec() const;

 protected:
  std::chrono::time_point<std::chrono::high_resolution_clock> start_time_;
  double elapsed_time_sec_;
};

class ScopedClock : private Clock {
 public:
  explicit ScopedClock(double* elapsed_time_sec_ptr, bool accumulate = true);

  ~ScopedClock();

 private:
  double* elapsed_time_sec_ptr_;
  bool accumulate_;
};

}  // namespace ellis_planner

#endif  // ELLIS_PLANNER_CLOCK_H_
