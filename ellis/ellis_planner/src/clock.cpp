// Copyright 2022 Ellis Ratner (eratner@berkeley.edu)
#include <ellis_planner/clock.h>

namespace ellis_planner {

Clock::Clock() : elapsed_time_sec_(0.0) {}

Clock::~Clock() {}

void Clock::Start() { start_time_ = std::chrono::high_resolution_clock::now(); }

void Clock::Stop() {
  auto end_time = std::chrono::high_resolution_clock::now();
  elapsed_time_sec_ = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count() / 1000.0;
}

double Clock::GetElapsedTimeSec() const { return elapsed_time_sec_; }

ScopedClock::ScopedClock(double* elapsed_time_sec_ptr, bool accumulate)
    : elapsed_time_sec_ptr_(elapsed_time_sec_ptr), accumulate_(accumulate) {
  Start();
}

ScopedClock::~ScopedClock() {
  Stop();
  if (elapsed_time_sec_ptr_) {
    if (accumulate_)
      *elapsed_time_sec_ptr_ += elapsed_time_sec_;
    else
      *elapsed_time_sec_ptr_ = elapsed_time_sec_;
  }
}

}  // namespace ellis_planner
