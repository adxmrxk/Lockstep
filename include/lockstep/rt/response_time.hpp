// rt/response_time.hpp : worst-case response time analysis.
//
// "It hasn't missed a deadline yet" is not a real-time guarantee, it is an
// absence of evidence. The guarantee comes from analysis: given each task's
// period, worst-case execution time and priority, compute the longest time any
// release can possibly take to finish, and compare it with its deadline.
//
// This is the standard fixed-priority recurrence (Joseph and Pandya 1986):
//
//     R_0     = C_i
//     R_{n+1} = C_i + SUM over hp(i) of ceil(R_n / T_j) * C_j
//
// Each iteration adds the interference from every higher-priority task that can
// be released inside the window computed so far. It is monotonically
// increasing, so it either reaches a fixed point -- the worst-case response
// time -- or passes the deadline, at which point the task set is not
// schedulable and no amount of testing will make it so.
//
// Assumptions, stated because they are what the answer depends on:
//   * single core, fixed priority, preemptive
//   * deadline <= period is allowed; the analysis handles constrained deadlines
//   * no blocking or shared resources. The bus is designed so this holds: the
//     publish path takes no lock, so a callback cannot be blocked by a lower
//     priority one holding something it needs. Add blocking terms here if that
//     ever stops being true.
//   * release jitter is not modelled.
#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ls::rt {

// All times in nanoseconds. Lower `priority` value means higher priority.
struct task_spec {
  std::uint64_t period_ns = 0;
  std::uint64_t deadline_ns = 0;  // relative deadline; 0 means "same as period"
  std::uint64_t wcet_ns = 0;      // worst-case execution time
  std::uint32_t priority = 0;
  const char* name = "";

  std::uint64_t effective_deadline() const noexcept {
    return deadline_ns != 0 ? deadline_ns : period_ns;
  }
};

struct task_analysis {
  const char* name = "";
  std::uint64_t response_ns = 0;
  std::uint64_t deadline_ns = 0;
  bool schedulable = false;
  bool converged = false;  // false if the recurrence ran past its deadline
};

struct analysis {
  std::vector<task_analysis> tasks;
  bool schedulable = false;
  double utilisation = 0.0;
};

namespace detail {
constexpr std::uint64_t ceil_div(std::uint64_t a, std::uint64_t b) noexcept {
  return (a + b - 1) / b;
}
}  // namespace detail

// Total processor utilisation. A necessary condition: above 1.0 nothing can
// save the task set. Not sufficient, which is why the recurrence exists.
inline double utilisation(const std::vector<task_spec>& tasks) noexcept {
  double u = 0.0;
  for (const auto& t : tasks) {
    if (t.period_ns == 0) continue;
    u += static_cast<double>(t.wcet_ns) / static_cast<double>(t.period_ns);
  }
  return u;
}

// Worst-case response time for one task under interference from all
// higher-priority ones.
inline task_analysis response_time(const task_spec& self,
                                   const std::vector<task_spec>& all) noexcept {
  task_analysis out;
  out.name = self.name;
  out.deadline_ns = self.effective_deadline();

  std::uint64_t r = self.wcet_ns;
  // The recurrence cannot need more iterations than there are release points
  // before the deadline; this bound simply stops a pathological input from
  // spinning.
  for (int guard = 0; guard < 10000; ++guard) {
    std::uint64_t next = self.wcet_ns;
    for (const auto& other : all) {
      if (&other == &self) continue;
      if (other.priority >= self.priority) continue;  // not higher priority
      if (other.period_ns == 0) continue;
      next += detail::ceil_div(r, other.period_ns) * other.wcet_ns;
    }

    if (next > out.deadline_ns) {
      // Already past the deadline, and the recurrence only grows.
      out.response_ns = next;
      out.schedulable = false;
      out.converged = false;
      return out;
    }
    if (next == r) {
      out.response_ns = r;
      out.schedulable = r <= out.deadline_ns;
      out.converged = true;
      return out;
    }
    r = next;
  }

  out.response_ns = r;
  out.schedulable = false;
  out.converged = false;
  return out;
}

inline analysis analyse(const std::vector<task_spec>& tasks) {
  analysis a;
  a.utilisation = utilisation(tasks);
  a.schedulable = true;
  a.tasks.reserve(tasks.size());
  for (const auto& t : tasks) {
    a.tasks.push_back(response_time(t, tasks));
    if (!a.tasks.back().schedulable) a.schedulable = false;
  }
  return a;
}

}  // namespace ls::rt
