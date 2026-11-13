#pragma once

#include <functional>
#include <tuple>
#include <vector>

namespace AutoManager {
struct AutoCommand {
  std::function<void(void)> initialize;
  std::function<void(void)> periodic;
  std::function<bool(void)> isFinished;
};

class AutoManager {
 public:
  AutoManager(std::vector<AutoCommand> commands);
  void periodic();
  void reset();

 private:
  size_t idx;
  std::vector<AutoCommand> m_commands;
};
};  // namespace AutoManager