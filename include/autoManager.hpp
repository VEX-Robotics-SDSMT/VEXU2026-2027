#pragma once

#include <functional>
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
  void initialize();
  void periodic();
  void reset();
  bool isFinished();

 private:
  size_t idx;
  std::vector<AutoCommand> m_commands;
};
};  // namespace AutoManager