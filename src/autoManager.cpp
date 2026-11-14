#include "autoManager.hpp"

namespace AutoManager {
AutoManager::AutoManager(std::vector<AutoCommand> commands)
    : m_commands(commands), idx(0) {}

void AutoManager::reset() { idx = 0; }

void AutoManager::initialize() { m_commands[0].initialize(); }

void AutoManager::periodic() {
  if (idx < m_commands.size()) {
    m_commands[idx].periodic();
    if (m_commands[idx].isFinished()) {
      idx++;
      if (idx < m_commands.size()) {
        m_commands[idx].initialize();
      }
    }
  }
}

bool AutoManager::isFinished() { return idx >= m_commands.size(); }
}  // namespace AutoManager
