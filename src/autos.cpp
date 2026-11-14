#include "autos.h"

#include "autoManager.hpp"
#include "main.h"

AutoManager::AutoCommand driveVoltageTime(int32_t left, int32_t right,
                                          double time_s);

AutoManager::AutoCommand driveDirection(double distance_in,
                                        double directionDegs);

AutoManager::AutoCommand turnBy(double direction_degs);
AutoManager::AutoCommand waitFor(double time_s);

void quadrant1(Drivetrain::Drivetrain& drive) {
  AutoManager::AutoManager manager = AutoManager::AutoManager({
      // driveVoltageTime(75, 75, 0.7),
      // driveVoltageTime(-127, -127, 1.5),
      // driveVoltageTime(35, 35, 0.35),
      // driveVoltageTime(-45, -45, 0.5),
      // driveVoltageTime(35, 35, 0.35),
      // driveVoltageTime(-45, -45, 0.5),
      // driveVoltageTime(35, 35, 0.35),
      turnBy(50),
      driveDirection(12, 0),
      driveVoltageTime(35, 35, 0.35),
      waitFor(0.25),
      driveVoltageTime(-55, -55, 0.5),
      waitFor(0.25),
      turnBy(40),
      driveDirection(40, 0),
      turnBy(-45),
      driveDirection(17, 0),
      turnBy(-45),
      driveVoltageTime(-40, -40, 2),
      driveVoltageTime(30, 30, 0.375),
      driveVoltageTime(-30, -30, 0.375),
      driveDirection(18, 0),
      turnBy(-90),
      driveDirection(18, 0),
      driveVoltageTime(35, 35, 1.25),
  });
  manager.initialize();
  while (!manager.isFinished()) {
    manager.periodic();
    drive.periodic();
    pros::delay(DT_S * 1000.0);
  }
}

AutoManager::AutoCommand driveVoltageTime(int32_t left, int32_t right,
                                          double time_s) {
  std::shared_ptr<int> counter = std::make_shared<int>(0);
  return {[left, right]() {
            drive.setState(Drivetrain::States::DriveVoltage(
                [left]() { return left; }, [right]() { return right; }));
          },
          [counter]() { (*counter)++; },
          [counter, time_s]() {
            if ((*counter) >= time_s / DT_S) {
              *counter = 0;
              drive.setState(Drivetrain::States::Idle{});
              return true;
            }
            return false;
          }};
}

AutoManager::AutoCommand driveDirection(double distance_in,
                                        double direction_degs) {
  return {[distance_in, direction_degs]() {
            drive.poseReset();
            drive.setState(Drivetrain::States::DriveDirection(distance_in,
                                                              direction_degs));
          },
          []() {}, []() { return drive.isIdle(); }};
}

AutoManager::AutoCommand turnBy(double direction_degs) {
  return {[direction_degs]() {
            drive.poseReset();
            drive.setState(Drivetrain::States::TurnHeading(direction_degs));
          },
          []() {}, []() { return drive.isIdle(); }};
}

AutoManager::AutoCommand waitFor(double time_s) {
  std::shared_ptr<int> counter = std::make_shared<int>(0);
  return {[]() {}, [counter]() { (*counter)++; },
          [counter, time_s]() {
            if ((*counter) >= time_s / DT_S) {
              *counter = 0;
              drive.setState(Drivetrain::States::Idle{});
              return true;
            }
            return false;
          }};
}