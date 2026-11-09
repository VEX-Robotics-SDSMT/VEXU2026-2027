#include "autos.h"

#include "main.h"

void quadrant1(Drivetrain::Drivetrain& drive) {
  int count = 0;
  int counter = 0;
  while (true) {
    switch (count) {
      case 0:
        drive.setState(Drivetrain::States::DriveVoltage(
            []() { return -50.0; }, []() { return -50.0; }));
        counter++;
        if (counter >= 0.25 / DT_S) {
          count++;
          counter = 0;
        }
        break;
        // drive.setState(Drivetrain::States::DriveVoltage(
        //     []() { return -50.0; }, []() { return -50.0; }));
        // counter++;
        // if (counter >= 0.25 / DT_S) {
        //   count++;
        //   counter = 0;
        // }
        break;
      case 1:
        drive.setState(Drivetrain::States::DriveVoltage([]() { return 50.0; },
                                                        []() { return 50.0; }));
        counter++;
        if (counter >= 0.25 / DT_S) {
          count++;
          counter = 0;
        }
        break;
      case 2:
        drive.setState(Drivetrain::States::DriveVoltage(
            []() { return -50.0; }, []() { return -50.0; }));
        counter++;
        if (counter >= 0.5 / DT_S) {
          count++;
          counter = 0;
        }
        break;
      case 3:
        drive.poseReset();
        drive.setState(Drivetrain::States::DriveDirection(18.0, 35.0));
        count++;
      case 4:
        if (drive.isIdle()) {
          count++;
        }
        break;
      case 5:
        drive.setState(Drivetrain::States::DriveVoltage(
            []() { return -50.0; }, []() { return -50.0; }));
        counter++;
        if (counter >= 0.5 / DT_S) {
          count++;
          counter = 0;
        }
        break;
      case 6:
        drive.setState(Drivetrain::States::TurnHeading(90.0));
        count++;
        break;
      case 7:
        if (drive.isIdle()) {
          count++;
        }
        break;
      case 8:
        drive.poseReset();
        drive.setState(Drivetrain::States::DriveDirection(36.0, 0));
        count++;
        break;
      default:
        drive.setState(Drivetrain::States::DriveVoltage([]() { return 0.0; },
                                                        []() { return 0.0; }));
        return;
    }
    drive.periodic();
    pros::delay(DT_S * 1000.0);
  }
}