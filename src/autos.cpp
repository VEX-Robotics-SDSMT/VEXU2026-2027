#include "autos.h"

#include "main.h"

void quadrant1(Drivetrain::Drivetrain& drive) {
  unsigned short count = 0;
  unsigned short counter = 0;
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
        break;
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
      case 9:
        if (drive.isIdle()) {
          count++;
        }
        break;
      case 10:
        drive.setState(Drivetrain::States::TurnHeading(90.0));
        count++;
        break;
      case 11:
        if (drive.isIdle()) {
          count++;
        }
        break;
      case 12:
      case 13:
      case 14:
      case 15:
      case 16:
        quadrant1MatchLoadScoreCycle(drive);
        count++;
        break;
      default:
        drive.setState(Drivetrain::States::Idle{});
        drive.periodic();
        return;
    }
    drive.periodic();
    pros::delay(DT_S * 1000.0);
  }
}

static void quadrant1MatchLoadScoreCycle(Drivetrain::Drivetrain& drive) {
  // TODO: There are other subsystems that have to be added here once everything
  // is merged. Approximate function calls are included as is reasonable
  unsigned short count = 0;
  unsigned short counter = 0;
  while (true) {
    switch (count) {
      case 0:
        // intake->retract();
        // intake->setWheels(true);
        count++;
        break;
      case 1:
        drive.poseReset();
        drive.setState(Drivetrain::States::DriveDirection(12.0, 0.0));
        count++;
        break;
      case 2:
        if (drive.isIdle()) {
          count++;
        }
        break;
      case 3:
        drive.setState(Drivetrain::States::DriveVoltage(
            []() { return -25.0; }, []() { return -25.0; }));
        counter++;
        if (counter >= 0.5 / DT_S) {
          count++;
          counter = 0;
        }
        break;
      case 4:
        drive.setState(Drivetrain::States::DriveVoltage([]() { return 25.0; },
                                                        []() { return 25.0; }));
        counter++;
        if (counter >= 0.25 / DT_S) {
          count++;
          counter = 0;
        }
        break;
      case 5:
        // intentional no-reset here because it should bring us back to the 12
        // inch line we started at
        drive.setState(Drivetrain::States::DriveDirection(0.0, 0.0));
        count++;
        break;
      case 6:
        if (drive.isIdle()) {
          count++;
        }
        break;
      case 7:
        drive.setState(Drivetrain::States::TurnHeading(
            -90));  // probably adjust this to be a DriveDirection depending on
                    // actual field performance
        count++;
        break;
      case 8:
        if (drive.isIdle()) {
          count++;
        }
        break;
      // actually score
      default:
        drive.setState(Drivetrain::States::Idle{});
        drive.periodic();
        return;
    }
    drive.periodic();
    pros::delay(DT_S * 1000.0);
  }
}