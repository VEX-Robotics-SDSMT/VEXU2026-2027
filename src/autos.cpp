#include "autos.h"

void quadrant1(Drivetrain::Drivetrain& drive) {
  int count = 0;
  while (true) {
    switch (count) {
      case 0:
        drive.setState(Drivetrain::States::DriveDirection(12.0, 0.0));
        drive.periodic();
        if (drive.isIdle()) {
          count++;
        }
        break;

      case 1:
        drive.setState(Drivetrain::States::TurnHeading(90.0));
        drive.periodic();
        if (drive.isIdle()) {
          count++;
        }
        break;
      case 2:
        drive.setState(Drivetrain::States::DriveDirection(12.0, -90.0));
        drive.periodic();
        if (drive.isIdle()) {
          count++;
        }
        break;
      default:
        return;
    }
  }
}