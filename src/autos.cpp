#include "autos.h"

#include "autoManager.hpp"
#include "main.h"

AutoManager::AutoCommand driveVoltageTime(int32_t left, int32_t right,
                                          double time_s);

AutoManager::AutoCommand driveDirection(double distance_in,
                                        double directionDegs);

AutoManager::AutoCommand driveDistance(double distance_in, int32_t speed_v);

AutoManager::AutoCommand turnBy(double direction_degs);
AutoManager::AutoCommand waitFor(double time_s);

AutoManager::AutoCommand setLiftHeight(double inches, bool wait = true);

#ifdef ROBOT_1
AutoManager::AutoCommand setEndEffectorState(EndEffector::State state);
AutoManager::AutoCommand liftClawIntake(bool wait = true);
AutoManager::AutoCommand stationIntake();
AutoManager::AutoCommand intakeIdle();
AutoManager::AutoCommand intakeExtended();

void quadrant1Voltage() {
  AutoManager::AutoManager manager = AutoManager::AutoManager({
      // deploy the arms
      driveVoltageTime(75, 75, 0.6),
      driveVoltageTime(-127, -127, 1.2),
      // set the toggle
      driveVoltageTime(35, 35, 0.35),
      driveVoltageTime(-45, -45, 0.5),
      driveVoltageTime(35, 35, 0.35),
      driveVoltageTime(-45, -45, 0.5),
      // flip the latch so that the effector deploys properly
      setEndEffectorState(EndEffector::State::INTAKING_RETRACTED),
      // finish toggle
      driveVoltageTime(35, 35, 0.35),
      // point towards the target
      turnBy(60),
      // get ready to score
      setEndEffectorState(EndEffector::State::READY),
      driveVoltageTime(35, 35, 2),
      setEndEffectorState(EndEffector::State::SCORING),
      waitFor(1.5),
      // START NEW STUFF FROM LAST TESTING
      // drive back from the goal
      driveDistance(12, -55),
      // turn towards the right wall
      turnBy(30),
      // go to pick up the cup/pin pair standing on the field
      driveDistance(35, 75),
      turnBy(-45),
      setEndEffectorState(EndEffector::State::INTAKING_EXTENDED),
      setLiftHeight(5, false),
      driveDistance(13, 35),
      // actually pick it up
      setLiftHeight(0, true),
      setLiftHeight(10, false),
      // drive back to the goal
      driveDistance(9, -35),
      turnBy(-135),
      driveVoltageTime(45, 45, 2),
      // score
      setEndEffectorState(EndEffector::State::SCORING),
      waitFor(1.5),
      // done
      setEndEffectorState(EndEffector::State::RETRACTED),

  });

  manager.initialize();
  while (!manager.isFinished()) {
    manager.periodic();
    drive.periodic();
    endeffector.periodic();
    lift.periodic();
    intake.periodic();
    pros::delay(DT_S * 1000.0);
  }
  for (uint8_t i = 10; i <= 30; i += 10) {
    AutoManager::AutoManager cycleManager = AutoManager::AutoManager({
        driveVoltageTime(-30, -30, 0.5),
        liftClawIntake(),
        driveDistance(25, -65),
        turnBy(90),
        stationIntake(),
        driveVoltageTime(-60, -60, 2),
        driveVoltageTime(35, 35, 0.375),
        intakeExtended(),
        driveVoltageTime(-35, -35, 0.5),
        driveDistance(18, 50),
        turnBy(-90),
        setEndEffectorState(EndEffector::State::READY),
        setLiftHeight(i, false),
        driveDistance(12, 75),
        // really just wait until the lift is at its height
        setLiftHeight(i, true),
        driveVoltageTime(35, 35, 2),
        setEndEffectorState(EndEffector::State::SCORING),
    });

    cycleManager.initialize();
    while (!cycleManager.isFinished()) {
      cycleManager.periodic();
      drive.periodic();
      endeffector.periodic();
      lift.periodic();
      intake.periodic();
      pros::delay(DT_S * 1000.0);
    }
  }
}

void quadrant1(Drivetrain::Drivetrain& drive, Intake::Intake& intake,
               Lift::Lift& lift, EndEffector::EndEffector& endeffector) {
  AutoManager::AutoManager manager = AutoManager::AutoManager({
      // deploy the arms
      driveVoltageTime(75, 75, 0.6),
      driveVoltageTime(-127, -127, 1.2),
      // set the toggle
      driveVoltageTime(35, 35, 0.35),
      driveVoltageTime(-45, -45, 0.5),
      driveVoltageTime(35, 35, 0.35),
      driveVoltageTime(-45, -45, 0.5),
      // flip the latch so that the effector deploys properly
      setEndEffectorState(EndEffector::State::INTAKING_RETRACTED),
      // finish toggle
      driveVoltageTime(35, 35, 0.35),
      // point towards the target
      turnBy(60),
      // get ready to score
      setEndEffectorState(EndEffector::State::READY),
      // setLiftHeight(5, false),
      driveDirection(12, 0),
      driveVoltageTime(55, 55, 0.5),
      // score
      setEndEffectorState(EndEffector::State::SCORING),
      waitFor(0.125),
      setEndEffectorState(EndEffector::State::READY),
      driveVoltageTime(-55, -55, 0.5),
      // liftClawIntake(),
      waitFor(1.125),
      turnBy(40),
      waitFor(1.125),
      driveDirection(40, 0),
      waitFor(1.125),
      turnBy(-45),
      waitFor(1.125),
      // setClawState(EndEffector::State::INTAKING_EXTENDED),
      // setLiftHeight(10),
      driveDirection(17, 0),
      // pick up the on-floor game-piece
      // setLiftHeight(2),
      // setLiftHeight(10),
      turnBy(-135),
      driveDirection(10, 0),
      driveVoltageTime(35, 35, 0.5),
      // score
      setEndEffectorState(EndEffector::State::SCORING),
      waitFor(0.125),
      driveVoltageTime(-35, -35, 0.25),
      // liftClawIntake(),
      // return to intake station

      // score
  });

  manager.initialize();
  while (!manager.isFinished()) {
    manager.periodic();
    drive.periodic();
    intake.periodic();
    lift.periodic();
    endeffector.periodic();
    pros::delay(DT_S * 1000.0);
  }

  // int height = 10;
  // for (int i = 0; i < 5; i++) {
  //   AutoManager::AutoManager scoreCycleManager({
  //       // get pointed at the intake station
  //       driveDirection(-15, 0),
  //       turnBy(90),

  //       // do the intaking

  //       setClawState(EndEffector::State::INTAKING_RETRACTED),
  //       stationIntake(),
  //       driveVoltageTime(-40, -40, 2),
  //       driveVoltageTime(30, 30, 0.375),
  //       driveVoltageTime(-30, -30, 0.375),
  //       intakeExtended(),
  //       // leave the station
  //       driveDirection(16, 0),
  //       intakeIdle(),
  //       turnBy(-90),
  //       // drive to the station
  //       driveDirection(18, 0),
  //       setLiftHeight(height),
  //       driveVoltageTime(35, 35, 1.25),
  //       // score
  //       setClawState(EndEffector::State::SCORING),
  //       waitFor(0.125),
  //       driveVoltageTime(-35, -35, 0.25),
  //       liftClawIntake(),
  //   });

  //   height += 6;
  // }
}

AutoManager::AutoCommand setEndEffectorState(EndEffector::State state) {
  return {
      [state]() { endeffector.setState(state); },
      []() {},
      []() { return true; },
  };
}

AutoManager::AutoCommand liftClawIntake(bool wait) {
  return {
      []() {
        // lift.setState(Lift::States::Position(0));
      },
      []() {},
      [wait]() {
        if (true || std::abs(lift.getHeight()) < 1.0) {
          endeffector.setState(EndEffector::State::INTAKING_RETRACTED);
          return true;
        }
        return !wait;
      },
  };
}

AutoManager::AutoCommand stationIntake() {
  return {
      []() { intake.setState(Intake::State::INTAKING_RETRACTED); },
      []() {},
      []() { return true; },
  };
}

AutoManager::AutoCommand intakeIdle() {
  return {
      []() { intake.setState(Intake::State::IDLE_RETRACTED); },
      []() {},
      []() { return true; },
  };
}

AutoManager::AutoCommand intakeExtended() {
  return {
      []() { intake.setState(Intake::State::INTAKING_EXTENDED); },
      []() {},
      []() { return true; },
  };
}

#endif

#ifdef ROBOT_2
AutoManager::AutoCommand setClawState(Claw::State state);

void quadrant2Voltage() {
  AutoManager::AutoManager manager = AutoManager::AutoManager({
      // deploy the arms
      driveVoltageTime(75, 75, 0.6),
      driveVoltageTime(-127, -127, 1.2),
      // set the toggle
      driveVoltageTime(35, 35, 0.35),
      driveVoltageTime(-45, -45, 0.5),
      driveVoltageTime(35, 35, 0.35),
      driveVoltageTime(-45, -45, 0.5),
      driveVoltageTime(35, 35, 0.35),
      // point towards the target
      setLiftHeight(5, false),
      turnBy(-45),
      // get ready to score
      driveVoltageTime(35, 35, 1.75),
      setClawState(Claw::State::OPEN),
      waitFor(1.25),
      // START NEW STUFF FROM LAST TESTING
      driveDistance(10, -45),
      turnBy(-45),
      driveDistance(18, 50),
  });

  manager.initialize();
  while (!manager.isFinished()) {
    manager.periodic();
    drive.periodic();
    claw.periodic();
    pros::delay(DT_S * 1000.0);
  }

  for (uint8_t i = 10; i <= 30; i += 10) {
    AutoManager::AutoManager cycleManager = AutoManager::AutoManager({
        driveVoltageTime(-60, -60, 2.5),
        setClawState(Claw::State::CLOSED),
        driveDistance(18, 70),
        turnBy(-135),
        setLiftHeight(i, false),
        driveDistance(18, 45),
        setLiftHeight(i, true),
        driveVoltageTime(35, 35, 2),
        setClawState(Claw::State::OPEN),
        waitFor(1.25),
        driveVoltageTime(-35, -35, 0.5),
        setLiftHeight(0),
        driveDistance(18, -45),
        turnBy(135),
    });

    cycleManager.initialize();
    while (!cycleManager.isFinished()) {
      drive.periodic();
      claw.periodic();
      pros::delay(DT_S * 1000.0);
    }
  }
}

AutoManager::AutoCommand setClawState(Claw::State state) {
  return {
    [state]() { claw.setState(state); }, []() {}, []() { return true; },
  };
}

#endif

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

AutoManager::AutoCommand driveDistance(double distance_in, int32_t speed_v) {
  return {
      [speed_v]() {
        drive.poseReset();
        drive.setState(Drivetrain::States::DriveVoltage(
            [speed_v]() { return speed_v; }, [speed_v]() { return speed_v; }));
      },
      []() {},
      [distance_in]() {
        if (drive.getDistIn() >= distance_in) {
          drive.setState(Drivetrain::States::Idle{});
          return true;
        }
        return false;
      }};
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

AutoManager::AutoCommand setLiftHeight(double inches, bool wait) {
  return {[inches]() {
            // lift.setState(Lift::States::Position(inches));
          },
          []() {},
          [inches, wait]() {
            return true;
            return (!wait) || std::abs(lift.getHeight() - inches) <= 1.0;
          }};
}
