#pragma once

#include <functional>

#include "devices.h"

#ifdef ROBOT_1
void quadrant1Voltage();

void quadrant1(Drivetrain::Drivetrain& drive, Intake::Intake& intake,
               Lift::Lift& lift, EndEffector::EndEffector& endeffector);
#endif

#ifdef ROBOT_2
void quadrant2Voltage();

void quadrant2Toggle(Drivetrain::Drivetrain& drive);

void quadrant2Conservative(Drivetrain::Drivetrain& drive);
#endif