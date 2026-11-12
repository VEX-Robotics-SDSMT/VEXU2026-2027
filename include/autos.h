#pragma once

#include <functional>

#include "devices.h"

/*
This is a function that is internal to the autos.
It drives from one foot out from the match load station and does the loading
procedure, then returns to the alliance colored scoring location and scores
It then returns to the location 1 foot from the match load station
*/
void quadrant1MatchLoadScoreCycle(
    Drivetrain::Drivetrain& drive,
    std::function<bool(void)> run = []() { return true; });

void quadrant1(Drivetrain::Drivetrain& drive);

void quadrant2Toggle(Drivetrain::Drivetrain& drive);

void quadrant2Conservative(Drivetrain::Drivetrain& drive);