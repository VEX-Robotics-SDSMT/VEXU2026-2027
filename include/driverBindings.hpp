#pragma once
#include "main.h"

namespace DriverBindings {
#ifdef ROBOT_1
pros::controller_digital_e_t toggleOuttake = pros::E_CONTROLLER_DIGITAL_R2;
pros::controller_digital_e_t toggleIntake = pros::E_CONTROLLER_DIGITAL_R1;

pros::controller_digital_e_t toggleIntakeRetracted =
    pros::E_CONTROLLER_DIGITAL_L1;

pros::controller_digital_e_t liftDown = pros::E_CONTROLLER_DIGITAL_DOWN;
pros::controller_digital_e_t liftUp = pros::E_CONTROLLER_DIGITAL_B;

pros::controller_digital_e_t clawFlipToggle = pros::E_CONTROLLER_DIGITAL_A;
pros::controller_digital_e_t clawScoreToggle = pros::E_CONTROLLER_DIGITAL_L2;

#endif
#ifdef ROBOT_2
#endif
}  // namespace DriverBindings