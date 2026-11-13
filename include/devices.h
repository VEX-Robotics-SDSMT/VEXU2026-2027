#pragma once
#include "Drivetrain.h"
#include "endeffector.h"
#include "intake.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"

//===============================================
// Controller
//===============================================
inline pros::Controller master(pros::controller_id_e_t::E_CONTROLLER_MASTER);

//===============================================
// Ports
//===============================================
#define RobotOne

#ifdef RobotOne

// Left Motors
const inline int8_t LM1 = 14;
const inline int8_t LM2 = 4;
const inline int8_t LM3 = 12;
const inline int8_t LM4 = 15;
const inline int8_t LM5 = 3;

// Right Motors
const inline int8_t RM1 = 6;
const inline int8_t RM2 = 5;
const inline int8_t RM3 = 9;
const inline int8_t RM4 = 18;
const inline int8_t RM5 = 7;

// Inertial
const inline uint8_t imu = 19;

// Intake Motors
const inline int8_t IN1 = 8;
const inline int8_t IN2 = 10;

// Score Motors
const inline uint8_t SCORE1 = 2;

// Lift Motors
const inline int8_t LIFTMOTOR1 = 1;
const inline int8_t LIFTMOTOR2 = 1;
const inline int8_t LIFTENC = 1;

// Encoder
const inline int8_t XENC = 4;

// Loader
const inline int8_t L1 = 1;
const inline int8_t L2 = 1;

// Pneumatics
const inline char INTAKELIFT = 'A';
const inline char SCOREARM = 'B';

#endif

//===============================================
// Motor Groups
//===============================================

// Drive Train Motors
inline pros::MotorGroup leftMotors({-LM1, LM2, -LM3, LM4,
                                    -LM5});  //, -LM2, -LM3, -LM4, -LM5});
inline pros::MotorGroup rightMotors({RM1, -RM2, -RM3, RM4,
                                     RM5});  //, RM2, RM3, RM4, RM5});

// Intake Motors
inline pros::MotorGroup intakeMotors({-IN1, -IN2});

// Loader Arm
inline pros::MotorGroup loaders({L1, -L2});

// Elevator Motors
inline pros::MotorGroup liftMotors({LIFTMOTOR1, -LIFTMOTOR2});
// Score Motors
inline pros::MotorGroup scoreMotors({SCORE1});

//===============================================
// Sensors
//===============================================

inline pros::IMU Imu(imu);

inline pros::Rotation xEncoder(XENC);

inline pros::Rotation liftEncoder(LIFTENC);

//===============================================
// Pneumatics
//===============================================
inline pros::adi::Pneumatics intakePneumatics(INTAKELIFT, false);

inline pros::adi::Pneumatics scoreFlipperPneumatics(SCOREARM, false);

//===============================================
// Drivetrain
//===============================================

inline Drivetrain::Drivetrain drive(&leftMotors, &rightMotors, &xEncoder, &Imu);
inline Intake::Intake intake(&intakePneumatics, &intakeMotors);
inline EndEffector::EndEffector endeffector(&scoreFlipperPneumatics,
                                            &scoreMotors);
