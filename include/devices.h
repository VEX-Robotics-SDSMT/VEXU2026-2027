#pragma once
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
const inline int8_t LM1 = 5;
const inline int8_t LM2 = 6;
const inline int8_t LM3 = 7;
const inline int8_t LM4 = 9;
const inline int8_t LM5 = 17;

// Right Motors
const inline int8_t RM1 = 3;
const inline int8_t RM2 = 12;
const inline int8_t RM3 = 15;
const inline int8_t RM4 = 14;
const inline int8_t RM5 = 4;

// Intake Motors
const inline int8_t IN1 = 1;
const inline int8_t IN2 = 1;

// Encoder
const inline int8_t XENC = 1;
const inline int8_t YENC = 1;

// Inertial
const inline uint8_t Inertial = 1;

// Loader
const inline int8_t L1 = 1;
const inline int8_t L2 = 1;

// Pneumatics
const inline char INTAKELIFT = 'A';
const inline char SCOREARM = 'A';

#endif

//===============================================
// Motor Groups
//===============================================

// Drive Train Motors
inline pros::MotorGroup leftMotors({-LM1, -LM2, -LM3, -LM4, -LM5});
inline pros::MotorGroup rightMotors({RM1, RM2, RM3, RM4, RM5});

// Intake Motors
inline pros::MotorGroup intakeMotors({IN1, -IN2});

// Loader Arm
inline pros::MotorGroup loaders({L1, -L2});

//===============================================
// Sensors
//===============================================

inline pros::Imu Imu(Inertial);

inline pros::Rotation xEncoder(XENC);
inline pros::Rotation yEncoder(YENC);

//===============================================
// Pneumatics
//===============================================

//===============================================
// Drivetrain
//===============================================

inline Mines::Drivetrain drive(&leftMotors, &rightMotors, &xEncoder, &Imu);