#pragma once

/**
 * @file MinesLib.h
 * @brief Header file to include the MinesLib library files in your project.
 */

#include "PID.h"
#include "Drivetrain.h"
#include "stdio.h"

#ifdef MINESLIB_TEST_LINKING
inline void MinesLibTestLinking()
{
    printf("MinesLib linked successfully.\n");
}
#endif