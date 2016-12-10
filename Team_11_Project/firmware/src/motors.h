/* 
 * File:   motors.h
 * Author: mgrier123
 *
 * Created on November 2, 2016, 1:08 PM
 */

#ifndef MOTORS_H
#define	MOTORS_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "navigation.h"

#define NINETY_DEGREE_RIGHT_TICKS 235 //265
#define NINETY_DEGREE_LEFT_TICKS 275
#define ONE_CENTIMETER_TICKS 27
#define FORWARD_ADJUSTMENT_TICKS 3
#define GRID_SQUARE_SIZE 7
#define DRAWING_TICKS 32

#ifdef	__cplusplus
extern "C" {
#endif
    
// this struct holds config information when doing motor commands
struct motorSpeed
{
    // speed the left motor will move at
    uint16_t leftSpeed;
    // speed the right motor will move at
    uint16_t rightSpeed;
    // encoder distance for left motor to move
    uint16_t leftDistance;
    // encoder distance for right motor to move
    uint16_t rightDistance;
} gMotorSpeed;

// this counts the ticks for the left encoder
volatile uint32_t gLeftEncoder;
// this counts the ticks for the left encoder
volatile uint32_t gRightEncoder;

void MoveForward();
void MoveBackward();
void MoveStop();
void MoveRight();
void MoveLeft();
void AdjustLeft();
void AdjustRight();
void AdjustForward();
void MoveForwardDrawing();


#ifdef	__cplusplus
}
#endif

#endif	/* MOTORS_H */

