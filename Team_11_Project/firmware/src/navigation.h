/* 
 * File:   navigation.h
 * Author: Matthew
 *
 * Created on November 4, 2016, 12:51 PM
 */

#ifndef NAVIGATION_H
#define	NAVIGATION_H
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "serializer.h"

#ifdef	__cplusplus
extern "C" {
#endif
    
// saves the home position of the rover
struct DrawMovementCommand gHomePosition;

// saves the current position of the rover
struct DrawMovementCommand gCurrentPosition;

// saves which direction the rover is currently facing.
enum Direction {
    // NORTH is facing up on the y axis
    NORTH, 
    // SOUTH is facing down on the y axis
    SOUTH, 
    // EAST is facing up on the x axis
    EAST, 
    // WEST is facing down on the x axis
    WEST
} gCurrentDirection;

typedef enum {
    // turns LEFT
    LEFT, 
    // turns RIGHT
    RIGHT, 
    // going STRAIGHT
    STRAIGHT
} Turn;

// saves which direction the rover is currently turning
Turn gCurrentTurning;

void DetermineFacingDirection(Turn directionTurned);



#ifdef	__cplusplus
}
#endif

#endif	/* NAVIGATION_H */

