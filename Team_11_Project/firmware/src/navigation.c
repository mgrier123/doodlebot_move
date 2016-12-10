#include "navigation.h"

/******************************************************************************
 * Function:
 *      DetermineFacingDirection(Turn directionTurned)
 * 
 * Remarks:
 *      This determines what direction the rover is facing based off of which
 *      direction the rover turned
 */
void DetermineFacingDirection(Turn directionTurned)
{
    switch(gCurrentDirection)
    {
        case NORTH:
            switch(directionTurned)
            {
                case LEFT:
                    gCurrentDirection = WEST;
                    break;
                case RIGHT:
                    gCurrentDirection = EAST;
                    break;
            }
            break;
            
        case SOUTH:
            switch(directionTurned)
            {
                case LEFT:
                    gCurrentDirection = EAST;
                    break;
                case RIGHT:
                    gCurrentDirection = WEST;
                    break;
            }
            break;
            
        case WEST:
            switch(directionTurned)
            {
                case LEFT:
                    gCurrentDirection = SOUTH;
                    break;
                case RIGHT:
                    gCurrentDirection = NORTH;
                    break;
            }
            break;
            
        case EAST:
            switch(directionTurned)
            {
                case LEFT:
                    gCurrentDirection = NORTH;
                    break;
                case RIGHT:
                    gCurrentDirection = SOUTH;
                    break;
            }
            break;
    }
}