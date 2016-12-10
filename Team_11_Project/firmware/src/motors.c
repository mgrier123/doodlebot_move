#include "motors.h"

/******************************************************************************
 * Function:
 *      InitRoverMotors()
 * 
 * Remarks:
 *      This initializes the rover motors and output compare module
 */
void InitRoverMotors() 
{
    //Initialize Motors
    //Set up proper pins as outputs (left)
        //pin that handles left motor enable - see data sheet
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_0); //direction
        //pin that handles left motor direction - see data sheet
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14); 
    
    //Keep motors off during initialize
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
            
    //Set up proper pins as outputs (right)
        //pin that handles right motor enable - see data sheet
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_1); //direction
        //pin that handles right motor direction - see data sheet
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1); 
    
    //set motors off to init
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
    
    //Start OC drivers
    DRV_OC0_Enable(); 
    DRV_OC1_Enable(); 
    
    DRV_OC0_Start();    //Start the OC drivers that talk to the motor shield
    DRV_OC1_Start();
    
    PLIB_PORTS_PinDirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);    
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_3);
}

/******************************************************************************
 * Function:
 *      MoveForward()
 * 
 * Remarks:
 *      Moves the rover forward
 */
void MoveForward()
{
    gMotorSpeed.rightDistance = ONE_CENTIMETER_TICKS;
    gMotorSpeed.leftDistance = ONE_CENTIMETER_TICKS;
    //Make right motor go forward 
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, gMotorSpeed.rightSpeed);
    
    //Make left motor go forward
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, gMotorSpeed.leftSpeed);
    
    gRightEncoder = 0;
    gLeftEncoder = 0;
    
    while ((gRightEncoder < gMotorSpeed.rightDistance) || (gLeftEncoder < gMotorSpeed.leftDistance)){;}
    MoveStop();
}

/******************************************************************************
 * Function:
 *      MoveForward()
 * 
 * Remarks:
 *      Moves the rover forward
 */
void MoveForwardDrawing()
{
    gMotorSpeed.rightDistance = DRAWING_TICKS;
    gMotorSpeed.leftDistance = DRAWING_TICKS;
    //Make right motor go forward 
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, gMotorSpeed.rightSpeed);
    
    //Make left motor go forward
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, gMotorSpeed.leftSpeed);
    
    gRightEncoder = 0;
    gLeftEncoder = 0;
    
    while ((gRightEncoder < gMotorSpeed.rightDistance) || (gLeftEncoder < gMotorSpeed.leftDistance)){;}
    MoveStop();
}

/******************************************************************************
 * Function:
 *      MoveForward()
 * 
 * Remarks:
 *      Moves the rover forward
 */
void AdjustForward()
{
    gMotorSpeed.rightDistance = FORWARD_ADJUSTMENT_TICKS;
    gMotorSpeed.leftDistance = FORWARD_ADJUSTMENT_TICKS;
    //Make right motor go forward 
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, gMotorSpeed.rightSpeed);
    
    //Make left motor go forward
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, gMotorSpeed.leftSpeed);
    
    gRightEncoder = 0;
    gLeftEncoder = 0;
    
    while ((gRightEncoder < gMotorSpeed.rightDistance) || (gLeftEncoder < gMotorSpeed.leftDistance)){;}
    MoveStop();
}

/******************************************************************************
 * Function:
 *      MoveForward()
 * 
 * Remarks:
 *      Moves the rover backward
 */
void MoveBackward()
{
    gMotorSpeed.rightDistance = ONE_CENTIMETER_TICKS;
    gMotorSpeed.leftDistance = ONE_CENTIMETER_TICKS;
    //Make right motor go backward 
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, gMotorSpeed.rightSpeed);
    
    //Make left motor go backward
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, gMotorSpeed.leftSpeed);
    
    gRightEncoder = 0;
    gLeftEncoder = 0;
    
    while ((gRightEncoder < gMotorSpeed.rightDistance) || (gLeftEncoder < gMotorSpeed.leftDistance)){;}
    MoveStop();
}

/******************************************************************************
 * Function:
 *      MoveStop()
 * 
 * Remarks:
 *      Stops the rover
 */
void MoveStop()
{
    //Make left motor stop 
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, 0);
    //Make right motor stop
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, 0);
}

/******************************************************************************
 * Function:
 *      MoveRight()
 * 
 * Remarks:
 *      Spins the rover right
 */
void MoveRight()
{
    gMotorSpeed.rightDistance = NINETY_DEGREE_RIGHT_TICKS;
    gMotorSpeed.leftDistance = NINETY_DEGREE_LEFT_TICKS;
    //Make right motor go forward 
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, gMotorSpeed.rightSpeed);
    
    //Make left motor go forward
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, gMotorSpeed.leftSpeed);
    
    gRightEncoder = 0;
    gLeftEncoder = 0;
    
    while (gRightEncoder < gMotorSpeed.rightDistance){;}
    MoveStop();
    
    gCurrentTurning = RIGHT;
    DetermineFacingDirection(gCurrentTurning);
}

/******************************************************************************
 * Function:
 *      MoveLeft()
 * 
 * Remarks:
 *      Spins the rover left
 */
void MoveLeft()
{
    gMotorSpeed.rightDistance = NINETY_DEGREE_RIGHT_TICKS;
    gMotorSpeed.leftDistance = NINETY_DEGREE_LEFT_TICKS;
    
    //Make right motor go forward 
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, gMotorSpeed.rightSpeed);
    
    //Make left motor go forward
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, gMotorSpeed.leftSpeed);
    
    gRightEncoder = 0;
    gLeftEncoder = 0;
    
    while (gLeftEncoder < gMotorSpeed.leftDistance){;}
    MoveStop();
    
    gCurrentTurning = LEFT;
    DetermineFacingDirection(gCurrentTurning);
}

/******************************************************************************
 * Function:
 *      MoveForward()
 * 
 * Remarks:
 *      Moves the rover forward
 */
void AdjustLeft()
{
    gMotorSpeed.rightDistance = FORWARD_ADJUSTMENT_TICKS;
    gMotorSpeed.leftDistance = FORWARD_ADJUSTMENT_TICKS;
    //Make right motor go forward 
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, gMotorSpeed.rightSpeed);
    
    //Make left motor go forward
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, gMotorSpeed.leftSpeed);
    
    gRightEncoder = 0;
    gLeftEncoder = 0;
    
    while ((gLeftEncoder < gMotorSpeed.leftDistance)){;}
    MoveStop();
}

/******************************************************************************
 * Function:
 *      MoveForward()
 * 
 * Remarks:
 *      Moves the rover forward
 */
void AdjustRight()
{
    gMotorSpeed.rightDistance = FORWARD_ADJUSTMENT_TICKS;
    gMotorSpeed.leftDistance = FORWARD_ADJUSTMENT_TICKS;
    //Make right motor go forward 
    PLIB_PORTS_PinSet(PORTS_ID_0, PORT_CHANNEL_C, PORTS_BIT_POS_14);
    PLIB_OC_PulseWidth16BitSet(OC_ID_1, gMotorSpeed.rightSpeed);
    
    //Make left motor go forward
    PLIB_PORTS_PinClear(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_1);
    PLIB_OC_PulseWidth16BitSet(OC_ID_2, gMotorSpeed.leftSpeed);
    
    gRightEncoder = 0;
    gLeftEncoder = 0;
    
    while ((gRightEncoder < gMotorSpeed.rightDistance)){;}
    MoveStop();
}