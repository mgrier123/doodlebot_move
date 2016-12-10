/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    ir_sensor.c
  Summary:
    This file contains the source code for the MPLAB Harmony application.
  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.
Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).
You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.
SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "ir_sensor.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data
  Summary:
    Holds application data
  Description:
    This structure holds the application's data.
  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void IR_SENSOR_Initialize ( void )
  Remarks:
    See prototype in ir_sensor.h.
 */
void IR_SENSOR_Initialize ( void )
{
    int i;
    gIrClock = 0;
    
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_B, PORTS_BIT_POS_7);
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_B, PORTS_BIT_POS_8);
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_B, PORTS_BIT_POS_9);
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_B, PORTS_BIT_POS_10);
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_B, PORTS_BIT_POS_11);
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, PORT_CHANNEL_B, PORTS_BIT_POS_12);
    
    //turn on sensor inputs
    PLIB_PORTS_PinModeSelect(PORTS_ID_0, PORTS_ANALOG_PIN_8, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_PinModeSelect(PORTS_ID_0, PORTS_ANALOG_PIN_9, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_PinModeSelect(PORTS_ID_0, PORTS_ANALOG_PIN_10, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_PinModeSelect(PORTS_ID_0, PORTS_ANALOG_PIN_11, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_PinModeSelect(PORTS_ID_0, PORTS_ANALOG_PIN_12, PORTS_PIN_MODE_DIGITAL);
    PLIB_PORTS_PinModeSelect(PORTS_ID_0, PORTS_ANALOG_PIN_13, PORTS_PIN_MODE_DIGITAL);
    
    gSensorRead = 0;
    gLines = 0;
    gState = 0;
    gRightLeft = 0;
    gApproachLeave = 1;
    gLines = 0;
    dbgOutputLoc(5);
}

/******************************************************************************
 Function:
    void charge_IR_sensor(int sensor_num, PORT_CHANNEL port_channel)
  
 Arguments:
 * The number of the sensor you want to charge
 * The channel the sensor is on
 * 
 Remarks:
 * Tells the pic to charge a particular sensor on the IR Array
 */
void ChargeIrSensor(int sensor_num, PORTS_CHANNEL port_channel)
{
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_OUTPUT, port_channel, sensor_num);
    SYS_PORTS_PinWrite(PORTS_ID_0, port_channel, sensor_num, true);
}

/******************************************************************************
 Function:
    void discharge_IR_sensor(int sensor_num, PORT_CHANNEL port_channel)
  
 Arguments:
 * The number of the sensor you want to charge
 * The channel the sensor is on
 * 
 Remarks:
 * Tells the pic to discharge a particular sensor on the IR Array
 */
void DischargeIrSensor(int sensor_num, PORTS_CHANNEL port_channel)
{
    SYS_PORTS_PinDirectionSelect (PORTS_ID_0, SYS_PORTS_DIRECTION_INPUT, port_channel, sensor_num);
    
}

void ReadSensor()
{
    int i;
    for (i = 0; i < 6; i++)
    {
        ChargeIrSensor(i+8, PORT_CHANNEL_B);

    }
    WaitDesiredTime(315);

    for (i = 0; i < 6; i++)
    {
        DischargeIrSensor(i + 8, PORT_CHANNEL_B);
    }

    WaitDesiredTime(3000);

    for (i = 0; i < 6; i++)
    {
        gResult[i] = SYS_PORTS_PinRead(PORTS_ID_0, PORT_CHANNEL_B, i+8);            
        if (gResult[i] == 0)
        {
            gResult[i] = 2;
        }
    }

    gIrClock = 0;
    gSensorRead = 1;
}

int ConvertIrArray()
{
    int ir_array_num = 0;
    
    if (gResult[0] == 1)
    {
        ir_array_num = ir_array_num + 100000;
    }
    else
    {
        ir_array_num = ir_array_num + 200000;
    }
    
    if (gResult[1] == 1)
    {
        ir_array_num = ir_array_num + 10000;
    }
    else
    {
        ir_array_num = ir_array_num + 20000;
    }
    
    if (gResult[2] == 1)
    {
        ir_array_num = ir_array_num + 1000;
    }
    else
    {
        ir_array_num = ir_array_num + 2000;
    }
    
    if (gResult[3] == 1)
    {
        ir_array_num = ir_array_num + 100;
    }
    else
    {
        ir_array_num = ir_array_num + 200;
    }
    
    if (gResult[4] == 1)
    {
        ir_array_num = ir_array_num + 10;
    }
    else
    {
        ir_array_num = ir_array_num + 20;
    }
    
    if (gResult[5] == 1)
    {
        ir_array_num = ir_array_num + 1;
    }
    else
    {
        ir_array_num = ir_array_num + 2;
    }
    
    return ir_array_num;
}

void FollowLine()
{
    while(1)
    {
        if (gSensorRead == 1)
        {
            gSensorRead = 0;
            gIrData = ConvertIrArray();
        }

        ReadSensor();

        if (MoveScout())
        {   
            return;
        }   
    }
    
}

bool MoveScout()
{
    if (gIrData == 222222)
    {
        return true;
    }
    else if (gIrData == 221122)
    {
        switch(gCurrentTurning)
        {
            case LEFT:
                AdjustLeft();
                break;
            case RIGHT:
                AdjustRight();
                break;
            case STRAIGHT:
                break;
            default:
                break;
        }
    }
    else if (gIrData == 111111 && gNavigationState != RETURN)
    {
        gSendCancel = true;
        return true;
    }
    else if (gIrData == 111111 && gNavigationState == RETURN)
    {
        return true;
    }
    else if (gIrData == 111222)
    {
        if (!gApproachLeave)
            AdjustRight();
        else
            AdjustForward();
    }
    else if (gIrData == 222111)
    {
        if (!gApproachLeave)
            AdjustLeft();
        else
            AdjustForward();
    }
    else if (gIrData == 112222 || gIrData == 122222 || gIrData == 212222)
    {
        if (!gApproachLeave)
            AdjustRight();
        else
            AdjustLeft();
    }
    else if (gIrData == 222211 || gIrData == 222221)
    {
        if (!gApproachLeave)
            AdjustLeft();
        else
            AdjustRight();
    }
    else if (gIrData == 222112 || gIrData == 222121)
    {
        if (!gApproachLeave)
            AdjustForward();
        else
            AdjustRight();
    }
    else if (gIrData == 112211 ||
             gIrData == 122221 ||
             gIrData == 212211 ||
             gIrData == 212212 ||
             gIrData == 212221 ||
            gIrData == 112121 ||
            gIrData == 222122 ||
            gIrData == 112221 ||
            gIrData == 112111 ||
            gIrData == 121111 ||
            gIrData == 122111 ||
            gIrData == 122121 ||
            gIrData == 122211 ||
            gIrData == 111121 ||
            gIrData == 111211 ||
            gIrData == 111221 ||
            gIrData == 222212)
            {
                    AdjustForward();
            }
    else if (
            gIrData == 112212 ||
            gIrData == 122112 ||
            gIrData == 122122 ||
            gIrData == 122212 ||
            gIrData == 212111 ||
            gIrData == 212112 ||
            gIrData == 212121 ||
            gIrData == 221111 ||
            gIrData == 221112 ||
            gIrData == 221121 ||
            gIrData == 221211 ||
            gIrData == 221212 ||
            gIrData == 112112 ||
            gIrData == 121112 ||
            gIrData == 121211 ||
            gIrData == 211111)
    {
        AdjustLeft();
    }
    else if (gIrData == 111112 ||
            gIrData == 111122 ||
            gIrData == 112122 ||
            gIrData == 121122 ||
            gIrData == 121222 ||
            gIrData == 211121 ||
            gIrData == 211122 ||
            gIrData == 211211 ||
            gIrData == 211212 ||
            gIrData == 211221 ||
            gIrData == 211222 ||
            gIrData == 212122 ||
            gIrData == 221221 ||
            gIrData == 221222 ||
            gIrData == 111212)
    {
        AdjustRight();
    }
    
    return false;
}

void WaitDesiredTime(int delay)
{
    T1CON = 0x0030; // turn timer off and set prescaller to 1:256
    TMR1 = 0;
    PR1 = 0xFFFF;
    T1CONSET = 0x8000; // start timer
    while (TMR1 < delay) {
            // just wait
    }
    T1CONCLR = 0x8000; // stop timer
}

void StartTimer1()
{
    TMR1 = 0;
    T1CONSET = 0x8000;
    PR1 = 0xFFFF;
}

void StopTimer1()
{
    T1CONCLR = 0x8000;
}
/*******************************************************************************
 End of File
 */