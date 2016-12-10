/*******************************************************************************
  MPLAB Harmony Application Header File
  Company:
    Microchip Technology Inc.
  File Name:
    ir_sensor.h
  Summary:
    This header file provides prototypes and definitions for the application.
  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
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
//DOM-IGNORE-END

#ifndef _IR_SENSOR_H
#define _IR_SENSOR_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "timers.h"
#include "queue.h"
#include "debug.h"
#include "serializer.h"
#include "navigation.h"

// DOM-IGNORE-BEGIN
#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif
// DOM-IGNORE-END 

// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application states
  Summary:
    Application states enumeration
  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
	/* Application's state machine's initial state. */
	IR_SENSOR_STATE_INIT=0,
	IR_SENSOR_STATE_SERVICE_TASKS,

	/* TODO: Define states used by the application state machine. */

} IR_SENSOR_STATES;


// *****************************************************************************
/* Application Data
  Summary:
    Holds application data
  Description:
    This structure holds the application's data.
  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* The application's current state */
    IR_SENSOR_STATES state;

    /* TODO: Define any additional data used by the application. */
    DRV_HANDLE handleTimer0;

} IR_SENSOR_DATA;


int gIrClock;
IR_SENSOR_DATA gIrSensorData;

//int g_counter[8];
int gResult[6];
int gSensorRead;
int gLines;
int gState;
int gIrData;
int gRightLeft; // 0 is right 1 is left
int gApproachLeave; // 0 is approach 1 is leave

//0: charging
//1: charged
//2: discharging
//3: discharged
int gArrayState[8];
// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/
	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void IR_SENSOR_Initialize ( void )
  Summary:
     MPLAB Harmony application initialization routine.
  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.
  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").
  Parameters:
    None.
  Returns:
    None.
  Example:
    <code>
    IR_SENSOR_Initialize();
    </code>
  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void IR_SENSOR_Initialize ( void );


/*******************************************************************************
  Function:
    void IR_SENSOR_Tasks ( void )
  Summary:
    MPLAB Harmony Demo application tasks function
  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.
  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.
  Parameters:
    None.
  Returns:
    None.
  Example:
    <code>
    IR_SENSOR_Tasks();
    </code>
  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void IR_SENSOR_Tasks( void );


void ChargeIrSensor(int sensor_num, PORTS_CHANNEL port_channel);


/******************************************************************************
 Function:
    void discharge_IR_sensor(int sensor_num, PORTS_CHANNEL port_channel)
  
 Arguments:
   int sensor_num: Pin number of the sensor you want to discharge
   PORTS_CHANNEL port_channel: Channel of the Pin you want to discharge
   
 Remarks:
 * Sensors need to be plugged into the J7-A8 to 15 where #1 is A8 #2 is A9 etc....
 */
void DischargeIrSensor(int sensorNum, PORTS_CHANNEL portChannel);

void ReadSensor();

int ConvertIrArray();

bool MoveScout();

void WaitDesiredTime(int delay);

void StopTimer1();

void StartTimer1();

void FollowLine();

#endif /* _IR_SENSOR_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */