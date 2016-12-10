/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    wifly.h

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

#ifndef _WIFLY_H
#define _WIFLY_H

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
#include "queue.h"
#include "debug.h"
#include "serializer.h"
#include "motors.h"
#include "navigation.h"
#include "IR_Sensor.h"

#include "peripheral/ports/plib_ports.h"
#include "peripheral/tmr/plib_tmr.h"
#include "peripheral/oc/plib_oc.h"

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
	WIFLY_STATE_INIT=0,
	WIFLY_STATE_SERVICE_TASKS,
    WIFLY_STATE_ERROR

	/* TODO: Define states used by the application state machine. */

} WIFLY_STATES;

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
    WIFLY_STATES state;
} WIFLY_DATA;

// the message to be used in both receive and transmit queues
typedef struct
{
    // the message being sent into the queue, with
    // buffer size = to MAX_MESSAGE_SIZE
    uint8_t message[MAX_MESSAGE_SIZE];
    // the size of the message in the queue
    uint8_t message_size;
} QueueMessage;

// enum for the state machine to be used when receiving
// serialized data over the wifly via UART
typedef enum
{
    GET_MESSAGE_HEADER,
    GET_ARRAY_SIZE,
    GET_MESSAGE,
    GET_MESSAGE_FOOTER
} ReceiveState;

// struct containing information to be used when receiving serialized
// data over the wifly via UART
struct receiveStruct
{
    // the current state the receive is in
    ReceiveState receiveState;
    // the size of the message being received
    uint8_t messageSize;
    // the message itself
    uint8_t message[MAX_MESSAGE_SIZE];
    // where in the message the receive method is
    uint8_t messagePlace;
} gReceiveStruct;

enum paperDrawingState {
    // waiting for next location from processing pic
    READY, 
    // moving to next coordinate, then doing handshake with drawing arm
    MOVING, 
    // moving across the paper
    DRAWING,
    CANCEL
} gPaperDrawingState;

enum navigationState {
    // waiting at home position for start command from processing pic
    READY_NAV, 
    // looks for the coordinate from the processing pic
    COORDINATE,
    // moving to location as specified by processing pic
    MOVING_NAV, 
    // makes handshake with drawing arm for first line
    HANDSHAKE, 
    // draws paper by entering gPaperDrawingStateMachine
    DRAWING_NAV, 
    // returns to home position after either a cancel or when done drawing
    RETURN,
    CANCEL_NAV
} gNavigationState;

// the transmit queue
QueueHandle_t MsgQueueProcPicTx;

// the receive queue
QueueHandle_t MsgQueueProcPicRx;

// boolean governing if the control states need to cancel
bool gSendCancel;

// counts the number of lines drawn
int gLineCounter;

bool gDrawMovement;

// counter used to delay the sending of messages
volatile uint32_t gMessageSendDelay;


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

/******************************************************************************
 * Function:
 *      ReceiveCharFromWifly()
 * 
 * Remarks:
 *      Receives a character from the wifly via USART
 * 
 * Returns:
 *      The character received from the wifly
 */
char ReceiveCharFromWifly();

/******************************************************************************
 * Function:
 *      ReceiveMessageFromWifly()
 *  
 * Remarks:
 *      This uses a state machine to receive a serialized message from the
 *      wifly via UART. 
 * 
 * Returns:
 *      True when the entire message has been received, otherwise returns false
 */
bool ReceiveMessageFromWifly();

/******************************************************************************
 * Function:
 *      RsetReceiveStruct()
 * 
 * Remarks:
 *      Resets the member variables of ReceiveStruct for the next message
 */
void ResetReceiveStruct();

/******************************************************************************
 * Function:
 *      TransmitCharToWifly()
 * 
 * Remarks:
 *      Sends a character over UART to the wifly
 */
void TransmitCharToWifly(uint8_t value);

/******************************************************************************
 * Function:
 *      TransmitMessageToWifly()
 * 
 * Remarks:
 *      Transmits an entire serialized message over UART to the wifly
 * 
 * Arguments:
 *      char* message: the message to be sent
 *      uint8_t length: the length of the message to be sent
 */
void TransmitMessageToWifly(char* message, uint8_t length);

/******************************************************************************
 * Function:
 *      AddTransmitMessageToQueue()
 * 
 * Remarks:
 *      Adds a task to the Tx queue
 * 
 * Arguments:
 *      QueueMessage qMsg: the struct containing the message and size to be
 *                          transmitted
 */
void AddTransmitMessageToQueue(QueueMessage qMsg);

/******************************************************************************
 Function:
    void AddTaskToQueue(const void *newTask)
  
 Arguments:
    const void *newTask: Pointer to task to be added to queue
 * 
 Remarks:
 * Adds new task to queue
 */
void AddTaskToQueue(QueueMessage newTask, QueueHandle_t queue);

/******************************************************************************
 Function:
    void AddTaskToQueueISR(const void *newTask)
  
 Arguments:
    const void *newTask: Pointer to task to be added to queue
 * 
 Remarks:
 * Adds new task to queue from ISR
 */
void AddTaskToQueueISR(QueueMessage newTask, QueueHandle_t queue);

void DrawBotArmHandshake(QueueMessage message);
QueueMessage DrawPaper(void);
void NavigateGrid(void);
void DelayMessageSending(QueueMessage qMsg);

/*******************************************************************************
  Function:
    void WIFLY_Initialize ( void )

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
    WIFLY_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void WIFLY_Initialize ( void );


/*******************************************************************************
  Function:
    void WIFLY_Tasks ( void )

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
    WIFLY_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void WIFLY_Tasks( void );

#endif /* _WIFLY_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */

