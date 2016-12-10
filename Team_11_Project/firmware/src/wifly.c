/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    wifly.c

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

#include "wifly.h"

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

WIFLY_DATA wiflyData;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
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
char ReceiveCharFromWifly()
{    
    return PLIB_USART_ReceiverByteReceive(USART_ID_1);
}

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
bool ReceiveMessageFromWifly()
{
    char new_char = ReceiveCharFromWifly();
    dbgOutputLoc(new_char);

    // enters state machine
    switch(gReceiveStruct.receiveState)
    {
        // this state looks for the VALUE_CONTAINER character so that
        // it knows that it knows it is a proper serialized message
        // and not something else
        case GET_MESSAGE_HEADER:
            if(new_char == VALUE_CONTAINER)
            {
                ResetReceiveStruct();
                gReceiveStruct.receiveState = GET_ARRAY_SIZE;
            }
            break;

        // this state gets the size of the message being sent, as the 
        // character immediately after the VALUE_CONTAINER should be
        // the size of the message.
        case GET_ARRAY_SIZE:
            gReceiveStruct.messageSize = new_char;
            gReceiveStruct.receiveState = GET_MESSAGE;
            break;

        // this builds the message by concatenating every following 
        // character onto the message.
        case GET_MESSAGE:
            gReceiveStruct.message[gReceiveStruct.messagePlace] = new_char;
            gReceiveStruct.messagePlace++;
            if(gReceiveStruct.messagePlace == gReceiveStruct.messageSize)
            {
                gReceiveStruct.receiveState = GET_MESSAGE_FOOTER;
            }
            break;

        // once the message has been completely received, the VALUE_CONTAINER
        // character is looked for as the footer. If it is not there, there
        // have been an error of some sort in transmission.
        case GET_MESSAGE_FOOTER:
            gReceiveStruct.receiveState = GET_MESSAGE_HEADER;
            if(new_char == VALUE_CONTAINER)
            {
                return true;
            }
            
            break;

        default:
            dbgOutputLoc(240);
            break;
    }

    return false;
}

/******************************************************************************
 * Function:
 *      RsetReceiveStruct()
 * 
 * Remarks:
 *      Resets the member variables of ReceiveStruct for the next message
 */
void ResetReceiveStruct()
{
    int i;
    for(i = 0; i < gReceiveStruct.messageSize; i++)
    {
        gReceiveStruct.message[i] = '\0';
    }
    
    gReceiveStruct.messageSize = 0;
    gReceiveStruct.messagePlace = 0;
}

/******************************************************************************
 * Function:
 *      TransmitCharToWifly()
 * 
 * Remarks:
 *      Sends a character over UART to the wifly
 */
void TransmitCharToWifly(unsigned char value)
{
    while(PLIB_USART_TransmitterBufferIsFull(USART_ID_1)) 
    {
    }
    PLIB_USART_TransmitterByteSend(USART_ID_1, value);
}

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
void TransmitMessageToWifly(char* message, uint8_t length)
{    
    // transmits VALUE_CONTAINER as header followed by
    // the length of the message
    TransmitCharToWifly(VALUE_CONTAINER);
    TransmitCharToWifly(length);

    uint8_t i;
    for (i = 0; i < length; i++)
    {
        TransmitCharToWifly(message[i]);
    }

    // transmits VALUE_CONTAINER as footer
    TransmitCharToWifly(VALUE_CONTAINER);

    // clears the transmit interrupt flag and disables the transmit
    // interrupt
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceDisable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
}

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
void AddTransmitMessageToQueue(QueueMessage qMsg)
{
    AddTaskToQueue(qMsg, MsgQueueProcPicTx);
    PLIB_INT_SourceEnable(USART_ID_1, INT_SOURCE_USART_1_TRANSMIT);
}

/******************************************************************************
 Function:
    void AddTaskToQueue(QueueMessage newTask)
  
 Arguments:
    QueueMessage Pointer to task to be added to queue
 * 
 Remarks:
 * Adds new task to queue
 */
void AddTaskToQueue(QueueMessage newTask, QueueHandle_t queue)
{
    //add to queue, if it is full send an error
    if(xQueueSendToFront(queue, (void *) &newTask, (TickType_t) 30) 
            == errQUEUE_FULL)
    {
      dbgOutputLoc(DLOC_ERROR);  
    }
}

/******************************************************************************
 Function:
    void AddTaskToQueueISR(QueueMessage newTask)
  
 Arguments:
    const void *newTask: Pointer to task to be added to queue
 * 
 Remarks:
 * Adds new task to queue from ISR
 */
void AddTaskToQueueISR(QueueMessage newTask, QueueHandle_t queue)
{
    BaseType_t *pxHigherPriorityTaskWoken = pdFALSE;
    //add to queue, if it is full send an error
    if(xQueueSendFromISR(queue, (void *) &newTask, pxHigherPriorityTaskWoken) 
            == errQUEUE_FULL)
    {
      dbgOutputLoc(DLOC_ERROR);  
    }
}

void DelayMessageSending(QueueMessage qMsg)
{
    if(gMessageSendDelay == 4000)
    {
        AddTransmitMessageToQueue(qMsg);
        gMessageSendDelay = 0;
    }
}

/******************************************************************************
 Function:
    void MoveForwardNumberOfLines(int numberOfLines)
  
 Arguments:
 int numberOfLines: the number of lines to move forward
 * 
 Remarks:
 * Moves the rover forward on the grid the desired number of lines
 */
void MoveForwardNumberOfLines(int numberOfLines)
{
    int i = 0;
    if(gIrData == 222222 && numberOfLines == 1 && gPaperDrawingState == DRAWING)
    {
        return;
    }
    
    while(1)
    {
        // resets the sensor data and reads the sensor for new data
        if (gSensorRead == 1)
        {
            gSensorRead = 0;
            gIrData = ConvertIrArray();
        }
        ReadSensor();

        // the rover is at an intersection, then it moves forward until it
        // is off of the intersection
        if (gIrData == 222222)
        {
            gApproachLeave = 1;
            AdjustForward();
        }
        else if (gIrData == 111111 && gNavigationState == RETURN)
        {
            return;
        }
        // otherwise it calls FollowLine() until it hits an intersection
        else if (gIrData != 222222)
        {
            gApproachLeave = 0;
            FollowLine();
            
            // if the sensor hits a state where it needs to cancel, it causes
            // state machines to enter a cancel state
            if(gSendCancel)
            {
                gPaperDrawingState = CANCEL;
                gNavigationState = CANCEL_NAV;
                return;
            }
            
            i++;
        }
        
        // once the counter == numberOfLine, exits function
        if (i == numberOfLines)
        {
            return;
        }
    }
}

/******************************************************************************
 Function:
    void DrawBotArmHandshake(QueueMessage message)
  
 Arguments:
 QueueMessage message: the message to be sent to the draw arm
 * 
 Remarks:
 * Performs the handshake with the drawing arm
 */
void DrawBotArmHandshake(QueueMessage message)
{    
    QueueMessage qMsg, qMsgReady;
    struct Status temp;
    gMessageSendDelay = 0;
    AddTransmitMessageToQueue(message);

    while(1)
    {        
        DelayMessageSending(message);
        if(xQueueReceive(MsgQueueProcPicRx, &qMsg, (TickType_t) 0))
        {
            int message_id = deserialize(qMsg.message, qMsg.message_size);
            if (message_id == STRUCT_STATUS)
            {
                // if start command issued, starts moving to that coordinate
                if (status.state == STATUS_READY || status.state == STATUS_FINISHED)
                {
                    return;
                }
                else if (status.state == STATUS_WAITING)
                {
                    temp.state = STATUS_READY;
                    temp.progress = gLineCounter;
                    qMsgReady.message_size = serializeStatusStruct(qMsgReady.message, &temp, TARGET_PROCESS);
                    AddTransmitMessageToQueue(qMsgReady);
                    return;
                }
                else if (status.state == STATUS_RUNNING)
                {
                    if (status.progress == 1)
                    {
                        gDrawMovement = true;
                    }
                    else if (status.progress == 0)
                    {
                        gDrawMovement = false;
                    }
                    return;
                }
            }
        }                   
    }
}

QueueMessage DrawPaper(void)
{
    QueueMessage qMsg, qMsgReady, qStatusCancel, qStatusFinish, 
            qMsgWaiting, qArmFinish, qMsgProc, qMsgArm;
    int i, j;
    struct Status temp;
    int counter;
    bool turn_left_right = false;    //true for right, false for left
    struct DrawMovementCommand tempCoordinate;
    tempCoordinate = drawMovementCommand;
    gLineCounter = 0;    
    
    // builds canceled message for processing pic
    temp.state = STATUS_CANCELED;
    temp.progress = 0;
    qStatusCancel.message_size = serializeStatusStruct(qStatusCancel.message, &temp, TARGET_PROCESS);
    
    // builds finished message for processing pic
    temp.state = STATUS_FINISHED;
    temp.progress = 0;
    qStatusFinish.message_size = serializeStatusStruct(qStatusFinish.message, &temp, TARGET_PROCESS);
    
    // builds finished message for draw arm
    temp.state = STATUS_FINISHED;
    temp.progress = 0;
    qArmFinish.message_size = serializeStatusStruct(qArmFinish.message, &temp, TARGET_DBARM);
    
    // builds waiting message for draw arm
    temp.state = STATUS_WAITING;
    temp.progress = 0;
    qMsgWaiting.message_size = serializeStatusStruct(qMsgWaiting.message, &temp, TARGET_DBARM);  
    
    while(1)
    {
        switch(gPaperDrawingState)
        {
            case READY:
                MoveStop();      
                // builds ready message for processing pic
                temp.state = STATUS_READY;
                temp.progress = gLineCounter;
                qMsgReady.message_size = serializeStatusStruct(qMsgReady.message, &temp, TARGET_PROCESS);
                DelayMessageSending(qMsgReady);
               
                gCurrentPosition.y = drawMovementCommand.y;
                // determines current position based off of which side of the 
                // canvas that the rover is on. EAST means it is on the higher
                // x side, and WEST means lower x side.
                if (gCurrentDirection == EAST)
                {
                    gCurrentPosition.x = (drawMovementCommand.x - 2) + 3 + drawMovementCommand.distance;
                }
                else
                {
                    gCurrentPosition.x = drawMovementCommand.x - 1 - drawMovementCommand.distance;
                }
                            
                // waits to receive start message from processing pic to move 
                // to next coordinate
                if(xQueueReceive(MsgQueueProcPicRx, &qMsg, (TickType_t) 0))
                {
                    int message_id = deserialize(qMsg.message, qMsg.message_size);
                    if (message_id == STRUCT_DRAWMOVE)
                    {
                        gPaperDrawingState = MOVING;
                    }
                    else if (message_id == STRUCT_COMMAND)
                    {
                        if (command.message == COMMAND_CANCEL)
                        {
                            return qStatusCancel;
                        }
                    }
                    else if (message_id == STRUCT_STATUS)
                    {
                        if(status.state == STATUS_FINISHED)
                        {
                            // performs handshake with draw arm and sends finished
                            // message to processing
                            DrawBotArmHandshake(qArmFinish);
                            return qStatusFinish; 
                        }
                    }
                }                
                break;
            
            case MOVING:     
                // turns right or left depending on if the drawbot is above
                // or below the canvas                                   
                if(turn_left_right)
                {
                    MoveRight();                   
                }
                else
                {
                    MoveLeft();
                }  
                                
                // moves the rover how ever many squares are necessary to reach
                // the next coordinate
                MoveForwardNumberOfLines(drawMovementCommand.y - gCurrentPosition.y);
                if(gPaperDrawingState == CANCEL)
                {
                    return;
                }
                
                for(i = 0; i < 3; i++)
                {
                   MoveForward();
                }
                
                
                // turns rover to face canvas again            
                if(turn_left_right)
                {
                    MoveRight();                  
                }
                else
                {
                    MoveLeft();
                }  
                
                
                // performs handshake with draw bot, updates current position
                // and changes state
                turn_left_right = !turn_left_right;
                DrawBotArmHandshake(qMsgWaiting);                
                gPaperDrawingState = DRAWING;
                break;
                
                
            case DRAWING:                
                // builds ready message and sends to processing pic
                temp.state = STATUS_RUNNING;
                temp.progress = gLineCounter;
                qMsgProc.message_size = serializeStatusStruct(qMsgProc.message, &temp, TARGET_PROCESS);
                
                MoveForwardNumberOfLines(2); 
                if(gPaperDrawingState == CANCEL)
                {
                    return;
                }
                
                // moves the width of the paper 1cm at a time, sending messages
                // to draw arm and processing every grid square for status
                for(counter = 0; counter < drawMovementCommand.distance; counter++)
                {
                    // transmits which grid square the rover is on to draw arm
                    // to keep in sync while drawing
                    temp.state = STATUS_RUNNING;
                    temp.progress = counter;
                    qMsgArm.message_size = serializeStatusStruct(qMsgArm.message, &temp, TARGET_DBARM);
                    DrawBotArmHandshake(qMsgArm);
                    
                    for(i = 0; i < GRID_SQUARE_SIZE; i++)
                    {
                        if (gDrawMovement)
                        {
                            MoveForwardDrawing();
                        }
                        else
                        {
                            MoveForward();
                        }
                    }
                    
                    AddTransmitMessageToQueue(qMsgProc);
//                    if(counter % 3 == 0)
//                    {
//                        for(j = 0; j < 3; j++)
//                        {
//                            AdjustLeft();
//                        }
//                    }
//                    WaitDesiredTime(50000);
                }
                
                for(i = 0; i < 3; i++)
                {
                    MoveForward();
                }
                
                MoveForwardNumberOfLines(2);
                if(gPaperDrawingState == CANCEL)
                {
                    return;
                }
                
                for(i = 0; i < 3; i++)
                {
                   MoveForward();
                }                
                
                gLineCounter++;
                temp.state = STATUS_READY;
                temp.progress = gLineCounter;
                qMsgReady.message_size = serializeStatusStruct(qMsgReady.message, &temp, TARGET_PROCESS);
                AddTransmitMessageToQueue(qMsgReady);
                
                gMessageSendDelay = 0;
                gDrawMovement = false;
                gPaperDrawingState = READY_NAV;
                break;
                
            default:
                break;
        }
    }
}

void NavigateGrid(void)
{
    QueueMessage qMsg, qMsgReady, qMsgWaiting, qMsgDone, qStatusCancel;
    struct Command temp_command;
    struct Status temp;
    int counter = 0;
    int i;
    int starting_y;
    enum Direction original_direction;
    
    temp.state = STATUS_READY;
    temp.progress = 0;
    qMsgReady.message_size = serializeStatusStruct(qMsgReady.message, &temp, TARGET_PROCESS);
    
    // builds ready message and sends to processing pic
    temp.state = STATUS_WAITING;
    temp.progress = 0;
    qMsgWaiting.message_size = serializeStatusStruct(qMsgWaiting.message, &temp, TARGET_DBARM);
    
    // builds canceled message for processing pic
    temp.state = STATUS_CANCELED;
    temp.progress = 0;
    qStatusCancel.message_size = serializeStatusStruct(qStatusCancel.message, &temp, TARGET_PROCESS);
    
    while(1)
    {
        switch(gNavigationState)
        {
            case READY_NAV:
                MoveStop();
                // waits to receive start command from processing pic to start
                // its entire job
                if(xQueueReceive(MsgQueueProcPicRx, &qMsg, (TickType_t) 0))
                    {
                        int message_id = deserialize(qMsg.message, qMsg.message_size);
                        if (message_id == STRUCT_COMMAND)
                        {
                            // if start command issued, starts moving to that coordinate
                            if (command.message == COMMAND_START)
                            {     
                                AddTransmitMessageToQueue(qMsgReady);
                                gMessageSendDelay = 0;
                                gNavigationState = COORDINATE;
                            }
                            // if cancel command issued, moves to return state
                            else if (command.message == COMMAND_CANCEL)
                            {
                                AddTransmitMessageToQueue(qStatusCancel);
                            }
                        }
                    }
                break;
                
            case COORDINATE:
                // waits to receive coordinate from processing pic
                DelayMessageSending(qMsgReady);
                if(xQueueReceive(MsgQueueProcPicRx, &qMsg, (TickType_t) 0))
                {
                    int message_id = deserialize(qMsg.message, qMsg.message_size);
                    if (message_id == STRUCT_DRAWMOVE)
                    {
                        gNavigationState = MOVING_NAV;
                    }
                }
                break;

            case MOVING_NAV: 
                
               MoveForwardNumberOfLines(drawMovementCommand.x - gHomePosition.x - 1);
               if(gNavigationState == CANCEL_NAV)
                {
                    break;
                }
               
               for(i = 0; i < 3; i++)
                {
                   MoveForward();
                } 
                
                if(drawMovementCommand.y > 1)
                {
                    // turns to face north if paper is above home row
                    MoveLeft();
                    MoveForwardNumberOfLines(drawMovementCommand.y - gHomePosition.y);
                    if(gNavigationState == CANCEL_NAV)
                    {
                        break;
                    }
                    
                    for(i = 0; i < 3; i++)
                    {
                       MoveForward();
                    } 

                    // turns the rover right, then determines new facing direction
                    MoveRight();
                }
                
                gCurrentPosition.x = drawMovementCommand.x - 2;
                gCurrentPosition.y = drawMovementCommand.y;
                starting_y = drawMovementCommand.y;
                gNavigationState = HANDSHAKE;
                break;

            case HANDSHAKE: 
                // performs handshake, then moves to next state
                DrawBotArmHandshake(qMsgWaiting);
                FollowLine();
                if(gNavigationState == CANCEL_NAV)
                {
                    break;
                }
                gNavigationState = DRAWING_NAV;
                break;
                
            case DRAWING_NAV:
                gPaperDrawingState = DRAWING;
                qMsgDone = DrawPaper();
                
                if(gSendCancel)
                {
                    break;
                }
                
                gNavigationState = RETURN;
                break;

            case RETURN:
                // this orients the rover SOUTH to ensure that movement logic
                // to return to home is consistent
                
                original_direction = gCurrentDirection;
                
                if(original_direction == WEST)
                {
                    MoveLeft();
                }
                else if (original_direction == EAST)
                {
                    MoveRight();
                }
                else if (original_direction == NORTH)
                {
                    for(counter = 0; counter < 2; counter++)
                    {
                        MoveLeft();
                    }
                }
                
                // moves rover to home row
                MoveForwardNumberOfLines(gCurrentPosition.y - gHomePosition.y);
                if(gNavigationState == CANCEL_NAV)
                {
                    break;
                }
                
                for(i = 0; i < 3; i++)
                {
                   MoveForward();
                }
                
                // returns home if the paper is on my home row
                if(starting_y == gHomePosition.y)
                {
                    switch(original_direction)
                    {
                        case EAST:
                            MoveRight();
                            MoveForwardNumberOfLines(2);
                            if(gNavigationState == CANCEL_NAV)
                            {
                                break;
                            }

                            for(counter = 0; counter < drawMovementCommand.distance; counter++)
                            {
                                for(i = 0; i < GRID_SQUARE_SIZE; i++)
                                {
                                   MoveForward();
                                }
                            }

                            for(i = 0; i < 3; i++)
                            {
                               MoveForward();
                            } 

                            MoveForwardNumberOfLines(11 - drawMovementCommand.distance);
                            if(gNavigationState == CANCEL_NAV)
                            {
                                break;
                            }
                            
                        case WEST:
                            // turns the rover right to move along x axis
                            MoveRight();
                            MoveForwardNumberOfLines(13);
                            if(gNavigationState == CANCEL_NAV)
                            {
                                break;
                            }   
                            
                        default:
                            break;
                    }
                }
                else
                {
                    // turns the rover right to move along x axis
                    MoveRight();
                    MoveForwardNumberOfLines(20);
                    if(gNavigationState == CANCEL_NAV)
                    {
                        break;
                    }  
                }
                
                for(i = 0; i < 2; i++)
                {
                   MoveForward();
                }
                
                for(counter = 0; counter < 2; counter++)
                {
                    MoveLeft();
                }
                
                AddTransmitMessageToQueue(qMsgDone);
                gNavigationState = READY_NAV;
                break;
                
            case CANCEL_NAV:
                temp_command.message = COMMAND_CANCEL;
                QueueMessage qCommandCancel;
                qCommandCancel.message_size = serializeCommandStruct(qCommandCancel.message, &temp_command, TARGET_PROCESS);
                AddTransmitMessageToQueue(qCommandCancel);
                
                gNavigationState = READY_NAV;
                break;
                
            default:
                break;
        }
    }
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void WIFLY_Initialize ( void )

  Remarks:
    See prototype in wifly.h.
 */

void WIFLY_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    wiflyData.state = WIFLY_STATE_INIT;
    
    //setup queue
    MsgQueueProcPicTx = xQueueCreate(20, sizeof(QueueMessage));
    MsgQueueProcPicRx = xQueueCreate(20, sizeof(QueueMessage));
    
    //check if queue init correctly
    if(MsgQueueProcPicTx == 0) 
    {
        dbgOutputLoc(DLOC_ERROR);
        wiflyData.state = WIFLY_STATE_ERROR;
    }
    
    //check if queue init correctly
    if(MsgQueueProcPicRx == 0) 
    {
        dbgOutputLoc(DLOC_ERROR);
        wiflyData.state = WIFLY_STATE_ERROR;
    }
    
    gReceiveStruct.receiveState = GET_MESSAGE_HEADER;
    gNavigationState = READY_NAV;
    gCurrentDirection = EAST;
    gMessageSendDelay = 0;
    
    // sets home position to
    gHomePosition.x = 0;
    gHomePosition.y = 1;
    gCurrentPosition = gHomePosition;
    gCurrentTurning = STRAIGHT;
    gSendCancel = false;
    gDrawMovement = false;
    
    gMotorSpeed.rightSpeed = 400;
    gMotorSpeed.leftSpeed = 475; 
}


/******************************************************************************
  Function:
    void WIFLY_Tasks ( void )

  Remarks:
    See prototype in wifly.h.
 */

void WIFLY_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( wiflyData.state )
    {
        /* Application's initial state. */
        case WIFLY_STATE_INIT:
        {
            bool appInitialized = true; 
            
            DRV_USART0_Open(USART_ID_1, DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_NONBLOCKING);
            InitRoverMotors();
            DRV_TMR0_Start();
            DRV_TMR1_Start();
            DRV_TMR2_Start();
        
            if (appInitialized)
            {            
                wiflyData.state = WIFLY_STATE_SERVICE_TASKS;
            }
            break;
        }

        case WIFLY_STATE_SERVICE_TASKS:
        {
            NavigateGrid();
            break;
        }
        
        case WIFLY_STATE_ERROR:
        {
            dbgOutputLoc(DLOC_ERROR);
        }
        
        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
} 

/*******************************************************************************
 End of File
 */
