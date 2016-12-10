/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "system_definitions.h"
#include "wifly.h"
#include "debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

void IntHandlerDrvTmrInstance0(void)
{    
    gMessageSendDelay++;
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_TIMER_2);
}
    

// encoders are pins 7 -> 23 and 3 -> 22
void IntHandlerDrvTmrInstance1(void)
{
    gRightEncoder++;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_3);
}       

void IntHandlerDrvTmrInstance2(void)
{
    gLeftEncoder++;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}

void IntHandlerDrvUsartInstance0(void)
{           
    QueueMessage qMsg;
    
    // this checks if the receive interrupt has been raised
    if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE))
    {
        // if the entire message has been received, pushes the message into
        // the receive queue
//        dbgOutputLoc(40);
        if(ReceiveMessageFromWifly())
        {
            int i;
            dbgOutputLoc(gReceiveStruct.messageSize);
            for(i = 0; i < gReceiveStruct.messageSize; i++)
            {
                qMsg.message[i] = gReceiveStruct.message[i];
            }
            qMsg.message_size = gReceiveStruct.messageSize;
            AddTaskToQueueISR(qMsg, MsgQueueProcPicRx);
        }
        
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    }
    
    // if the transmit interrupt flag has been raised, it receives the message
    // from the queue and transmits it via UART to the wifly
    if (PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT))
    {   
        while(!xQueueIsQueueEmptyFromISR(MsgQueueProcPicTx))
        {
            BaseType_t xTaskWokenByReceive = pdFALSE;
            if(xQueueReceiveFromISR(MsgQueueProcPicTx, (void *) &qMsg, &xTaskWokenByReceive)
                    == pdTRUE)
            {
                dbgOutputLoc(39);
                TransmitMessageToWifly(qMsg.message, qMsg.message_size);
            }
        }
    }
    
    if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_ERROR)) 
    {
        dbgOutputLoc(DLOC_ERROR);
        PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);  
    }
}  
/*******************************************************************************
 End of File
*/

