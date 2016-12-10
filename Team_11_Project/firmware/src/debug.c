/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    debug.c

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

#include "debug.h"
/*******************************************************************************
  Function:
    void dbgOutputLoc( unsigned char outVal )

  Remarks:
    Method that sets GPIO pins 30-37 on J9 to an 8 bit value
 */ 
void dbgOutputLoc(unsigned char outVal)
{
    // this reads the current value from Port E
    uint32_t port_e = SYS_PORTS_Read(PORTS_ID_0, PORT_CHANNEL_E);
    
    // this sets the lower 8 bits of Port E to the value being passed in
    // pins 30-37 on the Pic32. 0 = 37 and 7 = 30.
    port_e = outVal | (port_e & 0xFFFFFF00);
    
    // this writes the new value of port E to port E
    SYS_PORTS_Write(PORTS_ID_0, PORT_CHANNEL_E, port_e);    
}

/*******************************************************************************
  Function:
    void dbgOutputVal( unsigned char outVal )

  Remarks:
    Method that sets GPIO pins 46-53 on J8 to an 8 bit value
 */ 
void dbgOutputVal(unsigned char outVal)
{    
    // Sets pins F-1, D-6, D-8, D-11, G-7, G-8, G-6, G-9
    // Pins 46-53 on J8 on the Pic32MX    
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_F, PORTS_BIT_POS_1, outVal >> 7);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_6, (outVal >> 6) & 0x01);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_8, (outVal >> 5) & 0x01);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_D, PORTS_BIT_POS_11, (outVal >> 4) & 0x01);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_7, (outVal >> 3) & 0x01);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_8, (outVal >> 2) & 0x01);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_6, (outVal >> 1) & 0x01);
    SYS_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_G, PORTS_BIT_POS_9, outVal & 0x01);
}
/*******************************************************************************
 End of File
 */
