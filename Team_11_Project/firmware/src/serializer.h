/* ************************************************************************** */
/**
  @File Name
    serializer.h

  @Summary
 Takes message data formats and serializes them
 */
/* ************************************************************************** */

#ifndef _SERIALIZER_H    /* Guard against multiple inclusion */
#define _SERIALIZER_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif


    /* ************************************************************************** */
    /* ************************************************************************** */
    /* Section: Constants                                                         */
    /* ************************************************************************** */
    /* ************************************************************************** */
    /**
      @Summary
     Max size of the message array
     */
    #define MAX_MESSAGE_SIZE 100
    /**
      @Summary
     Size of the keys in char array
     */
    #define KEY_SIZE 4
    /**
      @Summary
     Constant used to determine end of value in message
     * Used in serialization and deserialization functions
     * DECLARE A GLOBAL VALUE DEFINING YOUR PIC TO USE THE SERIALIZE/DESERIALIZE FUNCTIONS
     */
    #define VALUE_CONTAINER '~'
    /**
      @Summary
     Constants for errors that can occur in deserialization
     */
     #define ERROR_BAD_SIZE     -1  //the size of the message is too small
     #define ERROR_WRONG_TARGET -2  //the target id is not equal to the global id
     #define ERROR_NO_STRUCT    -3  //the struct id is incorrect

    /**
      @Summary
     Constants used to define the what struct is being used
     * used in serialization/deserialization functions
     * DECLARE A GLOBAL VALUE DEFINING YOUR PIC TO USE THE SERIALIZE/DESERIALIZE FUNCTIONS
     */
    #define STRUCT_STATUS       0
    #define STRUCT_COMMAND      1
    #define STRUCT_SHAPE        2
    #define STRUCT_COORDINATE   3
    #define STRUCT_DRAWARM      4
    #define STRUCT_DRAWMOVE     5
    #define STRUCT_IRSENSOR     6
    /**
      @Summary
     Constants used to define the source and target that sends/receives the message
     * Used in getPICID(uint32_t message)
     * DECLARE A GLOBAL VALUE DEFINING YOUR PIC TO USE THE SERIALIZE/DESERIALIZE FUNCTIONS
     */
    #define SOURCE_PROCESS     0   //Processing PIC
    #define SOURCE_DBMOVE      1   //DoodleBot Move PIC
    #define SOURCE_DBARM       2   //DoodleBot Arm Control PIC
    #define SOURCE_SCOUTBOT    3   //ScoutBot PIC
    #define SOURCE_PI          4   //Raspberry Pi

    #define TARGET_PROCESS     0   //Processing PIC
    #define TARGET_DBMOVE      1   //DoodleBot Move PIC
    #define TARGET_DBARM       2   //DoodleBot Arm Control PIC
    #define TARGET_SCOUTBOT    3   //ScoutBot PIC
    #define TARGET_PI          4   //Raspberry Pi
    /**
      @Summary
     Constants used to define the state sent from the PICs
     * See "status" struct for use case details
     */
    #define STATUS_READY       0   //the bot is in the start position and awaiting initial command.
    #define STATUS_RUNNING     1   //the bot is executing its current command.
    #define STATUS_WAITING     2   //the bot has completed its current command and is awaiting the next one.
    #define STATUS_FINISHED    3   //the bot has completed its job and is returning to start position.
    #define STATUS_CANCELED    4   //the bot has been told to terminate its commands and is returning to start position.
    #define STATUS_ERROR       5   //the bot has encountered an error.
    /**
      @Summary
     Constants used to define the command sent to the PICs.
     * See "command" struct for use case details
     */
    #define COMMAND_START   0   //start or continue a job: sent prior to all commands to guarantee proper message sending.
    #define COMMAND_CANCEL  1   //terminate a job: can be user created or by massive error
    /**
      @Summary
     Constants used to define the shape being drawn
     * See "shape" struct for use case details
     */
    #define SQUARE          0
    #define RTRIANGLE_BL    1   //bottom left corner
    #define RTRIANGLE_BR    2   //bottom right corner
    #define RTRIANGLE_TL    3   //top left corner
    #define RTRIANGLE_TR    4   //top right corner
    #define RECTANGLE       5
    /**
      @Summary
     Constants used to define the arm position when drawing
     * See "drawArmCommand" struct for use case details
     */
    #define ARM_UP      0
    #define ARM_DOWN    1

    // *****************************************************************************
    // *****************************************************************************
    // Section: Data Types
    // *****************************************************************************
    // *****************************************************************************

    /*  A brief description of a section can be given directly below the section
        banner.
     */

    // *****************************************************************************
    // ** GLOBAL Structs ** //
    // These structs are used across all PICs.

    /** Status

      @Summary
     Returns current status and state of the pic

     */
    struct Status {
        /* explains state of pic
         * 0: STATUS_READY     - the bot is in the start position and awaiting initial command.
         * 1: STATUS_RUNNING   - the bot is executing its current command .
         * 2: STATUS_WAITING   - the bot has completed its current command and is awaiting the next one.
         * 3: STATUS_FINISHED  - the bot has completed its job and is returning to start position.
         * 4: STATUS_CANCELED  - the bot has been told to terminate its commands and is returning to start position.
         * 5: STATUS_ERROR     - the bot has encountered an error. */
        short state;

        /* percentage through current state - used for the running stage. 1-100 */
        short progress;

    } status;

    /** Command

      @Summary
     * Sends command to bots whether to start/continue or cancel the current job.
     * Sent prior to all other commands - preps the bot

     */
    struct Command {
        /* explains command to the PIC
         * 0: COMMAND_START  - start or continue a job: sent prior to all commands to guarantee proper message sending.
         * 1: COMMAND_CANCEL - terminate a job: can be user created or by massive error*/
        short message;

    } command;

    // *****************************************************************************
    // ** PIC-Specific Structs ** //
    // These structs are sent/received depending on the PIC.
    // Details of what PIC sends/receives each struct is detailed in its summary

    /** shape

      @Summary
     * Defines the shape the doodlebot will draw
     *
     ****** Sent by the PI (USER) to the PROCESSPIC ******

     */
    struct Shape {
        /* defines the shape
         * 0: SQUARE
         * 1: RTRIANGLE_BL   - bottom left corner
         * 2: RTRIANGLE_BR   - bottom right corner
         * 3: RTRIANGLE_TL   - top left corner
         * 4: RTRIANGLE_TR   - top right corner
         * 5: RECTANGLE         */
        short type;
    } shape;

    /** coordinate

      @Summary
     * Describes x and y coordinate of corner of canvas
     * in relation to its location on the grid
     * as well as how many corners have been found
     *
     ****** Sent by the SCOUTBOT to the PROCESSPIC ******

     */
    struct Coordinate {
        /* X Coordinate on canvas */
        short x;
        /* Y Coordinate on canvas */
        short y;
        /* Current number of corners the scout bot has found */
        short count;
    } coordinate;

    /** drawArmCommand

      @Summary
     * Array of 1's and 0's determining position of arm while drawing a row
     *
     ****** Sent by the PROCESSPIC to the DOODLEBOTARM ******

     */
    struct DrawArmCommand {
        /* Length of bit array */
        int size;

        /* Bit array of arm commands
         * We are using char since it is only 1 byte
         * The max size varies on the definition of max message size and key size.
         * 0: ARM_UP
         * 1: ARM_DOWN
         */
        char armCommandArray[MAX_MESSAGE_SIZE - KEY_SIZE];
    } drawArmCommand;

    /** drawMovementCommand

      @Summary
     * Details start location and distance to travel when drawing a row
     *
     ****** Sent by the PROCESSPIC to the DOODLEBOTMOVEMENT ******

     */
    struct DrawMovementCommand {
        /* X Coordinate on canvas */
        short x;
        /* Y Coordinate on canvas */
        short y;

        /* Distance to travel */
        int distance;
    } drawMovementCommand;

    // *****************************************************************************
    // ** HARDWARE-Specific Structs ** //
    // These structs are used internally on the pic, sent from hardware to a thread.
    // Details of what hardware/PIC sends/receives each struct is detailed in its summary

    /** IRSensorArray

      @Summary
     * Provides information coming from the IR Sensor Array to a PIC
     *
     ****** Sent by the IR SENSOR ARRAY + TIMER to the DOODLEBOTMOVEMENT and SCOUTBOT ******

     */
    struct irSensorArray {
        /* Array containing information from the sensor */
        char data[8];
    } IRSensorArray;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Serialization Functions
    // *****************************************************************************
    // *****************************************************************************
    /**
      @Function
        void  createKey(char* message, uint32_t targetID, uint32_t structID)

      @Summary
      passes in the key to the beginning of the message

      @Parameters
        @param message  - the message to be sent to the FREERTOS queue
        @param targetID - the id of the target being sent
        @param structID - the id of the struct being sent

      @Example
        @code
      //creates a key detailing the status struct being sent to the target pi
      createKey(qMsg.message, TARGET_PI, STRUCT_STATUS);
     */
    void createKey(char* message, uint32_t targetID, uint32_t structID);

    /**
      @Function
        uint8_t  serializeStatusStruct(char* message, struct Status* statusStruct, uint32_t target)

      @Summary
        serializes a status struct into a char* for a queuemessage

      @Parameters
        @param message - the message to be sent to the FREERTOS queue
        @param statusStruct - the status struct to be serialized
        @param target - the target process to read this struct

      @Returns
     bool whether the serialization occurred successfully
     * true: success
     * false: failure

      @Example
        @code
        if(serializeStatusStruct(qMsg.message, status))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeStatusStruct(char* message, struct Status* statusStruct, uint32_t target);

    /**
      @Function
        uint8_t  serializeCommandStruct(char* message, struct Command* commandStruct, uint32_t target)

      @Summary
        serializes a command struct into a char* for a queuemessage
        THIS IS FROM PI TO PIC ONLY.

      @Parameters
        @param message - the message to be sent to the FREERTOS queue
        @param commandStruct - the command struct to be serialized
        @param target - the target process to read this struct

      @Returns
     bool whether the serialization occurred successfully
     * true: success
     * false: failure

      @Example
        @code
        if(serializeCommandStruct(qMsg.message, command))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeCommandStruct(char* message, struct Command* commandStruct, uint32_t target);

    /**
      @Function
        uint8_t  serializeShapeStruct(char* message, struct Shape* shapeStruct, uint32_t target)

      @Summary
        serializes a shape struct into a char* for a queuemessage

      @Parameters
        @param message - the message to be sent to the FREERTOS queue
        @param shapeStruct - the shape struct to be serialized
        @param target - the target process to read this struct

      @Returns
     bool whether the serialization occurred successfully
     * true: success
     * false: failure

      @Example
        @code
        if(serializeShapeStruct(qMsg.message, shape))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeShapeStruct(char* message, struct Shape* shapeStruct, uint32_t target);

    /**
      @Function
       uint8_t  serializeCoordinateStruct(char* message, struct Coordinate* coordinateStruct, uint32_t target)

      @Summary
        serializes a coordinate struct into a char* for a queuemessage

      @Parameters
        @param message - the message to be sent to the FREERTOS queue
        @param coordinateStruct - the coordinate struct to be serialized
        @param target - the target process to read this struct

      @Returns
     bool whether the serialization occurred successfully
     * true: success
     * false: failure

      @Example
        @code
        if(serializeCoordinateStruct(qMsg.message, coordinate))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeCoordinateStruct(char* message, struct Coordinate* coordinateStruct, uint32_t target);

    /**
      @Function
        uint8_t serializeDrawArmCommandStruct(char* message, struct DrawArmCommand* drawArmCommandStruct, uint32_t target)

      @Summary
        serializes a drawArmCommand struct into a char* for a queuemessage

      @Parameters
        @param message - the message to be sent to the FREERTOS queue
        @param drawArmCommandStruct - the drawArmCommand struct to be serialized
        @param target - the target process to read this struct

      @Returns
     bool whether the serialization occurred successfully
     * true: success
     * false: failure

      @Example
        @code
        if(serializeDrawArmCommandStruct(qMsg.message, drawArmCommand))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeDrawArmCommandStruct(char* message, struct DrawArmCommand* drawArmCommandStruct, uint32_t target);

    /**
      @Function
        uint8_t  serializeDrawMovementCommandStruct(char* message, struct DrawMovementCommand* drawMovementCommandStruct, uint32_t target)

      @Summary
        serializes a drawMovementCommand struct into a char* for a queuemessage

      @Parameters
        @param message - the message to be sent to the FREERTOS queue
        @param drawMovementCommandStruct - the drawMovementCommand struct to be serialized
        @param target - the target process to read this struct

      @Returns
     bool whether the serialization occurred successfully
     * true: success
     * false: failure

      @Example
        @code
        if(serializeDrawMovementCommandStruct(qMsg.message, drawMovementCommand))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeDrawMovementCommandStruct(char* message, struct DrawMovementCommand* drawMovementCommandStruct, uint32_t target);

    /**
      @Function
        uint8_t  serializeIRSensorArrayStruct(char* message, struct irSensorArray* irStruct, uint32_t target)

      @Summary
        serializes a irSensorArray struct into a char* for a queuemessage

      @Parameters
        @param message - the message to be sent to the FREERTOS queue
        @param irStruct - the irSensorArray struct to be serialized
        @param target - the target process to read this struct

      @Returns
     bool whether the serialization occurred successfully
     * true: success
     * false: failure

      @Example
        @code
        if(serializeIRSensorArrayStruct(qMsg.message, IRSensorArray))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeIRSensorArrayStruct(char* message, struct irSensorArray* irStruct, uint32_t target);


    // *****************************************************************************
    // *****************************************************************************
    // Section: Deserialization Functions
    // *****************************************************************************
    // *****************************************************************************
    /**
      @Function
        int deserialize(char* message, int messageSize)

      @Summary
     Takes a QueueMessage, determines correctedness and then deserializes for the structs

      @Parameters
        @param message      - the message sent from the FREERTOS queue
        @param messageSize  - the size of the message (to check for proper size) 

      @Returns
     the ID of what struct has been updated

      @Example
        @code
        if(deserialize(qMsg.message, messageSize) == STRUCT_STATUS)
        {
            //message has been recieved and is STRUCT_STATUS. Continue
        }
     */
    int deserialize(char* , int messageSize);

    /**
      @Function
        int deserializeInt(char* message, int loc, int size, int* value)

      @Summary
     Takes a char array, and from the given start location reads serialized data into int value

      @Parameters
        @param message  - the character array containing the data
        @param loc      - the start location of reference of the array
        @param size     - the number of ints being read
        @param value    - the pointer to the integer value being updated

      @Returns
         int of location in array to continue
         * -1 if building int failed

      @Example
        @code
         * //within the coordinate struct
        if //an integer value has been defined
        {
         counter++;
         counter = deserializeInt(message, 1, coordinate.x);
        }
     */
    int deserializeInt(char* message, int loc, int size, int* value);

    /**
      @Function
        int deserializeUint32_t(char* message, int loc, int size, uint32_t* value)

      @Summary
     Takes a char array, and from the given start location reads serialized data into uint array value

      @Parameters
        @param message  - the character array containing the data
        @param loc      - the start location of the reference of the array
        @param size     - the number of ints being read
        @param value    - the pointer to the uint32_t* value being updated

      @Returns
         int of location in array to continue
         * -1 if building uint failed

      @Example
        @code
         * //within the IRSensorArray struct
        if //an integer value has been defined
        {
         counter++;
         counter = deserializeUint32_t(message, 8, IRSensorArray.data);
        }
     */
    int deserializeUint32_t(char* message, int loc, int size, uint32_t* value);

    /**
      @Function
        int deserializeShort(char* message, int loc, short* value)

      @Summary
     Takes a char array, and from the given start location reads serialized data into short value

      @Parameters
        @param message  - the character array containing the data
        @param loc      - the start location of the reference of the array
        @param value    - the pointer to the short value being updated

      @Returns
         int of location in array to continue
         * -1 if building short failed

      @Example
        @code
         * //within the status struct
        if //a short value has been defined
        {
         counter++;
         counter = deserializeShort(message, status.state);
        }
     */
    int deserializeShort(char* message, int loc, short* value);

    /**
      @Function
        int deserializeChar(char* message, int loc, int size, char* value)

      @Summary
     Takes a char array, and from the given start location reads serialized data into char* value

      @Parameters
        @param message  - the character array containing the data
        @param loc      - the start location of reference of the array
        @param size     - the number of ints being read
        @param value    - the pointer to the char value being updated

      @Returns
         int of location in array to continue
         * -1 if building int failed

      @Example
        @code
         * //within the coordinate struct
        if //an integer value has been defined
        {
         counter++;
         counter = deserializeChar(message, 1, drawArmCommand.armCommandArray);
        }
     */
    int deserializeChar(char* message, int loc, int size, char* value);

    // *****************************************************************************
    // *****************************************************************************
    // Section: Debugging Functions
    // *****************************************************************************
    // *****************************************************************************
    /**
      @Function
        void debugSerializedData(char* message)

      @Summary
     Takes a QueueMessage, returns human-readable value of struct

      @Parameters
        @param message      - the message sent from the FREERTOS queue
        @param messageSize  - the size of the message being sent

      @Example
        @code
        debugSerializedData(qMsg.message);
     */
    void debugSerializedData(char* message, int messageSize);

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
