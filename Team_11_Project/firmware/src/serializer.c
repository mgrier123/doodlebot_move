/* ************************************************************************** */
/**

  @File Name
    serializer.c

  @Summary
 Takes message data formats and serializes/deserializes them
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */

/* This section lists the other files that are included in this file.
 */

#include "serializer.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

//increments every message that is being sent, to guarantee that all messages are being received
int g_sequenceNumber = 0;

//SET THIS VALUE TO STATE WHAT DEVICE YOU ARE
//the id of the current device
uint32_t g_ID = SOURCE_DBMOVE;

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
    void createKey(char* message, uint32_t targetID, uint32_t structID) {
        //define source
        message[0] = g_ID;

        //define target
        message[1] = targetID;

        //define struct
        message[2] = structID;

        //define what message number this is
        message[3] = g_sequenceNumber % 126;

        //increment the number of messages sent
        //this only occurs if the target and source differ
        if(targetID != g_ID) {
          g_sequenceNumber++;
        }

    }

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
     uint8_t value of the size of the serialized array in bytes

      @Example
        @code
        if(serializeStatusStruct(qMsg.message, status))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeStatusStruct(char* message, struct Status* statusStruct, uint32_t target) {
        
        //first, create the key
        createKey(message, target, STRUCT_STATUS);

        char* pointer = message + KEY_SIZE;

        //next, write the values to the buffer
        memcpy(pointer,&(statusStruct->state),sizeof(short));
        pointer = pointer + sizeof(short);
        memcpy(pointer,&(statusStruct->progress),sizeof(short));


        return (KEY_SIZE + 2*sizeof(short));
    }

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
    uint8_t serializeCommandStruct(char* message, struct Command* commandStruct, uint32_t target) {
        //first, create the key
        createKey(message, target, STRUCT_COMMAND);

        char* pointer = message + KEY_SIZE;

        //next, write the values to the buffer
        memcpy(pointer,&(commandStruct->message),sizeof(short));

        return (KEY_SIZE + sizeof(short));
    }

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
     uint8_t value of the size of the serialized array in bytes

      @Example
        @code
        if(serializeShapeStruct(qMsg.message, shape))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeShapeStruct(char* message, struct Shape* shapeStruct, uint32_t target) {
        //first, create the key
        createKey(message, target, STRUCT_SHAPE);

        char* pointer = message + KEY_SIZE;

        //next, write the values to the buffer
        memcpy(pointer,&(shapeStruct->type),sizeof(short));

        return (KEY_SIZE + sizeof(short));
    }

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
     uint8_t value of the size of the serialized array in bytes

      @Example
        @code
        if(serializeCoordinateStruct(qMsg.message, coordinate))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeCoordinateStruct(char* message, struct Coordinate* coordinateStruct, uint32_t target){
        //first, create the key
        createKey(message, target, STRUCT_COORDINATE);

        char* pointer = message + KEY_SIZE;

        //next, write the values to the buffer
        memcpy(pointer,&(coordinateStruct->x),sizeof(short));
        pointer = pointer + sizeof(short);
        memcpy(pointer,&(coordinateStruct->y),sizeof(short));
        pointer = pointer + sizeof(short);
        memcpy(pointer,&(coordinateStruct->count),sizeof(short));

        return (KEY_SIZE + 3*sizeof(short));
    }

    /**
      @Function
        uint8_t  serializeDrawArmCommandStruct(char* message, struct DrawArmCommand* drawArmCommandStruct, uint32_t target)

      @Summary
        serializes a drawArmCommand struct into a char* for a queuemessage

      @Parameters
        @param message - the message to be sent to the FREERTOS queue
        @param drawArmCommandStruct - the drawArmCommand struct to be serialized
        @param target - the target process to read this struct

      @Returns
     uint8_t value of the size of the serialized array in bytes

      @Example
        @code
        if(serializeDrawArmCommandStruct(qMsg.message, drawArmCommand))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeDrawArmCommandStruct(char* message, struct DrawArmCommand* drawArmCommandStruct, uint32_t target) {
        //first, create the key
        //since this is always internal use yourself as the target.
        createKey(message, target, STRUCT_DRAWARM);

        char* pointer = message + KEY_SIZE;

        //next, write the values to the buffer
        memcpy(pointer,&(drawArmCommandStruct->size),sizeof(int));
        pointer = pointer + sizeof(int);

        //loop through entirety of array
        int i;

        for(i = 0; i < drawArmCommandStruct->size; i++) {
          memcpy(pointer,&(drawArmCommandStruct->armCommandArray[i]),sizeof(char));

          pointer = pointer + sizeof(char);
        }

        return (KEY_SIZE + sizeof(int) + drawArmCommandStruct->size * sizeof(char));
    }

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
     uint8_t value of the size of the serialized array in bytes

      @Example
        @code
        if(serializeDrawMovementCommandStruct(qMsg.message, drawMovementCommand))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeDrawMovementCommandStruct(char* message, struct DrawMovementCommand* drawMovementCommandStruct, uint32_t target) {
        //first, create the key
        createKey(message, target, STRUCT_DRAWMOVE);

        char* pointer = message + KEY_SIZE;

        //next, write the values to the buffer
        memcpy(pointer,&(drawMovementCommandStruct->x),sizeof(short));
        pointer = pointer + sizeof(short);
        memcpy(pointer,&(drawMovementCommandStruct->y),sizeof(short));
        pointer = pointer + sizeof(short);
        memcpy(pointer,&(drawMovementCommandStruct->distance),sizeof(int));

        return (KEY_SIZE + 2*sizeof(short) + sizeof(int));
    }

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
     uint8_t value of the size of the serialized array in bytes

      @Example
        @code
        if(serializeIRSensorArrayStruct(qMsg.message, IRSensorArray))
        {
            //message has been loaded. Continue
        }
     */
    uint8_t serializeIRSensorArrayStruct(char* message, struct irSensorArray* irStruct, uint32_t target) {
        //first, create the key
        //since this is always internal use yourself as the target.
        createKey(message, target, STRUCT_IRSENSOR);

        char* pointer = message + KEY_SIZE;

        //loop through entirety of array
        int i;

        for(i = 0; i < 8; i++) {
          memcpy(pointer,&(irStruct->data[i]),sizeof(char));

          pointer = pointer + sizeof(char);
        }

        return (KEY_SIZE + 8*sizeof(char));
    }


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
    int deserialize(char* message, int messageSize) {

        //first, determine if this is the correct pic
        if(messageSize < KEY_SIZE) //this array doesn't have enough info: return
        {
           return ERROR_BAD_SIZE;
        }

        //detect if the message is meant for us. If not return false
        //PI has global access to all messages
        if(message[1] == g_ID || g_ID == SOURCE_PI)
        {
            int counter = KEY_SIZE; //start location in array of actual struct data (since first four are chars)
            //determine struct being deserialized
            switch(message[2])
            {
                case STRUCT_STATUS:
                    counter = deserializeShort(message, counter, &status.state);
                    counter = deserializeShort(message, counter, &status.progress);
                    break;
                case STRUCT_COMMAND:
                    counter = deserializeShort(message, counter, &command.message);
                    break;
                case STRUCT_SHAPE:
                    counter = deserializeShort(message, counter, &shape.type);
                    break;
                case STRUCT_COORDINATE:
                    counter = deserializeShort(message, counter, &coordinate.x);
                    counter = deserializeShort(message, counter, &coordinate.y);
                    counter = deserializeShort(message, counter, &coordinate.count);
                    break;
                case STRUCT_DRAWARM:
                   counter = deserializeInt(message, counter, 1, &drawArmCommand.size);
                   counter = deserializeChar(message, counter, drawArmCommand.size, drawArmCommand.armCommandArray);
                    break;
                case STRUCT_DRAWMOVE:
                    counter = deserializeShort(message, counter, &drawMovementCommand.x);
                    counter = deserializeShort(message, counter, &drawMovementCommand.y);
                    counter = deserializeInt(message, counter, 1, &drawMovementCommand.distance);
                    break;
                case STRUCT_IRSENSOR:
                   counter = deserializeChar(message, counter, 8, IRSensorArray.data);
                    break;
                default:
                    return ERROR_NO_STRUCT; //we couldn't find the struct to deserialize
                    break;
            }

            return message[2]; //return the struct id that was updated
        }
        else
        {
            return ERROR_WRONG_TARGET;
        }
    }

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
    int deserializeInt(char* message, int loc, int size, int* value){

        int i;

        for(i = 0; i < size; i++) {
            memcpy(&value[i], message + loc, sizeof(int));

            loc = loc + sizeof(int);
        }

        return loc;
    }

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
    int deserializeUint32_t(char* message, int loc, int size, uint32_t* value) {

        int i;

        for(i = 0; i < size; i++) {
            memcpy(&value[i], message + loc, sizeof(uint32_t));

            loc = loc + sizeof(uint32_t);
        }

        return loc;
    }

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
    int deserializeShort(char* message, int loc, short* value) {
        memcpy(value, message + loc, sizeof(short));

        return loc + sizeof(short);
    }

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
    int deserializeChar(char* message, int loc, int size, char* value) {

        int i;

        for(i = 0; i < size; i++) {
            memcpy(&value[i], message + loc, sizeof(char));

            loc = loc + sizeof(char);
        }

        return loc;
    }

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
    void debugSerializedData(char* message, int messageSize) {

      int structID = deserialize(message, messageSize);

      switch(structID) {
        case ERROR_BAD_SIZE:
            printf("Error. Serialized message is too small to be deserialized properly.\n") ;
            break;
        case ERROR_NO_STRUCT:
            printf("Error. No struct ID found. Valid key IDs are from 0-6. Yours is %d.\n", message[2]);
            break;
        case ERROR_WRONG_TARGET:
            printf("Error. You do not have permission to view this struct. Try reassigning your g_ID.\n");
            break;
        case STRUCT_STATUS:
            printf("Struct: Status\n\tstate = %d\n \tprogress = %d\n",
                status.state , status.progress);
            break;
        case STRUCT_COMMAND:
            printf("Struct: Command\n\tmessage = %d\n",
                command.message);
            break;
        case STRUCT_SHAPE:
            printf("Struct: Shape\n\ttype = %d\n",
                command.message);
            break;
        case STRUCT_COORDINATE:
            printf("Struct: Coordinate\n\tx = %d\n \ty = %d\n \tcount = %d\n",
                coordinate.x, coordinate.y, coordinate.count);
            break;
        case STRUCT_DRAWARM:
            printf("Struct: Draw Arm Command\n\tsize = %d\n \tdata = [%s]\n",
                drawArmCommand.size, drawArmCommand.armCommandArray);
            break;
        case STRUCT_DRAWMOVE:
            printf("Struct: Draw Movement Command\n\tx = %d\n \ty = %d\n \tdistance = %d\n",
                drawMovementCommand.x, drawMovementCommand.y, drawMovementCommand.distance);
            break;
        case STRUCT_IRSENSOR:
            printf("Struct: IR Sensor Array\n\tdata = [%s]\n",
                IRSensorArray.data);
            break;
        default:
            printf("Something went wrong. Please try again.\n"); //we couldn't find the struct to deserialize
            break;
      }
  }


/* *****************************************************************************
 End of File
 */
