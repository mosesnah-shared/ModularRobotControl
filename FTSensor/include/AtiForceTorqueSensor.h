/*
 * AtiForceTorqueSensor.h
 *
 *  Created on: 17.12.2017
 *      Author: Lachner
 */

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "fstream"
#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */

//#include "Logging.h"

/* Typedefs used so integer sizes are more explicit */
typedef unsigned int uint32;
typedef int int32;
typedef unsigned short uint16;
typedef short int16;
typedef unsigned char byte;

typedef struct response_struct {

    uint32 rdt_sequence;
    uint32 ft_sequence;
    uint32 status;
    double FTData[6];

} RESPONSE;

class AtiForceTorqueSensor 
{
    public:

    AtiForceTorqueSensor(char*);
    ~AtiForceTorqueSensor();



   void startSensor();

   double* Acquire();

private:

    std::ofstream _myFile;


};



