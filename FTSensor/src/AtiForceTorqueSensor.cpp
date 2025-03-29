#include <sstream>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
//#include "sockLib.h"
//#include "oslayer.h"
#include "fstream"
#include "AtiForceTorqueSensor.h"

using namespace std;

int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
struct sockaddr_in addr;	        /* Address of Net F/T. */
struct hostent *he;			/* Host entry for Net F/T. */
std::byte request[8];			/* The request data sent to the Net F/T. */
RESPONSE resp;				/* The structured response received from the Net F/T. */
std::byte response[36];			/* The raw response data received from the Net F/T. */
int i;					/* Generic loop/array index. */
int err;				/* Error status of operations. */
char * AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };	//The names of the force and torque ax
int argc;



AtiForceTorqueSensor::AtiForceTorqueSensor(char* add){



    /* Calculate number of samples, command code, and open socket here. */
    socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketHandle == -1) {
        exit(1);
    }

    *(uint16*)&request[0] = htons(0x1234); /* standard header. */
    *(uint16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
    *(uint32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */

    /* Sending the request. */
    he = gethostbyname(add);

    memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);

    err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
    if (err == -1) {
        exit(2);
    }



}

AtiForceTorqueSensor::~AtiForceTorqueSensor(){

}

double* AtiForceTorqueSensor::Acquire()
{

    send( socketHandle, request, 8, 0 );

        /* Receiving the response. */
        recv( socketHandle, response, 36, 0 );
        resp.rdt_sequence = ntohl(*(uint32*)&response[0]);
        resp.ft_sequence = ntohl(*(uint32*)&response[4]);
        resp.status = ntohl(*(uint32*)&response[8]);
        for( i = 0; i < 6; i++ ) {
            int32 raw = ntohl(*(int32*)&response[12 + i * 4]);

            resp.FTData[i] = ((double)raw)/ 1000000.0;
        }

        /* Output the response data. */
//        printf( "Status: 0x%08x\n", resp.status );
//        for (i =0;i < 6;i++) {
//           printf("%s: %f\n", AXES[i], resp.FTData[i]);


//        }
        //usleep(100);
    return resp.FTData;
}

