// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;


#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FALSE 0
#define TRUE 1
#define FLAG 0x7e
#define A_S 0x03
#define A_R 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B

int fd;

int stateMachineSender(State state, int fd) {

    unsigned char byte;
    
    while(state != STOP_STATE){
        if(read(fd, &byte, 1) > 0){
            switch (state){
            case START:
                if(byte == FLAG) 
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if(byte == A_R) 
                    state = A_RCV;
                else if(byte != FLAG)
                    state = START;
                break;
            
            case A_RCV:
                if(byte == C_UA) 
                    state = C_RCV;
                else if(byte == FLAG)
                    state = FLAG;
                else
                    state = START;
                break;

            case C_RCV:
                if(byte == (C_UA ^ A_R)) 
                    state = BCC_OK;
                else if(byte == FLAG)
                    state = FLAG;
                else
                    state = START;
                break;

            case BCC_OK:
                if(byte == FLAG) 
                    state = STOP_STATE;
                else
                    state = START;
                break;

            default:
                break;
            }
        }
    }
}


int stateMachineReceiver(State state, int fd) {
    
    unsigned char byte;
    
    while(state != STOP_STATE){
        if(read(fd, &byte, 1) > 0){
            switch (state){
            case START:
                if(byte == FLAG) 
                    state = FLAG_RCV;
                break;

            case FLAG_RCV:
                if(byte == A_S) 
                    state = A_RCV;
                else if(byte != FLAG)
                    state = START;
                break;
            
            case A_RCV:
                if(byte == C_SET) 
                    state = C_RCV;
                else if(byte == FLAG)
                    state = FLAG;
                else
                    state = START;
                break;

            case C_RCV:
                if(byte == (C_SET ^ A_S)) 
                    state = BCC_OK;
                else if(byte == FLAG)
                    state = FLAG;
                else
                    state = START;
                break;

            case BCC_OK:
                if(byte == FLAG) 
                    state = STOP_STATE;
                else
                    state = START;
                break;

            default:
                break;
            }
        }
    }
}



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {

    State state = START;

    fd = open(serialPortName, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    if (tcgetattr(fd, &oldtio) == -1) {
        perror("tcgetattr");
        exit(-1);
    }

    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 30;
    newtio.c_cc[VMIN] = 5;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    switch (connectionParameters.role) {
    case LlTx:
        while(connectionParameters.nRetransmissions > 0){
            unsigned char frame[5] = {FLAG, A_S, C_SET, (A_S ^ C_SET), FLAG};
            int bytes = write(fd, frame, 5);
            stateMachineSender(&state, fd);
            connectionParameters.nRetransmissions--;
        }
        
        if (state != STOP_STATE) return -1;
        break;

    case LlRx:
        stateMachineReceiver(&state, fd);
        unsigned char frame[5] = {FLAG, A_R, C_UA, (A_R ^ C_UA), FLAG};
        int bytes = write(fd, frame, 5);
        break;
    
    default:
        return -1;
        break;
    }

    return fd;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
