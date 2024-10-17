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
#include <unistd.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source
#define FALSE 0
#define TRUE 1
#define FLAG 0x7e
#define A_TX 0x03
#define A_RX 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define BUF_SIZE 256

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {

    State state = START;

    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0) {
        perror(connectionParameters.serialPort);
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
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    unsigned char buf_R[BUF_SIZE + 1] = {0};

    switch (connectionParameters.role) {

        // reads message from Receiver
        case LlTx:
            unsigned char buf_W1[5] = {FLAG, A_TX, C_SET, A_TX ^ C_SET, FLAG};
            write(fd, buf_W1, 5);
            
            while (state != STOP_STATE) {
                int bytes_R = read(fd, buf_R, 1);
                if(bytes_R > 0) {
                    printf("0x%02X\n", buf_R[0]);
                    switch (state) {
                        case START:
                            if (buf_R[0] == FLAG) state = FLAG_RCV;
                            break;

                        case FLAG_RCV:
                            if (buf_R[0] == FLAG) continue;
                            else if (buf_R[0] == A_RX) state = A_RCV; 
                            else state = START;
                            break;

                        case A_RCV:
                            if (buf_R[0] == FLAG) state = FLAG_RCV;
                            else if (buf_R[0] == C_UA) state = C_RCV;
                            else state = START;
                            break;

                        case C_RCV:
                            if (buf_R[0] == FLAG) state = FLAG_RCV;
                            else if (buf_R[0] == (A_RX^C_UA)) state = BCC_OK;
                            else state = START;
                            break;

                        case BCC_OK:
                            if (buf_R[0] == FLAG) state = STOP_STATE;
                            else state = START;
                            break;

                        default:
							break;
                    }
                }
            }
            
            if(state != STOP_STATE) return -1;
            break;

        // reads message from Sender
        case LlRx:
            while(state != STOP_STATE){

                int bytes_R = read(fd, buf_R, 1);
                if(bytes_R > 0) {
                    printf("0x%02X\n", buf_R[0]);
                    switch (state) {
                        case START:
                            if (buf_R[0] == FLAG) state = FLAG_RCV;
                            break;

                        case FLAG_RCV:
                            if (buf_R[0] == FLAG) continue;
                            else if (buf_R[0] == A_TX) state = A_RCV; 
                            else state = START;
                            break;

                        case A_RCV:
                            if (buf_R[0] == FLAG) state = FLAG_RCV;
                            else if (buf_R[0] == C_SET) state = C_RCV;
                            else state = START;
                            break;

                        case C_RCV:
                            if (buf_R[0] == FLAG) state = FLAG_RCV;
                            else if (buf_R[0] == (A_TX^C_SET)) state = BCC_OK;
                            else state = START;
                            break;

                        case BCC_OK:
                            if (buf_R[0] == FLAG) state = STOP_STATE;
                            else state = START;
                            break;

                        default:
							break;
                    }
                }
            }
            
            unsigned char buf_W2[5] = {FLAG, A_RX, C_UA, A_RX ^ C_UA, FLAG};
			write(fd, buf_W2, 5);
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
