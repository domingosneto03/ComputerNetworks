// Link layer protocol implementation
//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
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
#include <signal.h>

#define _POSIX_SOURCE 1 // POSIX compliant source
#define FALSE 0
#define TRUE 1
#define FLAG 0x7e
#define A 0x03
#define A_ 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_RR0 0xAA
#define C_RR1 0xAB
#define C_REJ0 0x54
#define C_REJ1 0x55
#define C_DISC 0x0B
#define C_N0 0x00
#define C_N1 0x80
#define BUF_SIZE 256
#define ESCAPE 0x7d
#define HT_SIZE 6
#define TX 1
#define RX 0 

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC1_OK,
    BCC2_OK,
    STOP_STATE
} State;


volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;
int timeout = 0;
int retransmissions = 0;
int info = 0;
int role;

int i_frames = 0;
int u_frames = 0;
int s_frames = 0;
int dup_frames = 0;
int rej_frames = 0;


void alarmHandler(int signal) {
    alarmEnabled = TRUE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
}

State StateMachine(State *state, int func, LinkLayerRole role) {
    if ((func != 0 && role != -1) || (func == 0 && role == -1)) {
        return -1;
    } else {
        unsigned char control_field;
        unsigned char buf_R[BUF_SIZE + 1] = {0};
        while(*state != STOP_STATE) {
            int bytes_R = readByteSerialPort(buf_R);
            if(bytes_R > 0) {
                printf("0x%02X\n", buf_R[0]);
                switch (*state) {
                    case START:
                        if (buf_R[0] == FLAG) {
                            *state = FLAG_RCV;
                        } else {
                            printf("Unexpected byte, staying in START\n");
                        } 
                        break;


                    case FLAG_RCV:
                        if (buf_R[0] == FLAG)  {
                            printf("FLAG received again, staying in FLAG_RCV\n");
                            continue;
                        } else if (buf_R[0] == A) {
                            *state = A_RCV; 
                        } else {
                            *state = START;
                            printf("Unexpected byte, returning to START\n");
                        }
                        break;

                    case A_RCV:
                        if (buf_R[0] == FLAG) {
                            *state = FLAG_RCV;
                            printf("FLAG received, returning to FLAG_RCV\n");
                        } else if (func == 0 && role == LlTx && buf_R[0] == C_UA) {
                            control_field = buf_R[0];
                            *state = C_RCV;
                        } else if (func == 0 && role == LlRx && buf_R[0] == C_SET) {
                            control_field = buf_R[0];
                            *state = C_RCV;
                        } else if (func == 1 && role == -1 && (buf_R[0] == C_RR0 || buf_R[0] == C_RR1 || buf_R[0] == C_REJ0 || buf_R[0] == C_REJ1 || buf_R[0] == C_DISC)) {
                            control_field = buf_R[0];
                            *state = C_RCV;
                        } else if (func == 3 && role == -1 && buf_R[0] == C_DISC) {
                            control_field = buf_R[0];
                            *state = C_RCV;
                        } else {
                            *state = START;
                            printf("Unexpected byte, returning to START\n");
                        }
                        break;

                    case C_RCV:
                        if (buf_R[0] == FLAG) {
                            *state = FLAG_RCV;
                            printf("FLAG received, returning to FLAG_RCV\n");
                        } else if (buf_R[0] == (control_field ^ A)) {
                            *state = BCC1_OK;
                        } else {
                            *state = START;
                            printf("Unexpected byte, returning to START\n");
                        }
                        break;

                    case BCC1_OK:
                        if (buf_R[0] == FLAG) {
                            *state = STOP_STATE;
                        } else {
                            *state = START;
                            printf("Unexpected byte, returning to START\n");
                        }
                        break;

                    default:
                        break;
                }
            }
        }
        return control_field;
    }  
}


////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
        return -1;
    }

    State state = START;
    timeout = connectionParameters.timeout;
    retransmissions = connectionParameters.nRetransmissions;

    switch (connectionParameters.role) {
        case LlTx:
            role = TX;
            (void)signal(SIGALRM, alarmHandler);
            while (connectionParameters.nRetransmissions > 0 && state != STOP_STATE) {
                unsigned char buf_W1[5] = {FLAG, A, C_SET, A ^ C_SET, FLAG};
                writeBytesSerialPort(buf_W1, 5);
                printf("SET frame sent.\n");
                u_frames++;

                alarm(connectionParameters.timeout);
                alarmEnabled = FALSE;

                printf("---------------------------------------------\n");
                StateMachine(&state, 0, LlTx);
                connectionParameters.nRetransmissions--;
            }
            if(state != STOP_STATE) {
                return -1;
            }
            s_frames++;
            break;
        
        case LlRx:
            role = RX;
            printf("---------------------------------------------\n");
            StateMachine(&state, 0, LlRx);
            unsigned char buf_W2[5] = {FLAG, A, C_UA, A ^ C_UA, FLAG};
			writeBytesSerialPort(buf_W2, 5);
            printf("UA frame sent.\n");
            u_frames++;
            s_frames++;
            break;
        
        default:
            return -1;
            break;
    }
    return 1;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char* buf_W = (unsigned char*) malloc(HT_SIZE + 2 * bufSize);
	//Header
    buf_W[0] = FLAG;
    buf_W[1] = A;
    
    if (info == 0) {
        buf_W[2] = C_N0;
    } else { 
        buf_W[2] = C_N1;
    }
    buf_W[3] = A ^ buf_W[2];
    
    info = !info;
	int data = 4;
	
    unsigned char BCC2 = buf[0];
	for(int i = 1; i < bufSize; i++) {
		BCC2 ^= buf[i];
	}

	for(int i = 0; i < bufSize; i++) {
		if(buf[i] == FLAG || buf[i] == ESCAPE) {
			buf_W[data++] = ESCAPE;
			buf_W[data++] = (buf[i]^0x20);
		} else {
			buf_W[data++] = buf[i];
		}
	}
    
	if(BCC2 == FLAG || BCC2 == ESCAPE) {
        buf_W[data++] = ESCAPE;
        buf_W[data++] = (BCC2^0x20);
    } else {
        buf_W[data++] = BCC2;
    }

	buf_W[data++] = FLAG;

	buf_W = realloc(buf_W, data);

	int check = FALSE;
    unsigned char control_field = 0;
	int bytes_W = 0;

	(void) signal(SIGALRM, alarmHandler);
	
	while(retransmissions > 0){
        State state = START;
		bytes_W = writeBytesSerialPort(buf_W, data);
        i_frames++;
    
		//Wait until all bytes have been wrtien
		sleep(1);

		alarm(timeout);
		alarmEnabled = FALSE;	

        printf("---------------------------------------------\n");
        control_field = StateMachine(&state, 1, -1);
		if(control_field == C_RR0 || control_field == C_RR1) {
			check = TRUE;
            s_frames++;
			break;
		} else if(control_field == C_REJ0 || control_field == C_REJ1) {
            s_frames++;
			continue;
		}
        retransmissions--;
	}
	
	free(buf_W);
	
	if(check) {
		return bytes_W;
	} else {
		llclose(0);
		return -1;
	}
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    State state = START;
    unsigned char buf_R[BUF_SIZE + 1] = {0};
    unsigned char control_field;
    unsigned char last_control_field = 0x7e;
    int i = 0;
    printf("---------------------------------------------\n");
    while (state != STOP_STATE && alarmEnabled == FALSE) {
        int bytes_R = readByteSerialPort(buf_R);
        if(bytes_R > 0) {
            //printf("0x%02X\n", buf_R[0]);
            switch (state) {
                case START:
                    if (buf_R[0] == FLAG) {
                        state = FLAG_RCV;
                    } else {
                        printf("Unexpected byte, staying in START\n");
                    } 
                    break;

                case FLAG_RCV:
                    if (buf_R[0] == FLAG)  {
                        printf("FLAG received again, staying in FLAG_RCV\n");
                        continue;
                    } else if (buf_R[0] == A) {
                        state = A_RCV; 
                    } else {
                        state = START;
                        printf("Unexpected byte, returning to START\n");
                    }
                    break;

                case A_RCV:
                    if (buf_R[0] == FLAG) {
                        state = FLAG_RCV;
                        printf("FLAG received, returning to FLAG_RCV\n");
                    } else if (buf_R[0] == C_N0 || buf_R[0] == C_N1 || buf_R[0] == C_DISC) {
                        state = C_RCV;
                        control_field = buf_R[0];
                        i_frames++;
                        if(control_field == last_control_field) {
                            dup_frames++;
                            printf("Duplicated frame detected\n");
                        }
                    } else {
                        state = START;
                        printf("Unexpected byte, returning to START\n");
                    }
                    break;

                case C_RCV:
                    if (buf_R[0] == FLAG) {
                        state = FLAG_RCV;
                        printf("FLAG received, returning to FLAG_RCV\n");
                    } else if (buf_R[0] == (A ^ control_field)) {
                        if (control_field ==  C_DISC) {
                            u_frames++;
                            unsigned char buf_W[5] = {FLAG, A_, C_DISC, A_ ^ C_DISC, FLAG};
                            writeBytesSerialPort(buf_W, 5);
                            return 0;
                        }
                        state = BCC1_OK;
                    } else {
                        state = START;
                        printf("Unexpected byte, returning to START\n");
                    }
                    break;

                case BCC1_OK:
                    if (buf_R[0] == ESCAPE) {
                        //printf("ESC received, reading next byte\n");
                        readByteSerialPort(buf_R);
                        packet[i++] = buf_R[0] ^ 0x20;
                        //printf("After ESC: 0x%02X\n", packet[i - 1]);

                    } else if (buf_R[0] == FLAG){
                        printf("Packet received\n");
                        printf("Checking BCC2 and processing packet\n");
                        unsigned char bcc2 = packet[--i];
                        packet[i] = '\0';
                        unsigned char acc = 0;
                        for (unsigned int j = 0; j < i; j++) {
                            acc ^= packet[j];
                        }
                        if (bcc2 == acc){
                            printf("BCC2 check passed, sending RR\n");
                            state = STOP_STATE;
                            unsigned char rr;
                            if (control_field == C_N0) {
                                rr = C_RR0;
                                printf("Control Field is C_N0, sending RR0\n");
                            } else {
                                rr = C_RR1;
                                printf("Control Field is C_N1, sending RR1\n");
                            }
                            s_frames++;
                            unsigned char buf_W[5] = {FLAG, A, rr, A ^ rr, FLAG};
                            writeBytesSerialPort(buf_W, 5);
                            return packet[0];
                        } else {
                            printf("BCC2 check failed, sending REJ\n");
                            unsigned char rej;
                            if (control_field == C_N0) {
                                rej = C_REJ0;
                                printf("Control Field is C_N0, sending REJ0\n");
                            } else {
                                rej = C_REJ1;
                                printf("Control Field is C_N1, sending REJ1\n");
                            }
                            rej_frames++;
                            s_frames++;
                            unsigned char buf_W[5] = {FLAG, A, rej, A ^ rej, FLAG};
                            writeBytesSerialPort(buf_W, 5);
                            return -1;
                        }
                    } else {
                        packet[i++] = buf_R[0];
                    }
                    break;    
                default:
                    break;
            }
        }
    }
    return -1;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    State state = START;

	(void) signal(SIGALRM, alarmHandler);

	while(retransmissions != 0 && state != STOP_STATE) {
		unsigned char buf_W[5] = {FLAG, A, C_DISC, A ^ C_DISC, FLAG};
		writeBytesSerialPort(buf_W, 5);

		alarm(timeout);
		alarmEnabled = FALSE;

        u_frames++;
        StateMachine(&state, 3, -1);
		retransmissions--;

	}

	if(state != STOP_STATE) {
		return -1;
	}

	unsigned char buf_W[5] = {FLAG, A_, C_UA, A_ ^ C_UA, FLAG};
	writeBytesSerialPort(buf_W, 5);
    u_frames++;

    printf("---------------------------------------------\n");

    switch(role) {
        case TX:
            printf("I frames sent: %d\n", i_frames);
            printf("U frames sent: %d\n", u_frames);
            printf("S frames received: %d\n", s_frames);
            break;

        case RX:
            printf("I frames received: %d\n", i_frames);
            printf("U frames received: %d\n", u_frames);
            printf("S frames sent: %d\n", s_frames);
            printf("Duplicated frames received: %d\n", dup_frames);
            printf("Rejected frames: %d\n", rej_frames);
            break;
    }

    printf("---------------------------------------------\n");

    int clstat = closeSerialPort();
    return clstat;
}