// Read from serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1

#define FLAG 0x7e

#define A_S 0x03
#define A_R 0x01

#define C_S 0x03
#define C_R 0x07

#define BUF_SIZE 256  

volatile int STOP = FALSE;

typedef enum {
    START,
    FLAG_RCV,
    A_RCV,
    C_RCV,
    BCC_OK,
    STOP_STATE
} State;

State currentState = START;

// Function to handle state transitions based on the received byte
void stateMachine(unsigned char byte) {
    switch (currentState) {
        case START:
            if (byte == FLAG) {
                currentState = FLAG_RCV;
                printf("State changed to FLAG_RCV\n");
            }
            break;

        case FLAG_RCV:
            if (byte == FLAG) {
                printf("FLAG received again, staying in FLAG_RCV\n");
            } else if (byte == A_S) {
                currentState = A_RCV;
                printf("State changed to A_RCV\n");
            } else {
                currentState = START;
                printf("Unexpected byte, returning to START\n");
            }
            break;

        case A_RCV:
            if (byte == FLAG) {
                currentState = FLAG_RCV;
                printf("FLAG received, back to FLAG_RCV\n");
            } else if (byte == C_S) {
                currentState = C_RCV;
                printf("State changed to C_RCV\n");
            } else {
                currentState = START;
                printf("Unexpected byte, returning to START\n");
            }
            break;

        case C_RCV:
            if (byte == FLAG) {
                currentState = FLAG_RCV;
                printf("FLAG received, back to FLAG_RCV\n");
            } else if (byte == A_S^C_S) {
                currentState = BCC_OK;
                printf("State changed to BCC_OK\n");
            } else {
                currentState = START;
                printf("Unexpected byte, returning to START\n");
            }
            break;

        case BCC_OK:
            if (byte == FLAG) {
                currentState = STOP_STATE;
                printf("State changed to STOP, message received successfully\n");
            } else {
                currentState = START;
                printf("Unexpected byte, returning to START\n");
            }
            break;

        case STOP_STATE:
            break;

        default:
            currentState = START;
            break;
    }
}

int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Loop for input
    unsigned char buf_SET[BUF_SIZE + 1] = {0}; // +1: Save space for the final '\0' char

    while (STOP == FALSE)
    {
        int bytes = read(fd, buf_SET, 5);
        if (bytes > 0) {
            for(int i=0; i<bytes; i++)
                stateMachine(buf_SET[i]);
        }

        // Check received message
        if (currentState==STOP_STATE)
            printf("Message from Sender received with success. (%d bytes read)\n", bytes);
        else {
            printf("Message from Sender not received correctly.\n");
            STOP = TRUE;
        }
		
		for (int i=0; i < bytes; i++)
			printf("0x%02X\n", buf_SET[i]);
			
        // Create string to send
        unsigned char buf_UA[BUF_SIZE] = {0};

        //Mensagem to send
        buf_UA[0] = FLAG;
        buf_UA[1] = A_R;
        buf_UA[2] = C_R;
        buf_UA[3] = A_R ^ C_R;
        buf_UA[4] = FLAG; 

        int bytes_UA = write(fd, buf_UA, 5);
        printf("Message sent. (%d bytes written)\n", bytes_UA);
        if (buf_SET[bytes] == '\0')
            STOP = TRUE;
    }

    // The while() cycle should be changed in order to respect the specifications
    // of the protocol indicated in the Lab guide

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
