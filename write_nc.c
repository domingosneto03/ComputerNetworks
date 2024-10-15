// Write to serial port in non-canonical mode
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
#include <signal.h>

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
#define MAX_RETRANSMISSIONS 3

volatile int STOP = FALSE;
volatile int alarmEnabled = FALSE;
volatile int alarmCount = 0;

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("Alarm #%d\n", alarmCount);
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

    // Open serial port device for reading and writing, and not as controlling tty
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
    newtio.c_cc[VTIME] = 30; // Inter-character timer unused
    newtio.c_cc[VMIN] = 0;  // Blocking read until X chars received

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

    (void)signal(SIGALRM, alarmHandler);

    // Create string to send
    unsigned char buf_SET[BUF_SIZE] = {0};

    //Message to send
    buf_SET[0] = FLAG;
    buf_SET[1] = A_S;
    buf_SET[2] = C_S;
    buf_SET[3] = A_S^C_S;
    buf_SET[4] = FLAG; 

    // In non-canonical mode, '\n' does not end the writing.
    // Test this condition by placing a '\n' in the middle of the buffer.
    // The whole buffer must be sent even with the '\n'.
    buf_SET[5] = '\n';

    unsigned char buf_UA[BUF_SIZE + 1] = {0};

    while (alarmCount < MAX_RETRANSMISSIONS && STOP == FALSE) {

        int bytes = write(fd, buf_SET, 5);
        printf("SET frame sent.\n");

        alarmEnabled = TRUE;
        alarm(3);

        int bytes_UA = read(fd, buf_UA, 5);
        buf_UA[bytes_UA] = '\0';

        // Check if UA frame was correctly received
        if (buf_UA[0] == FLAG && buf_UA[1] == A_R && buf_UA[2] == C_R && buf_UA[3] == A_R ^ C_R && buf_UA[4] == FLAG) {
            printf("UA frame received successfully.\n");
            alarm(0);
            STOP = TRUE;
        } else {
            printf("UA frame not received correctly.\n");
            if (alarmCount >= MAX_RETRANSMISSIONS) {
                printf("Maximum number of retransmissions reached. Aborting.\n");
                STOP = TRUE;
            }
        }
    }

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}