#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "application_layer.h"
#include "link_layer.h"

#define DATA_PACKET 0x01
#define START_PACKET 0x02
#define END_PACKET 0x03

#define CONTROL_START 2
#define CONTROL_END 3
#define MAX_PACKET_SIZE 1024

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer ll;
    sprintf(ll.serialPort, "%s", serialPort);

    ll.role = strcmp(role, "tx") ? LlRx : LlTx;
    ll.baudRate = baudRate;
    ll.nRetransmissions = nTries;
    ll.timeout = timeout;

    // Open the link layer
    if (llopen(ll) == -1) {
        printf("llopen error\n");
        exit(1);
    }

    if (ll.role == LlTx) {
        // Transmitter role: send data
        sendFile(filename);
    } else {
        // Receiver role: receive data
        receiveFile(filename);
    }

    // Close the link layer
    if (llclose(1) == -1) {
        printf("llclose error\n");
        exit(1);
    }
}

// Function to create a control packet (START or END)
int createControlPacket(int control, const char *filename, unsigned char *packet) {
    int index = 0;
    packet[index++] = control;

    // Filename TLV
    packet[index++] = 0x01; // Type for filename
    int filenameLength = strlen(filename);
    packet[index++] = filenameLength;
    memcpy(&packet[index], filename, filenameLength);
    index += filenameLength;

    return index;
}

// Function to send a file
void sendFile(const char *filename) {
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("Failed to open file");
        exit(1);
    }

    unsigned char packet[MAX_PACKET_SIZE];
    int packetSize = createControlPacket(CONTROL_START, filename, packet);

    if (llwrite(packet, packetSize) == -1) {
        printf("Failed to send START packet\n");
        fclose(file);
        exit(1);
    }

    unsigned char buffer[1024];
    int bytesRead;
    while ((bytesRead = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        packetSize = createDataPacket(buffer, bytesRead, packet);
        if (llwrite(packet, packetSize) == -1) {
            printf("Error sending data packet\n");
            fclose(file);
            exit(1);
        }
    }

    packetSize = createControlPacket(CONTROL_END, filename, packet);
    if (llwrite(packet, packetSize) == -1) {
        printf("Failed to send END packet\n");
        fclose(file);
        exit(1);
    }

    fclose(file);
}

// Function to receive a file
void receiveFile(const char *filename) {
    unsigned char packet[MAX_PACKET_SIZE];
    int packetSize = llread(packet);

    if (packetSize < 0 || packet[0] != CONTROL_START) {
        printf("Failed to receive START packet\n");
        exit(1);
    }

    FILE *file = fopen(filename, "wb");
    if (!file) {
        perror("Failed to open file");
        exit(1);
    }

    while (1) {
        packetSize = llread(packet);
        if (packetSize < 0) {
            printf("Failed to receive packet\n");
            fclose(file);
            exit(1);
        }
        if (packet[0] == CONTROL_END) break;

        if (fwrite(&packet[1], 1, packetSize - 1, file) < (size_t)(packetSize - 1)) {
            perror("Error writing to file");
            fclose(file);
            exit(1);
        }
    }

    fclose(file);
}

// Function to create a data packet
int createDataPacket(const unsigned char *data, int dataSize, unsigned char *packet) {
    int index = 0;
    packet[index++] = DATA_PACKET;
    packet[index++] = (dataSize >> 8) & 0xFF;
    packet[index++] = dataSize & 0xFF;
    memcpy(&packet[index], data, dataSize);
    index += dataSize;
    return index;
}
