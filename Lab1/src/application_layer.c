#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "application_layer.h"
#include "link_layer.h"

#define DATA_PACKET_SIZE 256
#define START_CONTROL 1
#define END_CONTROL 3
#define DATA_CONTROL 2

void sendFile(const char *filename);
void receiveFile(const char *filename);

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
        sendFile(filename);
    } else {
        receiveFile(filename);
    }

    // Close the link layer
    if (llclose(FALSE) == -1) {
        printf("llclose error\n");
        exit(1);
    }
}

void sendFile(const char *filename) {
    // Open the file
    FILE *file = fopen(filename, "rb");
    if (!file) {
        perror("File opening error");
        exit(1);
    }

    // Get file size
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);

    // Send START packet
    unsigned char startPacket[DATA_PACKET_SIZE];
    startPacket[0] = START_CONTROL;
    sprintf((char *)&startPacket[1], "%ld", fileSize); // Add file size to packet
    if (llwrite(startPacket, strlen((char *)startPacket)) == -1) {
        perror("Error sending START packet");
        fclose(file);
        exit(1);
    }

    // Send data packets
    unsigned char dataPacket[DATA_PACKET_SIZE];
    size_t bytesRead;
    while ((bytesRead = fread(dataPacket + 1, 1, DATA_PACKET_SIZE - 1, file)) > 0) {
        dataPacket[0] = DATA_CONTROL; // Data packet control field
        if (llwrite(dataPacket, bytesRead + 1) == -1) {
            perror("Error sending data packet");
            fclose(file);
            exit(1);
        }
    }

    // Send END packet
    unsigned char endPacket[DATA_PACKET_SIZE];
    endPacket[0] = END_CONTROL;
    sprintf((char *)&endPacket[1], "%ld", fileSize);
    if (llwrite(endPacket, strlen((char *)endPacket)) == -1) {
        perror("Error sending END packet");
        fclose(file);
        exit(1);
    }

    fclose(file);
    printf("File transfer complete.\n");
}

void receiveFile(const char *filename) {
    FILE *file = fopen(filename, "wb");
    if (!file) {
        perror("File creation error");
        exit(1);
    }

    unsigned char packet[DATA_PACKET_SIZE];
    int packetType;

    // Wait for START packet
    packetType = llread(packet);
    /*
    if (packetType != START_CONTROL) {
        perror("Expected START packet, received something else");
        fclose(file);
        exit(1);
    }
    */
    printf("File transfer started.\n");

    // Receive data packets
    while ((packetType = llread(packet)) != END_CONTROL) {
        if (packetType == DATA_CONTROL) {
            fwrite(&packet[1], 1, strlen((char *)&packet[1]), file);
        } else {
            perror("Unexpected packet type");
            fclose(file);
            exit(1);
        }
    }

    fclose(file);
    printf("File received successfully.\n");
}
