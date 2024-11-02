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

    // Depending on the role, functions to send and receive the file
    if (ll.role == LlTx) {
        sendFile(filename);
    } else {
        receiveFile(filename);
    }

    // Close the link layer
    if (llclose(1) == -1) {
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
    memcpy(&startPacket[1], &fileSize, sizeof(long));
    if (llwrite(startPacket, strlen((char *)startPacket)) == -1) {
        perror("Error sending START packet");
        fclose(file);
        exit(1);
    }

    // Send data packets
    unsigned char dataPacket[DATA_PACKET_SIZE];
    size_t bytesRead;
    while ((bytesRead = fread(&dataPacket[2], 1, DATA_PACKET_SIZE - 2, file)) > 0) {
        dataPacket[0] = DATA_CONTROL;
        dataPacket[1] = bytesRead;
        if (llwrite(dataPacket, bytesRead + 2) == -1) {
            perror("Error sending data packet");
            fclose(file);
            exit(1);
        }
    }


    // Send END packet
    unsigned char endPacket[DATA_PACKET_SIZE];
    endPacket[0] = END_CONTROL;
    memcpy(&endPacket[1], &fileSize, sizeof(long));
    if (llwrite(endPacket, sizeof(long) + 1) == -1) {
        perror("Error sending END packet");
        fclose(file);
        exit(1);
    }


    fclose(file);
    printf("---------------------------------------------\n");
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
    
    if (packetType == START_CONTROL) {
        long receivedFileSize;
        memcpy(&receivedFileSize, &packet[1], sizeof(long));
    } else {
        perror("Expected START packet, received something else");
        fclose(file);
        exit(1);
    }
    
    printf("File transfer started.\n");

    // Receive data packets until END packet
    while ((packetType = llread(packet)) != END_CONTROL) {
        if (packetType == DATA_CONTROL) {
            int dataSize = packet[1];
            fwrite(&packet[2], 1, dataSize, file);
        } else {    
            perror("Unexpected packet type");
            fclose(file);
            exit(1);
        }
    }

    // Wait for END packet
    if (packetType == END_CONTROL) {
        long endFileSize;
        memcpy(&endFileSize, &packet[1], sizeof(long));

    } else {
        perror("Expected END packet, received something else");
        fclose(file);
        exit(1);
    }

    fclose(file);
    printf("---------------------------------------------\n");
    printf("File received successfully.\n");
}
