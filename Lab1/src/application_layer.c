// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define DATA_PACKET 1
#define START_PACKET 2
#define END_PACKET 3
#define SIZE_FIELD 0
#define NAME_FIELD 1

int sendControlPacket(int packetType, const char* fileName, long fileSize) {
    size_t fileNameLength = strlen(fileName) + 1;
    size_t fileSizeBytes = (fileSize > 0) ? ((sizeof(long) * 8 - __builtin_clzl(fileSize) + 7) / 8) : 1;
    size_t packetSize = 5  + fileNameLength + fileSizeBytes;

    unsigned char* controlPacket = (unsigned char*)malloc(packetSize);

    int i = 0;
    controlPacket[i++] = packetType;
    controlPacket[i++] = SIZE_FIELD;

    for (size_t j = 0; j < sizeof(size_t); j++) {
        controlPacket[i + j] = (fileSizeBytes >> (j * 8)) & 0xFF;
    }
    i += sizeof(size_t);

    controlPacket[i++] = NAME_FIELD;
    memcpy(controlPacket + i, fileName, fileNameLength);

    if (llwrite(controlPacket, packetSize) < 0) {
        printf("Failed to send control packet\n");
        free(controlPacket);
        return -1;
    }

    free(controlPacket);
    return 1;
}

int readControlPacket(unsigned char expectedPacketType, unsigned char* buffer, size_t* fileSize, char* fileName) {
    int bufferSize;
    if ((bufferSize = llread(buffer)) < 0) {
        printf("Error reading control packet\n");
        return -1;
    }

    if (buffer[0] != expectedPacketType) {
        printf("Invalid control packet\n");
        return -1;
    }

    int i = 1;
    unsigned char fieldType;
    while (i < bufferSize) {
        fieldType = buffer[i++];
        if (fieldType == NAME_FIELD) {
            size_t nameLength = buffer[i++];
            memcpy(fileName, &buffer[i], nameLength);
            fileName[nameLength] = '\0';
            i += nameLength;
        }
        else if (fieldType == SIZE_FIELD) {
            *fileSize = 0;
            for (size_t j = 0; j < sizeof(size_t); j++) {
                *fileSize |= ((size_t)buffer[i++] << (j * 8));
            }
        } else {
            printf("Invalid control packet type\n");
            return -1;
        }
    }
    return 1;
}

int sendDataPacket(unsigned char* data, int dataSize) {
    size_t packetSize = dataSize + 3;
    unsigned char* packet = (unsigned char*)malloc(packetSize);
    if (packet == NULL) {
        printf("Memory allocation failed\n");
        return -1;
    }

    packet[0] = DATA_PACKET;
    packet[1] = (dataSize >> 8) & 0xFF;
    packet[2] = dataSize & 0xFF;
    memcpy(packet + 3, data, dataSize);

    int status = llwrite(packet, packetSize);
    free(packet);

    if (status < 0) {
        printf("Error sending data packet\n");
        return -1;
    }

    return 0;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int maxRetries, int timeout, const char *filename) {
    LinkLayer ll;
    sprintf(ll.serialPort, "%s", serialPort);
    ll.role = strcmp(role, "tx") ? LlRx : LlTx;
    ll.baudRate = baudRate;
    ll.nRetransmissions = maxRetries;
    ll.timeout = timeout;

    if (llopen(ll) == -1) {
        perror("Error opening link layer\n");
        exit(1);
    }

    switch (ll.role) {
        case LlRx: {
            size_t fileSize;
            char receivedFileName[0xFF];
            unsigned char* buffer = (unsigned char*)malloc(MAX_PAYLOAD_SIZE);

            if (buffer == NULL) {
                printf("Memory allocation failed\n");
                exit(-1);
            }

            if (readControlPacket(START_PACKET, buffer, &fileSize, receivedFileName) < 0) {
                printf("Error reading control packet\n");
                free(buffer);
                exit(-1);
            }

            printf("Start packet received\n");

            FILE* fileOutput = fopen((char*)filename, "wb+");
            if (fileOutput == NULL) {
                printf("Error opening file\n");
                free(buffer);
                exit(-1);
            }

            printf("File opened for writing\n");
            int dataSize;

            while ((dataSize = llread(buffer)) >= 0) {
                if (dataSize == 0) continue;

                if (buffer[0] == END_PACKET) {
                    printf("End packet received\n");
                    break;
                }

                fwrite(buffer + 3, 1, (buffer[1] << 8) | buffer[2], fileOutput);
            }

            free(buffer);
            fclose(fileOutput);
            printf("File closed\n");

            if (llclose(TRUE) < 0) {
                printf("Error closing connection\n");
                exit(-1);
            }

            printf("Connection closed\n");
            break;
        }

        case LlTx: {
            FILE* file = fopen(filename, "rb");
            if (file == NULL) {
                printf("Error opening file for reading\n");
                exit(-1);
            }

            fseek(file, 0, SEEK_END);
            long fileSize = ftell(file);
            fseek(file, 0, SEEK_SET);

            printf("Sending start packet\n");

            if (sendControlPacket(START_PACKET, filename, fileSize) < 0) {
                printf("Error sending start packet\n");
                fclose(file);
                exit(-1);
            }

            unsigned char* buffer = (unsigned char*)malloc(MAX_PAYLOAD_SIZE - 3);
            if (buffer == NULL) {
                printf("Memory allocation failed\n");
                fclose(file);
                exit(-1);
            }

            int dataSize;
            while ((dataSize = fread(buffer, 1, MAX_PAYLOAD_SIZE - 3, file)) > 0) {
                if (sendDataPacket(buffer, dataSize) < 0) {
                    printf("Error sending data packet\n");
                    free(buffer);
                    fclose(file);
                    exit(-1);
                }
            }

            free(buffer);
            fclose(file);

            if (sendControlPacket(END_PACKET, filename, fileSize) < 0) {
                printf("Error sending end packet\n");
                exit(-1);
            }

            printf("End packet sent\n");

            if (llclose(1) < 0) {
                printf("Error closing connection\n");
                exit(-1);
            }

            printf("Connection closed\n");
            break;
        }

        default:
            printf("Invalid role specified\n");
            break;
    }
}
