/*******************************************************************************
 *
 * Copyright (c) 2015 Thomas Telkamp
 *
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the Eclipse Public License v1.0
 * which accompanies this distribution, and is available at
 * http://www.eclipse.org/legal/epl-v10.html
 *
 *******************************************************************************/

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <iostream>
#include <cstdlib>
#include <sys/time.h>
#include <cstring>

#include <sys/ioctl.h>
#include <net/if.h>

using namespace std;

//#include "base64.h"

#include <wiringPi.h>
#include <wiringPiSPI.h>

typedef bool boolean;
typedef unsigned char byte;

static const int CHANNEL = 0;

byte currentMode = 0x81;

char message[256];
//char b64[256];

bool sx1272 = false;

byte receivedbytes;

struct sockaddr_in si_other;
int s, slen=sizeof(si_other);
struct ifreq ifr;


enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };

/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1272 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;

// Set spreading factor (SF7 - SF12)
sf_t sf = SF9;

// Set center frequency
uint32_t  freq = 868100000; // in Mhz! (868.1)

// Set location
float lat=0.0;
float lon=0.0;
int   alt=0;


// #############################################

#define REG_FIFO                    0x00
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_OPMODE                  0x01
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42

#define SX72_MODE_RX_CONTINUOS      0x85
#define SX72_MODE_TX                0x83
#define SX72_MODE_SLEEP             0x80
#define SX72_MODE_STANDBY           0x81

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

// CONF REG
#define REG1                        0x0A
#define REG2                        0x84

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66

#define BUFLEN 2048  //Max length of buffer

#define PROTOCOL_VERSION  1
#define PKT_PUSH_DATA 0
#define PKT_PUSH_ACK  1
#define PKT_PULL_DATA 2
#define PKT_PULL_RESP 3
#define PKT_PULL_ACK  4

#define TX_BUFF_SIZE  2048
#define STATUS_SIZE	  1024

void die(const char *s)
{
    perror(s);
    exit(1);
}

void selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}

byte readRegister(byte addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void writeRegister(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);

    unselectreceiver();
}


boolean receivePkt(char *payload)
{

    // clear rxDone
    writeRegister(REG_IRQ_FLAGS, 0x40);

    int irqflags = readRegister(REG_IRQ_FLAGS);


    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        printf("CRC error\n");
        writeRegister(REG_IRQ_FLAGS, 0x20);
        return false;
    } else {


        byte currentAddr = readRegister(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readRegister(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

        writeRegister(REG_FIFO_ADDR_PTR, currentAddr);

	printf("payload:");
        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readRegister(REG_FIFO);
           printf("%d",payload[i]);
	   printf(" ");		
        }
	printf("\n");
    }
    return true;
}

void SetupLoRa()
{
    
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        printf("SX1276 detected, starting.\n");
            sx1272 = false;

    writeRegister(REG_OPMODE, SX72_MODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeRegister(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeRegister(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeRegister(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeRegister(REG_SYNC_WORD, 0x12); // LoRaWAN public sync word

        writeRegister(REG_MODEM_CONFIG3,0x00);
        writeRegister(REG_MODEM_CONFIG,0x78);
        writeRegister(REG_MODEM_CONFIG2,0x94);

    // Set Continous Receive Mode
    writeRegister(REG_LNA, LNA_MAX_GAIN);  // max lna gain
    writeRegister(REG_OPMODE, SX72_MODE_RX_CONTINUOS);
}

void sendudp(char *msg, int length) {

//to jsem cele smazal

}

void sendstat() {

//to jsem cele smazal

}

void receivepacket() {

    long int SNR;
    int rssicorr;

    if(digitalRead(dio0) == 1)
    {
        if(receivePkt(message)) {
            byte value = readRegister(REG_PKT_SNR_VALUE);
            if( value & 0x80 ) // The SNR sign bit is 1
            {
                // Invert and divide by 4
                value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                SNR = -value;
            }
            else
            {
                // Divide by 4
                SNR = ( value & 0xFF ) >> 2;
            }
            
            if (sx1272) {
                rssicorr = 139;
            } else {
                rssicorr = 157;
            }

            printf("Packet RSSI: %d, ",readRegister(0x1A)-rssicorr);
            printf("RSSI: %d, ",readRegister(0x1B)-rssicorr);
            printf("SNR: %li, ",SNR);
            printf("Length: %i",(int)receivedbytes);
            printf(" ");
//tiskni cas
	time_t  t = time(NULL);
	struct tm *tm = localtime(&t);
	printf("%s\n", asctime(tm));



//tady jsem smazal celou cast tvorby datagramu  pro odeslani pres UDP

            fflush(stdout);

        } // received a message

    } // dio0=1
}

int main () {


    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    wiringPiSPISetup(CHANNEL, 500000);

    SetupLoRa();

    /* display result */
    printf("Gateway ID: %.2x:%.2x:%.2x:ff:ff:%.2x:%.2x:%.2x\n",
           (unsigned char)ifr.ifr_hwaddr.sa_data[0],
           (unsigned char)ifr.ifr_hwaddr.sa_data[1],
           (unsigned char)ifr.ifr_hwaddr.sa_data[2],
           (unsigned char)ifr.ifr_hwaddr.sa_data[3],
           (unsigned char)ifr.ifr_hwaddr.sa_data[4],
           (unsigned char)ifr.ifr_hwaddr.sa_data[5]);

    printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
    printf("------------------\n");

    while(1) {

        receivepacket();

        delay(1);
    }

    return (0);

}

