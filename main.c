#pragma config FOSC=HSPLL
#pragma config WDTEN=OFF
#pragma config XINST=OFF

#include <xc.h>
#include <stdint.h>
#include <math.h>
#include "LCD.h"

//Connections
// Module must be powered with 5V. IO levels are 3.3V
//Module TX -> PIC board RG2 (RX)
//Module RX -> PIC board RG1 (TX)

#define HEADER_SIZE 4
#define EOF_SIZE    2
#define PACKET_SIZE 30

typedef struct {
    int16_t x_pos;
    int16_t y_pos;
    int16_t speed;
    uint16_t resolution;
} target;

union {

    struct {
        uint8_t header[HEADER_SIZE];
        target target1;
        target target2;
        target target3;
        uint8_t eof[EOF_SIZE];
    };
    uint8_t bytes[PACKET_SIZE];
} packet;

uint8_t header_values[] = {0xAA, 0xFF, 0x03, 0x00};
uint8_t eof[] = {0x55, 0xCC};
int8_t rxCount = 0;
uint8_t newPacket = 0;
int packetCount = 0;

void processPacket();
void configUART();

void main(void) {
    OSCTUNEbits.PLLEN = 1;
    TRISDbits.TRISD0 = 0;
    LATDbits.LATD0 = 0;
    LCDInit();
    lprintf(0, "HLK-LD2450");
    __delay_ms(1000);
    configUART();
    //Turn on all enabled interrupts
    INTCONbits.PEIE = 1; //turn on peripheral interrupts
    INTCONbits.GIE = 1; //Turn on interrupts
    while (1) {
        if (newPacket) {
            newPacket = 0;
            target t;
            processPacket(1, &t);
            //uint16_t distance;
            //distance = (uint16_t)sqrt((float)t.x_pos * (float)t.x_pos + (float)t.y_pos * (float)t.y_pos);
            lprintf(0, "x=%d y=%d", t.x_pos, t.y_pos);
            lprintf(1, "s=%d d=%d", t.speed, t.resolution);
        }
    }
}

void configUART() {
    //RG2 is RX, RG1 is TX
    TRISGbits.RG2 = 1;
    TRISGbits.RG1 = 0;
    SPBRG2 = 30; //256000 baud
    SPBRGH2 = 0;
    TXSTA2bits.BRGH = 1;
    BAUDCON2bits.BRG16 = 1;
    TXSTA2bits.SYNC = 0; //asynchronous mode
    RCSTA2bits.SPEN = 1; //Enable the serial port
    RCSTA2bits.CREN = 1; //Enable reception
    PIE3bits.RC2IE = 1; //enable rx interrupt
}

void processPacket(int targetNum, target *pt) {
    //only do target 1 right now
    pt->x_pos = packet.target1.x_pos;
    if (pt->x_pos & 0x8000) {
        pt->x_pos &= 0x7fff;
    } else {
        pt->x_pos = -pt->x_pos;
    }
    pt->y_pos = packet.target1.y_pos;
    if (pt->y_pos & 0x8000) {
        pt->y_pos &= 0x7fff;
    } else {
        pt->y_pos = -pt->y_pos;
    }
    pt->speed = packet.target1.speed;
    if (pt->speed & 0x8000) {
        pt->speed &= 0x7fff;
    } else {
        pt->speed = -pt->speed;
    }
    pt->resolution = packet.target1.resolution;
}

void __interrupt(high_priority) HighIsr(void) {
    uint8_t rx;
    if (PIR3bits.RC2IF == 1) { //A byte is available from UART
        rx = RCREG2; 
        packet.bytes[rxCount] = rx;
        if (rxCount < HEADER_SIZE) {
            if (rx != header_values[rxCount]) {
                rxCount = 0;
            } else {
                ++rxCount;
            }
        } else {
            ++rxCount;
            if (rxCount == PACKET_SIZE) {
                rxCount = 0;
                ++packetCount;
                LATDbits.LATD0 ^= 1;
                if (packetCount % 5 == 0) {
                    //only update every 5th packet (0.5s) change this for faster or slower updates
                    newPacket = 1;
                }
            }
        }
    }
}