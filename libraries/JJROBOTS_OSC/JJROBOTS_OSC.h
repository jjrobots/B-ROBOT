#ifndef JJROBOTS_OSC_h
#define JJROBOTS_OSC_h

#include "Arduino.h"

int Serial1_available(void);
unsigned char Serial1_read(void);
void Serial1_write(uint8_t c);
void Serial1_print(const char str[]);
void Serial1_println(const char str[]);
void Serial1_flush();

class JJROBOTS_OSC_Class
{
  private:
    // UPD input buffer
    char UDPBuffer[8];
    // OSC message read variables
    unsigned char readStatus;
    unsigned char readCounter;
    unsigned char readNumParams;
    unsigned char commandType;
    unsigned char touchMessage;

    //char OSC_led1[21]="/1/led1\x00,f\x00\x00\x00\x00\x00\x00"; // Message for LED1
    //char OSC_led1[21] = {'/','1','/','l','e','d','1','\0',',','f','\0','\0','\0','\0','\0','\0'};
	
	//char OSC_fadder1[21]="/2/fadder1\x00\x00,f\x00\x00\x00\x00\x00\x00";  // Message for fadder1 page 2
    
    float extractParamFloat1();
    float extractParamFloat2();

  public:
  // Controls
    uint8_t page;
	uint8_t newMessage;
    float fadder1;
    float fadder2;
    float fadder3;
    float fadder4;
    float xy1_x;
    float xy1_y;
    float xy2_x;
    float xy2_y;
    uint8_t push1;
    uint8_t push2;
    uint8_t push3;
    uint8_t push4;
    uint8_t toggle1;
    uint8_t toggle2;
    uint8_t toggle3;
    uint8_t toggle4;

	JJROBOTS_OSC_Class();
    void MsgSend(char *c,unsigned char msgSize, float p);
    void MsgRead();
    void MsgRead2();
    void ParseMsg();
	
};

extern JJROBOTS_OSC_Class OSC;

#endif