/*
  Example of JJROBTS_OSC library.
  Code by Jose Julio and Juan Pedro . JJROBOTS.COM
*/

#include <JJROBOTS_OSC.h>

void setup()
{
  Serial.begin(115200);   // For debug output
}
void loop()
{
  OSC.MsgRead();
  if (OSC.newMessage)
	{
	Serial.print("Message received: ");
	}
}
  