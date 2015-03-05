// CONFIGURATION UTILITY FOR RNXV WIFI MODULE
// Author: JJROBOTS.COM (Jose Julio & Juan Pedro)
// Date: 02/09/2014
// Updated: 17/02/2015
// Version: 1.01
// License: GPL v2

#include <JJROBOTS_WIFI.h>

void setup()
{
  delay(5000);
  Serial.begin(115200);

  // Wifi module initialization
  // We dont know if the module is at default 9600 or at 115200 so we reset on both baud rates...
  Serial.println("Factory Reset...");
  Serial1.begin(9600);
  JJWIFI.WifiInit();
  JJWIFI.WifiFactoryReset();
  Serial1.end();
  Serial1.begin(115200);  // Default Wifi module Serial speed
  JJWIFI.WifiInit();
  JJWIFI.WifiFactoryReset();
  Serial.println("Wifi configuration...");  // CONFIGURATION ONLY NEEDED FIRST TIME
  JJWIFI.WifiChangeBaudRateFast();
  JJWIFI.WifiEnableTCPUDP("2222","2223","192.168.1.11"); // Port 2222
  JJWIFI.WifiAP("JJROBOTS","12345678");    // Soft AP mode SSID:"JJROBOTS" pass "12345678", you could change the name...
										   // The password only works for modules with firmware 4.41 or above
  
  delay(2000);
  Serial.println();
  Serial.println("CONFIG:");
  JJWIFI.WifiViewConfig();
  Serial.println();
  Serial.println("END!!");
  Serial.println("The module should be ready to use!");
  Serial.println();
}


// MAIN LOOP
void loop() 
{ 
}




