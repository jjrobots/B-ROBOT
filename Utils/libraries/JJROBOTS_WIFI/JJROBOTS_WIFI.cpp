/*
	JJROBOTS_WIFI.cpp -  Library for JJROBOTS wifi module.
	Code by Jose Julio and Juan Pedro. JJROBOTS.COM

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version
		
		
*/
#include "Arduino.h"
#include "JJROBOTS_WIFI.h"

#include <avr/interrupt.h>
//#include "WProgram.h"


// Constructors ////////////////////////////////////////////////////////////////

JJWIFI_Class::JJWIFI_Class()
{
}

// Private function
void _SerialEcho()
{
char incomingByte;

  delay(400);
  //Serial.println("SerialEcho");
  while (Serial1.available() > 0) {
    incomingByte = Serial1.read();
    Serial.print(incomingByte);
  }
}


// Public Methods //////////////////////////////////////////////////////////////
void JJWIFI_Class::WifiInit()
{
	// Serial and Serial1 should be initialized on the main code...
    //Serial1.begin(115200);  // Default Wifi module Serial speed
    //if (!Serial)
    //    Serial.begin(115200);     // Default baud rate if not initialized
    //delay(1000);
}

void JJWIFI_Class::WifiSendCommand(char command[],boolean EndOfLine)
{
  if (EndOfLine){
    Serial1.println(command);
    _SerialEcho();
  }
  else{
    Serial1.print(command);
  }
  delay(10);
  //return true;
}

void JJWIFI_Class::WifiEnterCommandMode()
{
  // First we try an exit (to exit an old command mode)
  Serial1.print("exit\r");
  _SerialEcho();
  // Entramos en modo Comando
  Serial1.print("$$$");
  _SerialEcho();
  delay(10);
}

void JJWIFI_Class::WifiExitCommandMode()
{
  // First we try an exit (to exit an old command mode)
  WifiSendCommand("exit");
  delay(10);
}

void JJWIFI_Class::WifiFactoryReset()
{
  WifiEnterCommandMode();
  WifiSendCommand("factory RESET");
  WifiSendCommand("reboot");
  delay(1000);
}

// Change baudrate to 115200
void JJWIFI_Class::WifiChangeBaudRateFast()
{
  Serial1.end();
  Serial1.begin(9600);    // Deafult baud rate of Wifi module
  WifiEnterCommandMode();
  WifiSendCommand("set uart baudrate 115200");
  WifiSendCommand("save");
  WifiSendCommand("reboot");
  delay(500);
  Serial1.end();
  Serial1.begin(115200);
  delay(500);
}

// Change baudrate to 9600 (default baud rate)
void JJWIFI_Class::WifiChangeBaudRateSlow()
{
  Serial1.end();
  Serial1.begin(115200);    // Fast baud rate 
  WifiEnterCommandMode();
  WifiSendCommand("set uart baudrate 9600");
  WifiSendCommand("save");
  WifiSendCommand("reboot");
  delay(500);
  Serial1.end();
  Serial1.begin(9600);
}

void JJWIFI_Class::WifiJoin(char ssid[], char password[], boolean WPA, boolean DHCP)
{
  WifiEnterCommandMode();
  if (DHCP)
    WifiSendCommand("set ip dhcp 1");  // Enable DHCP
  else
    WifiSendCommand("set ip dhcp 0");  // Disable DHCP (use static IP)
  if (WPA){
    WifiSendCommand("set wlan phrase ",false);
    WifiSendCommand(password);
  }
  else{  // WEP 128bits
    WifiSendCommand("set wlan key ",false);
    WifiSendCommand(password);
  }
  WifiSendCommand("join ",false);
  WifiSendCommand(ssid);
  WifiExitCommandMode();
}

void JJWIFI_Class::WifiSetIP(char ip[], char netmask[], char gateway[])
{
  WifiEnterCommandMode();
  WifiSendCommand("set ip address ",false);
  WifiSendCommand(ip);
  WifiSendCommand("set ip netmask ",false);
  WifiSendCommand(netmask);
  WifiSendCommand("set ip gateway ",false);
  WifiSendCommand(gateway);
  WifiExitCommandMode();
}

void JJWIFI_Class::WifiEnableUDP(char local_port[], char remote_port[], char remote[])
{
  WifiEnterCommandMode();
  WifiSendCommand("set ip proto 1");
  WifiSendCommand("set ip localport ",false);
  WifiSendCommand(local_port);
  WifiSendCommand("set ip host ",false);
  WifiSendCommand(remote);
  WifiSendCommand("set ip remote ",false);
  WifiSendCommand(remote_port);
  WifiSendCommand("save");
  WifiSendCommand("reboot");   // We need to reboot to take effect
  delay(500);
  //Wifi_ExitCommandMode();
}

void JJWIFI_Class::WifiEnableTCPUDP(char local_port[], char remote_port[], char remote[])
{
  WifiEnterCommandMode();
  WifiSendCommand("set ip proto 3");
  WifiSendCommand("set ip localport ",false);
  WifiSendCommand(local_port);
  WifiSendCommand("set ip host ",false);
  WifiSendCommand(remote);
  WifiSendCommand("set ip remote ",false);
  WifiSendCommand(remote_port);
  WifiSendCommand("save");
  WifiSendCommand("reboot");   // We need to reboot to take effect
  delay(500);
  //Wifi_ExitCommandMode();
}


void JJWIFI_Class::WifiViewConfig()
{
  WifiEnterCommandMode();
  WifiSendCommand("ver");
  WifiSendCommand("get apmode");
  WifiSendCommand("get ip");
  //Wifi_SendCommand("get dns");
  WifiSendCommand("get wlan");
  WifiExitCommandMode();
}

void JJWIFI_Class::WifiAP(char ssidname[], char passphrase[])
{
  WifiEnterCommandMode();
  WifiSendCommand("set wlan join 7");
  // Old firmwares
  WifiSendCommand("set wlan ssid ",false);
  WifiSendCommand(ssidname);
  WifiSendCommand("set wlan chan 1");
  // Compatibility with firmware 4.41 Soft AP with WPA2 security
  WifiSendCommand("set apmode ssid ",false);
  WifiSendCommand(ssidname);
  WifiSendCommand("set apmode passphrase ",false);
  WifiSendCommand(passphrase);
  WifiSendCommand("set ip address 192.168.1.1");
  WifiSendCommand("set ip netmask 255.255.255.0");
  WifiSendCommand("set ip dhcp 4");   // DHCP server
  WifiSendCommand("save");
  WifiSendCommand("reboot");   // We need to reboot to take effect
}

// make one instance for the user to use
JJWIFI_Class JJWIFI;