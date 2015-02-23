#ifndef JJROBOTS_WIFI_h
#define JJROBOTS_WIFI_h

#include "Arduino.h"

class JJWIFI_Class
{
  private:
  public:
	JJWIFI_Class();
    void WifiInit();
    void WifiSendCommand(char command[],boolean EndOfLine = true);
    void WifiEnterCommandMode();
    void WifiExitCommandMode();
    void WifiFactoryReset();
    void WifiChangeBaudRateFast();
    void WifiChangeBaudRateSlow();
    void WifiJoin(char ssid[], char password[], boolean WPA, boolean DHCP = true);
    void WifiSetIP(char ip[], char netmask[], char gateway[]);
    void WifiEnableUDP(char local_port[], char remote_port[], char remote[]);
	void WifiEnableTCPUDP(char local_port[], char remote_port[], char remote[]);
    void WifiAP(char ssidname[]);
	void WifiViewConfig();
};

extern JJWIFI_Class JJWIFI;

#endif