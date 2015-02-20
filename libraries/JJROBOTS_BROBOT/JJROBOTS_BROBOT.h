#ifndef JJROBOTS_BROBOT_h
#define JJROBOTS_BROBOT_h

class BROBOT_Class
{
  private:
	int servo_min_pwm;
	int servo_max_pwm;
	int battery;
	bool first_time;
  public:
	BROBOT_Class();
	void initServo();
    void moveServo1(int pwm);
	void moveServo2(int pwm);
	int readBattery();
};

extern BROBOT_Class BROBOT;

#endif