/*
  3PI template code for NYU "Intro to Robotics" course.
  Yann LeCun, 02/2009.
  This program was modified from an example program from Pololu. 
 */

// The 3pi include file must be at the beginning of any program that
// uses the Pololu AVR library and 3pi.
#include <pololu/3pi.h>

// This include file allows data to be stored in program space.  The
// ATmega168 has 16k of program space compared to 1k of RAM, so large
// pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

//sensor values
int const vals[5] = {0, 125, 250, 375, 500};

void update_bounds(const unsigned int *s, unsigned int *minv, unsigned int *maxv) {
	int i;
	for (i=0; i<5; i++) { 
		if (s[i]<minv[i]) minv[i] = s[i];
		if (s[i]>maxv[i]) maxv[i] = s[i];
	}
}

//Calibrates the sensor
void calibrate(unsigned int *sensors, unsigned int *minv, unsigned int *maxv) {
	//give instructions
	clear();
	lcd_goto_xy(0, 0);
	print("Fluffy");
	lcd_goto_xy(0, 1);
	print("hates u");
	
	wait_for_button_press(BUTTON_A);
	
	delay_ms(500);
	
	//activate the motors
	set_motors(40, -40);
	
	//take 20 readings from the sensors, that should be enough to calibrate
	int i;
	for (i = 0; i < 165; i++) {
		read_line_sensors(sensors, IR_EMITTERS_ON);
		update_bounds(sensors, minv, maxv);
		delay_ms(10);
	}
	
	//bring down the sensor readings...if we don't, then the reading might actually be lower than the min from time to time
	//and that creates problems for the unsigned int...because subtraction...blah blah blah
	for (i = 0; i < 5; i++)
		minv[i] -= 40;
	
	//and turn the motors off, we're done
	set_motors(0, 0);
	
	delay_ms(750);
}

//return line position
int line_position(unsigned int *sensors, unsigned int *minv, unsigned int *maxv) {
	unsigned int i, avg = 0, sum = 0;
	char seen = 0;
	
	static int last = 0;
	
	for (i = 0; i < 5; i++) {
		sensors[i] = ((long)(sensors[i] - minv[i]) * 100) / (maxv[i] - minv[i]);
		
		if (sensors[i] >= 100)
			return last;
	}
	
	for (i = 0; i < 5; i++) {
		if (sensors[i] > 25)
			seen = 1;
	
		//if we're seeing something
		if (sensors[i] > 5) {
			avg += sensors[i] * vals[i];
			sum += sensors[i];
		}
	}
	
	if (!seen) {
		if (last >= 250)
			return 500;
		else
			return 0;
	}
	
	last = avg / sum;
	
	return last;
}

//This is the main function, where the code starts.  All C programs
//must have a main() function defined somewhere.
int main() {
	//global array to hold sensor values
	unsigned int sensors[5];
	
	//global arrays to hold min and max sensor values
	//for calibration
	unsigned int minv[5] = {65500, 65500, 65500, 65500, 65500}, maxv[5] = {0, 0, 0, 0, 0};
	 
	//set up the 3pi
	pololu_3pi_init(2000);
	
	//calibrate the stuff
	calibrate(sensors, minv, maxv);

	//see if we're setting a speed
	//int speed = adjustSpeed(135);
	int const speed = 200;

	int propK = 25;
	int propI = 7000;
	
	goto loop;
	
	propKAdjust:
		set_motors(0, 0);
		
		while (1) {
			delay_ms(100);
			
			if (button_is_pressed(BUTTON_A))
				propK++;
			if (button_is_pressed(BUTTON_B))
				propK--;
			if (button_is_pressed(BUTTON_C))
				break;
			
			clear();
			print("PropK:");
			lcd_goto_xy(0, 1);
			print_long(propK);
		}
		
		goto loop;
		
	propIAdjust:
		set_motors(0, 0);
		
		while (1) {
			delay_ms(100);
			
			if (button_is_pressed(BUTTON_A))
				propI += 100;
			if (button_is_pressed(BUTTON_B))
				propI -= 100;
			if (button_is_pressed(BUTTON_C))
				break;
			
			clear();
			print("PropI:");
			lcd_goto_xy(0, 1);
			print_long(propI);
		}
	
	loop:;
	
	//holds the deriv
	int deriv;
	
	//holds the integral
	int integ = 0;
	
	//holds the last position
	int lastProp = 0;
	
	//line position relative to center
	int position = 0;
	
	long last = millis();
	
	//run in circles
	while(1) {
		if (button_is_pressed(BUTTON_A))
			goto propKAdjust;
		if (button_is_pressed(BUTTON_B))
			goto propIAdjust;	
	
		//read the line sensor values
		read_line_sensors(sensors, IR_EMITTERS_ON);
		
		//compute line positon
		position = line_position(sensors, minv, maxv);
		
		//get the middle sensors to = 0
		int prop = position - 250;
		
		//save the running time
		long now = millis();
		long diff = now - last;
		
		//calc the derivative
		deriv = ((prop - lastProp) * 10) / diff;
		
		//if the robot has changed directions, clear the integral
		if ((lastProp < 0 && prop > 0) || (prop < 0 && lastProp > 0))
			integ = prop * diff;
		else
			integ += prop * diff;
		
		//make sure integral doesn't go below 0
		if (integ < 0)
			integ *= -1;
		
		//get a proportional speed
		int propSpeed = prop * 2 + (integ / propI) + (deriv * propK);
		
		//set our last run time
		last = now;
		
		//get a proportional speed
		int left = speed+propSpeed;
		int right = speed-propSpeed;
		
		//make sure the motors are never off / going negative
		if (left <= 0) {
			int diff = 0 - left;
			right += diff - 10;
			left = 10;
		}
		if (right <= 0) {
			int diff = 0 - right;
			left += diff - 10;
			right = 10;	
		}
		
		//limit the motors to their maxes
		if (left > 255)
			left = 255;
		if (right > 255)
			right = 255;
		
		lastProp = prop;

		set_motors(left, right);
		delay_ms(1);
	}
}
