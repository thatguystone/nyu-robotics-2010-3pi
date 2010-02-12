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
//const int vals[5] = {0, 120, 250, 380, 500};
int vals[5] = {0, 30, 250, 470, 500};

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
	print("A-Cal");
	
	//and do some stuff for calibration
	while (1) {
		//A recalibrates all the sensors
		if (button_is_pressed(BUTTON_A)) {
			//give the user time to move his hand away
			delay_ms(500);
			
			//activate the motors
			set_motors(40, -40);
			
			//take 20 readings from the sensors, that should be enough to calibrate
			int i;
			for (i = 0; i < 88; i++) {
				read_line_sensors(sensors, IR_EMITTERS_ON);
				update_bounds(sensors, minv, maxv);
				delay_ms(20);
			}
			
			//and turn the motors off, we're done
			set_motors(0, 0);
			
			delay_ms(500);
			
			break;
		}
	}
}


// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize() {
	//This must be called at the beginning of 3pi code, to set up the
	//sensors.  We use a value of 2000 for the timeout, which
	//corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
	pololu_3pi_init(2000);
	
	//let's kill that battery!
	red_led(1);
	green_led(1);
}

//return line position
int line_position(unsigned int *sensors, unsigned int *minv, unsigned int *maxv) {
	unsigned int i, avg = 0, sum = 0;
	char seen = 0;
	
	static int last = 0;
	
	for (i = 0; i < 5; i++)
		sensors[i] = ((long)(sensors[i] - minv[i]) * 100) / (maxv[i] - minv[i]);
	
	for (i = 0; i < 5; i++) {
		if (sensors[i] > 40)
			seen = 1;
	
		//if we're seeing something
		if (sensors[i] > 25) {
			avg += sensors[i] * vals[i];
			sum += sensors[i];
		}
	}
	
	if (!seen) {
		if (last >= 200)
			return 500;
		else
			return 0;
	}
	
	last = avg / sum;
	
	return last;
}

//adjusts the speed based on user input
int adjustSpeed(int speed) {
	while(1) {
		if (button_is_pressed(BUTTON_A))
			speed += 10;
		if (button_is_pressed(BUTTON_B))
			speed -= 10;
		if (button_is_pressed(BUTTON_C))
			break;
		
		clear();
		print("Spd: ");
		print_long(speed);
		delay_ms(100);
	}
	
	clear();
	print("Going...");
	
	//give the user time to move his hand
	delay(750);
	
	return speed;
}

//This is the main function, where the code starts.  All C programs
//must have a main() function defined somewhere.
int main() {
	//global array to hold sensor values
	unsigned int sensors[5];
	
	//global arrays to hold min and max sensor values
	//for calibration
	unsigned int minv[5], maxv[5];
	 
	//set up the 3pi, and wait for B button to be pressed
	initialize();
	
	//calibrate the stuff
	calibrate(sensors, minv, maxv);

	//see if we're setting a speed
	int speed = adjustSpeed(150);

	//holds the deriv
	int deriv;
	
	int propK = vals[1], propI = vals[3];
	
	goto loop;
	
	/*
	propKAdjust:
		while (1) {
			set_motors(0, 0);
			if (button_is_pressed(BUTTON_A))
				propK += 10;
			if (button_is_pressed(BUTTON_B))
				propK -= 10;
			if (button_is_pressed(BUTTON_C))
				break;
			
			vals[1] = propK;
			
			clear();
			print("K:");
			print_long(propK);
			delay_ms(100);
		}
		goto loop;
	
	propIAdjust:
		while (1) {
			set_motors(0, 0);
			if (button_is_pressed(BUTTON_A))
				propI += 10;
			if (button_is_pressed(BUTTON_B))
				propI -= 10;
			if (button_is_pressed(BUTTON_C))
				break;
			
			vals[3] = propI;
			
			clear();
			print("I:");
			print_long(propI);
			delay_ms(100);
		}
	*/
	
	propIAdjust: propKAdjust:
		//see if we're setting a speed
		speed = adjustSpeed(speed);
	loop:
	;
	
	//holds the integral
	int integ = 0;
	
	//holds the last position
	int lastProp = 0;
	
	//line position relative to center
	int position = 0;
	
	//run in circles
	while(1) {
		if (button_is_pressed(BUTTON_B))
			goto propKAdjust;
		if (button_is_pressed(BUTTON_A))
			goto propIAdjust;
	
		//Read the line sensor values
		read_line_sensors(sensors, IR_EMITTERS_ON);
		
		//compute line positon
		position = line_position(sensors, minv, maxv);
		
		//get the middle sensors to = 0
		int prop = position - 250;
		
		//clear();
		//print_long(prop);
		
		//calc the derivative
		deriv = prop - lastProp;
		//and integral
		integ += prop; 
		lastProp = prop;
		
		int propSpeed = prop/3 + integ/700 + deriv*22;
		
		int left = speed+propSpeed;
		int right = speed-propSpeed;
		
		if (left < 0) {
			int diff = 0 - left;
			right += diff - 10;
			left = 10;
		}
		if (right < 0) {
			int diff = 0 - right;
			left += diff - 10;
			right = 10;	
		}
		
		if (left > 255)
			left = 255;
		if (right > 255)
			right = 255;
		
		/*
		lcd_goto_xy(0, 1);
		print_long(propSpeed);
		lcd_goto_xy(4, 0);
		print_long(left);
		lcd_goto_xy(4, 1);
		print_long(right);
		
		delay_ms(250);
		continue;
		*/
		
		set_motors(left, right);
	}
}
