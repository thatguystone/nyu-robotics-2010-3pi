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
int const vals[5] = {0, 30, 250, 470, 500};

// Data for generating the characters used in load_custom_characters
// and display_readings.  By reading levels[] starting at various
// offsets, we can generate all of the 7 extra characters needed for a
// bargraph.  This is also stored in program space.
const char levels[] PROGMEM = {
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b00000,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111,
	0b11111
};

char display_characters[9] = { ' ', 0, 1, 2, 3, 4, 5, 6, 255 };

// This function loads custom characters into the LCD.  Up to 8
// characters can be loaded; we use them for 7 levels of a bar graph.
void load_custom_characters() {
	lcd_load_custom_character(levels+0,0); // no offset, e.g. one bar
	lcd_load_custom_character(levels+1,1); // two bars
	lcd_load_custom_character(levels+2,2); // etc...
	lcd_load_custom_character(levels+3,3);
	lcd_load_custom_character(levels+4,4);
	lcd_load_custom_character(levels+5,5);
	lcd_load_custom_character(levels+6,6);
	clear(); // the LCD must be cleared for the characters to take effect
}

// This function displays the sensor readings using a bar graph.
void display_bars(const unsigned int *s, const unsigned int *minv, const unsigned int* maxv) {
	// Initialize the array of characters that we will use for the
	// graph.  Using the space, and character 255 (a full black box).
	
	lcd_goto_xy(0,1);
	
	unsigned char i;
	for (i=0;i<5;i++) {
		int c = ((int)s[i]-(int)minv[i])*9/((int)maxv[i]-(int)minv[i]);
		c = (c<0)?0:(c>8)?8:c;
		// if (i==0) {print_long(s[0]); print_long(c); }
		print_character(display_characters[c]);
	}
}

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
	
	while (!button_is_pressed(BUTTON_A));
	
	delay_ms(500);
	
	//activate the motors
	set_motors(30, -30);
	
	//take 20 readings from the sensors, that should be enough to calibrate
	int i;
	for (i = 0; i < 250; i++) {
		read_line_sensors(sensors, IR_EMITTERS_ON);
		update_bounds(sensors, minv, maxv);
		delay_ms(10);
	}
	
	//and turn the motors off, we're done
	set_motors(0, 0);
	
	//and do some stuff for calibration
	while (!button_is_pressed(BUTTON_C)) {
		clear();
		print("Line:");
		read_line_sensors(sensors, IR_EMITTERS_ON);
		display_bars(sensors, minv, maxv);
		delay_ms(100);
	}
	
	clear();
	print("Going...");
}


// Initializes the 3pi, displays a welcome message, calibrates, and
// plays the initial music.
void initialize() {
	//This must be called at the beginning of 3pi code, to set up the
	//sensors.  We use a value of 2000 for the timeout, which
	//corresponds to 2000*0.4 us = 0.8 ms on our 20 MHz processor.
	pololu_3pi_init(2000);
	
	load_custom_characters();
	
	//let's kill that battery!
	//red_led(1);
	//green_led(1);
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
		if (sensors[i] > 15) {
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
	 
	//set up the 3pi, and wait for B button to be pressed
	initialize();
	
	//calibrate the stuff
	calibrate(sensors, minv, maxv);

	//see if we're setting a speed
	//int speed = adjustSpeed(135);
	int const speed = 150;

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
		//Read the line sensor values
		read_line_sensors(sensors, IR_EMITTERS_ON);
		
		//compute line positon
		position = line_position(sensors, minv, maxv);
		
		//get the middle sensors to = 0
		int prop = position - 250;
		
		//calc the derivative
		deriv = prop - lastProp;
		//and integral
		integ += prop; 
		lastProp = prop;
		
		long now = millis();
		long diff = now - last;
		int propSpeed = prop/2 + (((integ/1000) + (deriv*30)) * diff);
		last = now;
		
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
		
		set_motors(left, right);
	}
}
