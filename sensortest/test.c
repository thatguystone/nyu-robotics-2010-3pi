/*
Andrew Stone
Stanley Dady

NYU, Robotics - Spring 2010

Dead Reckoning
*/

//The 3pi include file must be at the beginning of any program that
//uses the Pololu AVR library and 3pi.
#include <pololu/3pi.h>

//This include file allows data to be stored in program space.  The
//ATmega168 has 16k of program space compared to 1k of RAM, so large
//pieces of static data should be stored in program space.
#include <avr/pgmspace.h>

//stuff for running
#include "3pi_kimenatics.h"

#include <math.h>

//sensor values
int const vals[5] = {0, 125, 250, 375, 500};

long homeX, homeY;

//get the current angle we are heading on
long lastTheta, alpha, tAlpha, theta;

//if we are on the line, default to yes so that we follow from the start
char seen = 1;

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
	//say something to the user
	clear();
	lcd_goto_xy(0, 0);
	print(" Fluffy");
	lcd_goto_xy(0, 1);
	print("A=Go!");
	
	//wait on the calibration button
	wait_for_button_press(BUTTON_A);
	
	//wait for the user to move his hand
	delay_ms(500);

	//activate the motors
	set_motors(40, -40);
	
	//take 165 readings from the sensors...why not?
	int i;
	for (i = 0; i < 168; i++) {
		read_line_sensors(sensors, IR_EMITTERS_ON);
		update_bounds(sensors, minv, maxv);
		delay_ms(10);
	}

	//and turn the motors off, we're done
	set_motors(0, 0);
	
	delay_ms(750);
}

//updates the position we think our robot is at (given our left and right motor speeds for nice calculations)
//everything using DT is divided by 1000 to get it back into milliseconds
void updatePosition(int left, int right, long dt) {
	//alpha = (theta_i + theta_(i+1)) / 2
	//theta_(i+1) = theta_i + dt * motor2angle
	//x_(i+1) = motor2speed * dt * sin(alpha) + x_i
	//y_(i+1) = motor2speed * dt * cos(alpha) + y_i

	static int i = 0;
	
    theta += dt * motor2angle(left, right);
	
	alpha = (lastTheta + theta) / c2;
	int m2s = motor2speed((left + right) / c2);
    homeX += (m2s * dt * Sin(alpha / 1000)) / cmillion;
    homeY += (m2s * dt * Cos(alpha / 1000)) / cmillion;
	
	lastTheta = theta;
	
	if (i++ % 200 == 0) {
		clear();
		print_long(motor2angle(left, right));
		lcd_goto_xy(0, 1);
		print_long(theta);
	}
}

//since we use this in multiple places, just make it easier to get to
int getCalibratedSensor(unsigned int s, unsigned int min, unsigned int max) {
	//cast everything...for some reason, without all these explicit casts, nothing calculates correctly.
	return ((long)(((int)((int)s - (int)min)) * 100)) / (max - min);
}

//return line position
int line_position(unsigned int *sensors, unsigned int *minv, unsigned int *maxv, int *range) {
	//a temporary value to hold our current readings in INT form
	int s[5];
	
	//for iterating
	int i;
	
	//for doing calculations -- make sure there is no overflow
	unsigned long avg = 0, sum = 0;
	
	//hold our last value of this function, in case we lose the line
	static int last = 0;
	
	for (i = 0; i < 5; i++) {
		s[i] = getCalibratedSensor(sensors[i], minv[i], maxv[i]);
		
		//if above 100, reset!
		if (s[i] >= 100)
			s[i] = 100;
		
		//if the reading is below 0, reset!
		if (s[i] < 0)
			s[i] = 0;
		
		//if we get a crap reading (ie. it jumps too far in one direction), ignore it
		if (s[i] < (range[i] - 30) || s[i] > (range[i] + 30))
			s[i] = range[i];
		
		//save our last reading for our range
		range[i] = s[i];
	}
	
	char seenThisRound = 0;
	
	for (i = 0; i < 5; i++) {
		//did we see the line?
		if (s[i] > 13)
			seenThisRound = 1;
	
		//if we're seeing something
		if (s[i] > 10) {
			//again, casting here is necessary...otherwise, it has all sorts of fun...
			avg += (long)((long)s[i] * vals[i]);
			sum += s[i];
		}
	}
	
	seen = seenThisRound;
	
	if (!seenThisRound) {
		return last;
	}
	
	return last = avg / sum;
}

//This is the main function, where the code starts.  All C programs
//must have a main() function defined somewhere.
int main() {
	//holds sensor values
	unsigned int sensors[5];
	
	//hold min and max sensor values for calibration
	unsigned int minv[5] = {65500, 65500, 65500, 65500, 65500}, maxv[5] = {0, 0, 0, 0, 0};
	
	//holds the previous value so that we can make sure the sensor didn't report a crap value
	int range[5];
	
	seen = 1;
	homeX = 0;
	homeY = 0;
	lastTheta = 0;
	alpha = 0;
	tAlpha = 0;
	theta = 0;
	 
	//set up the 3pi
	pololu_3pi_init(2000);
	
	//calibrate the stuff
	calibrate(sensors, minv, maxv);
	
	//set our range from our calibrated readings so that it doesn't break the other readings when we start moving
	int i;
	for (i = 0; i < 5; i++)
		range[i] = getCalibratedSensor(sensors[i], minv[i], maxv[i]);

	//set the speed
	int const speed = 30;

	//holds the deriv
	int deriv;
	
	//holds the integral
	int integ = 0;
	
	//holds the last position
	int lastProp = 0;
	
	//line position relative to center
	int position = 0;
	
	long last = millis();
	
	/*
	load_custom_characters();
	while (1) {
		read_line_sensors(sensors, IR_EMITTERS_ON);
		position = line_position(sensors, minv, maxv, range);
		display_bars(sensors, minv, maxv);
	}
	*/
	
	//run in circles
	while (seen) {
		//read the line sensor values
		read_line_sensors(sensors, IR_EMITTERS_ON);
		
		//compute line positon
		position = line_position(sensors, minv, maxv, range);
		
		//get the middle sensors to = 0
		int prop = position - 250;
		
		//save the running time
		long now = millis();
		long diff = now - last;
		
		//calc the derivative
		deriv = prop - lastProp;
		
		//if the robot has changed directions, clear the integral
		if ((lastProp < 0 && prop > 0) || (prop < 0 && lastProp > 0))
			integ = 0;
		else
			integ += prop;
		
		//get a proportional speed
		int propSpeed = (prop / 4) + (integ / 6000) + (deriv / 3);
		
		//set our last run time
		last = now;
		
		//get a proportional speed
		int left = speed+propSpeed;
		int right = speed-propSpeed;
		
		//make sure the motors are never off / going negative
		if (left <= 0) {
			right += (0 - left) - 20;
			left = 20;
		}
		if (right <= 0) {
			left += (0 - right) - 20;
			right = 20;	
		}
		
		//limit the motors to their maxes
		if (left > 100)
			left = 100;
		if (right > 100)
			right = 100;
		
		lastProp = prop;
		
		//update our guestimate of the position of the robot
		updatePosition(left, right, diff);

		set_motors(left, right);
	}
	
	//once we get here, we are sure that we didn't see the line, so stop
	set_motors(0, 0);
	clear();
	print("Lost");
	
	delay_ms(1000);
	
	clear();
	print_long(homeX);
	lcd_goto_xy(4, 0);
	print_long(alpha);
	lcd_goto_xy(0, 1);
	print_long(homeY);
	lcd_goto_xy(4, 1);
	print_long(theta);
	
	while (1);
	
	//go home
	int thetaToHome = (180 - Tan(homeY / homeX));
	int wait = angle2time(thetaToHome);
	
	//start turning...
	set_motors(30, -30);
	delay_ms(wait);
	set_motors(0, 0);
	
	//and find out how long it takes to get home
	int dist = sqrt(pow(homeX, 2) + pow(homeY, 2));
	
	wait = distance2time(dist);
	
	set_motors(30, 30);
	delay_ms(wait);
	set_motors(0, 0);
	
	while (1);
}
