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

//for doing math at the end...when we're not moving...so it can take its time.
#include <math.h>

//sensor values
int const vals[5] = {0, 125, 250, 375, 500};

//hold our current position
long homeX, homeY;

//get the current angle we are heading on
long lastTheta, alpha, theta;

//if we are on the line, default to yes so that we follow from the start
char seen = 1;

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
	
	//delay_ms(750);
}

//updates the position we think our robot is at (given our left and right motor speeds for nice calculations)
//everything using DT is divided by 1000 to get it back into milliseconds
void updatePosition(int left, int right, long dt) {
    theta += dt * motor2angle(left, right);
	
	alpha = (lastTheta + theta) / c2;
	int m2s = motor2speed((left + right) / c2) * dt;
	homeX += (m2s * Sin(alpha / c1000)) / cmillion;
	homeY += (m2s * Cos(alpha / c1000)) / cmillion;
	
	lastTheta = theta;
}

//since we use this in multiple places, just make it easier to get to
inline int getCalibratedSensor(unsigned int s, unsigned int min, unsigned int max, int *range) {
	//cast everything...for some reason, without all these explicit casts, nothing calculates correctly.
	int reading = ((long)(((int)((int)s - (int)min)) * 100)) / (max - min);
	
	//if above 100, reset!
	if (reading >= 100)
		reading = 100;
	
	//if the reading is below 0, reset!
	if (reading < 0)
		reading = 0;
	
	//if we get a crap reading (ie. it jumps too far in one direction), ignore it
	if (*range > 1 && (reading < (*range - 30) || reading > (*range + 30)))
		reading = *range;
	
	//save our last reading for our range
	*range = reading;
	
	return reading;
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
	
	for (i = 0; i < 5; i++)
		s[i] = getCalibratedSensor(sensors[i], minv[i], maxv[i], &range[i]);
	
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

void goHome() {
	//these values appear to be 4 times larger than what they should be...
	//adjusting the constants screwed up all the other calculations...so adjust here.
	homeX /= 4;
	homeY /= 4;
	theta /= 1000;
	
	//dramatic pause
	delay_ms(2000);
	
	//tell the user what we're doing...
	/*clear();
	print("Going");
	lcd_goto_xy(0, 1);
	print("home!");
	*/
	
	//calculate the time angle to turn at and the time to wait for this turn
	int thetaToHome = (theta < 0 ? 180 - theta : 180 + (90 -  theta));
	int wait = angle2time(thetaToHome);
	
	//start turning...
	set_motors(-30, 30);
	delay_ms(wait);
	set_motors(0, 0);
	
	//and find out how long it takes to get home and how far we need to go
	int dist = sqrt((homeX*homeX) + (homeY*homeY)); //pow() doesn't work?
	wait = distance2time(dist - 20);
	
	//go home!
	set_motors(30, 30);
	delay_ms(wait);
	set_motors(0, 0);
}

//This is the main function, where the code starts.  All C programs
//must have a main() function defined somewhere.
int main() {
	//holds sensor values
	unsigned int sensors[5];
	
	//hold min and max sensor values for calibration
	unsigned int minv[5] = {65500, 65500, 65500, 65500, 65500}, maxv[5] = {0, 0, 0, 0, 0};
	
	//holds the previous value so that we can make sure the sensor didn't report a crap value
	int range[5] = {-1, -1, -1, -1, -1};
	
	//reset everything...(seems like there's a bug with multiple equals, but didn't test it too much...could just be me)
	seen = 1;
	homeX = 0;
	homeY = 0;
	lastTheta = 0;
	alpha = 0;
	theta = 0;
	 
	//set up the 3pi
	pololu_3pi_init(2000);
	
	//calibrate the stuff
	calibrate(sensors, minv, maxv);
	
	//set our range from our calibrated readings so that it doesn't break the other readings when we start moving
	int i;
	for (i = 0; i < 5; i++)
		range[i] = getCalibratedSensor(sensors[i], minv[i], maxv[i], &range[i]);

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
			right += (0 - left) - 25;
			left = 25;
		}
		if (right <= 0) {
			left += (0 - right) - 25;
			right = 25;	
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
	
	clear();
	print_long(homeX);
	
	lcd_goto_xy(4, 0);
	print_long(theta);
	lcd_goto_xy(4, 1);
	print_long(theta / 1000);
	
	lcd_goto_xy(0, 1);
	print_long(homeY);
	delay_ms(1000);
	
	goHome();
	
	while (1);
}
