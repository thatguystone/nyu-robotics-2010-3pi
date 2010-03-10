#include <stdlib.h>
#include <stdio.h>
#include <math.h>

long motor2speed(int v) {
  // v*4.7682 - 33 mm/s
  // This is robot 493 with fully charged batteries.
  // your mileage (millimeterage) may vary
  int r = ( (v>0)?v:-v )*430/8 - 680;
  r = (r>0)?r:0;
  if (v>=0) {
    return (long)(r);
  } else {
    return (long)(-r);
  }
}

// converts the two motor speeds to
// a rotational speed in degrees per second.
// Positive rotation is clockwise (like the heading of a boat).
long motor2angle(int ml, int mr) {
  // a = (vl-vr)*180/(pi*w) = (vl-vr)*360/(2*pi*w)
  // where: 
  // vl,vr: speeds of left and right wheels in 1/10 mm/s.
  // w: width of the robot in 1/10th of mm
  long vl = motor2speed(ml);
  long vr = motor2speed(mr);
  return (vl-vr)*2/30;
}

int angle2time(int angle) {
	//130°/sec @ speed 30
	return (angle * 1000) / 130;
}

int distance2time(int distance) {
	return (distance * 1000) / 950;
}

int main() {
	int dist = sqrt(pow(-23, 2) + pow(557, 2));
	printf("%d - %d \n", dist, distance2time(dist));
}
