#include <pololu/3pi.h>

int main() {
	pololu_3pi_init(2000);
	
	wait_for_button_press(BUTTON_A);
	delay_ms(500);
	
	set_motors(-30, 30);
	delay_ms(1300);
	set_motors(0, 0);
	
	clear();
	print("A-Go");
	lcd_goto_xy(0, 1);
	print_long(30);

	wait_for_button_press(BUTTON_A);
	delay_ms(500);
	
	set_motors(255,255);
	delay_ms(3000);
	set_motors(0, 0);
	
	clear();
	print("A-Go");
	lcd_goto_xy(0, 1);
	print_long(40);
	
	wait_for_button_press(BUTTON_A);
	delay_ms(500);
	
	set_motors(40, 40);
	delay_ms(1000);
	set_motors(0, 0);
	
	clear();
	print("B-Spin");
	lcd_goto_xy(0, 1);
	print_long(30);
	
	wait_for_button_press(BUTTON_B);
	delay_ms(500);
	
	set_motors(30, -30);
	delay_ms(1000);
	set_motors(0, 0);
	
	clear();
	print("B-Spin");
	lcd_goto_xy(0, 1);
	print_long(40);
	
	wait_for_button_press(BUTTON_B);
	delay_ms(500);
	
	set_motors(40, -40);
	delay_ms(1000);
	set_motors(0, 0);
}
