#include "lcd.h"

void lcd_delay(void)
{
	for(uint16_t i = 0; i <= 4000; i++)
	{

	}
}

void lcd_write(uint8_t wrt)
{
    if(((wrt >> 7) & 0x01) == 1) {d7_set();} else {d7_reset();}
    if(((wrt >> 6) & 0x01) == 1) {d6_set();} else {d6_reset();}
    if(((wrt >> 5) & 0x01) == 1) {d5_set();} else {d5_reset();}
    if(((wrt >> 4) & 0x01) == 1) {d4_set();} else {d4_reset();}
    if(((wrt >> 3) & 0x01) == 1) {d3_set();} else {d3_reset();}
    if(((wrt >> 2) & 0x01) == 1) {d2_set();} else {d2_reset();}
    if(((wrt >> 1) & 0x01) == 1) {d1_set();} else {d1_reset();}
    if(( wrt & 0x01)       == 1) {d0_set();} else {d0_reset();}
}


void lcd_write_data(uint8_t data)
{
	rs1;
	e1;
	lcd_write(data);
	lcd_delay();
	e0;
}


void lcd_write_command(uint8_t com)
{
	rs0;
	e1;
	lcd_write(com);
	lcd_delay();
	e0;
}

void lcd_initialization(void)
{
	HAL_Delay(40);
	lcd_write_command(0x30);
	HAL_Delay(1);
	lcd_write_command(0x30);
	HAL_Delay(1);
	lcd_write_command(0x30);
	HAL_Delay(1);
	lcd_write_command(0x38);//режим 4 бит, 2 линии (для нашего большого дисплея это 4 линии), шрифт 5х8
	HAL_Delay(1);
	lcd_write_command(0x08);//еще раз для верности
	HAL_Delay(1);
	lcd_write_command(0x01);//уберем мусор
	HAL_Delay(2);
	lcd_write_command(0x06);//пишем влево.
	HAL_Delay(1);
	lcd_write_command(0x0F);//возврат курсора в нулевое положение
	lcd_write_command(0x3A);
	HAL_Delay(2);
}

void lcd_set_cursor(uint8_t x, uint8_t y)
{
    switch(y)
    {
	case 0:
		lcd_write_command(x|0x80);
		HAL_Delay(1);
		break;
	case 1:
		lcd_write_command((0x40+x)|0x80);
		HAL_Delay(1);
		break;
	case 2:
		lcd_write_command((0x14+x)|0x80);
		HAL_Delay(1);
		break;
	case 3:
		lcd_write_command((0x54+x)|0x80);
		HAL_Delay(1);
		break;
    }
}

void lcd_draw_char(char chr)
{
	lcd_write_data(chr);
}

void lcd_draw_text(char* str)
{
	uint8_t count = 0;
	while(str[count] != 0)
	{
		lcd_write_data(str[count]);
		count++;
	}
}

void lcd_clear_display(void)
{
	lcd_write_command(0x01);
}




