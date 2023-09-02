#ifndef LCD_H_
#define LCD_H_

#include "stm32f4xx_hal.h"


//—————————————-
#define d0_set()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET)
#define d1_set()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET)
#define d2_set()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET)
#define d3_set()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET)
#define d4_set()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET)
#define d5_set()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET)
#define d6_set()   HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_SET)
#define d7_set()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET)

#define d0_reset() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET)
#define d1_reset() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET)
#define d2_reset() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET)
#define d3_reset() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET)
#define d4_reset() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET)
#define d5_reset() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET)
#define d6_reset() HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, GPIO_PIN_RESET)
#define d7_reset() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET)

#define e1    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET) // установка линии E в 1
#define e0    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET) // установка линии E в 0
#define rs1   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET) // установка линии RS в 1 (данные)
#define rs0   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET) // установка линии RS в 0 (команда)


void lcd_write(uint8_t wrt);
void lcd_write_data(uint8_t data);
void lcd_write_command(uint8_t com);

void lcd_initialization(void);

void lcd_set_cursor(uint8_t x, uint8_t y);
void lcd_draw_char(char chr);
void lcd_draw_text(char* str);

void lcd_clear_display(void);
void lcd_return_home(void);
void lcd_entry_mode_set(uint8_t ID, uint8_t SH);
void lcd_display_onoff_control(uint8_t D, uint8_t C, uint8_t B);
void lcd_cursor_or_display_shift(uint8_t SC, uint8_t RL);
void lcd_function_set(uint8_t DL, uint8_t P);

#endif /* LCD_H_ */
