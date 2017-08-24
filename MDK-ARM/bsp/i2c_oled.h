#ifndef __I2C_OLED_H
#define __I2C_OLED_H

void OLED_WrDat(unsigned char IIC_Data);
void OLED_WrCmd(unsigned char IIC_Command);
void oled_set_pos(unsigned char x, unsigned char y);
void oled_fill(unsigned char* bmp_dat);
void oled_reflash(unsigned char* ram);
void oled_clear_screen(void);
void oled_init(void);
void oled_power_on(void);
void oled_power_off(void);
void oled_disNum(unsigned char inv, unsigned char x, unsigned char y, short num);
void disfloat_num(unsigned char x, unsigned char y, float num);

void oled_p6x8str(unsigned char inv, unsigned char x, unsigned char y, const char* ch);
//void oled_p6x8strInv(unsigned char inv, unsigned char x, unsigned char y,const char * ch);
void oled_p16x16ch(const unsigned char x, const unsigned char y,
                   const unsigned char index, const unsigned char num,
                   const char* p);
void oled_p16x16num(const unsigned char x, const unsigned char y,
                    const unsigned char num);
void oled_p6x8strAndNum(unsigned char inv, unsigned char x, unsigned char y,
                        const char* ch, short num);
void oled_printf(unsigned char x, unsigned char y, const char* fmt, ...);

extern unsigned char       GRAM[];
extern const unsigned char logoUESTC[];
extern const char          display_chinese[];
extern const char          F16x16[];

#endif
