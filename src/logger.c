#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "logger.h"

// https://github.com/rxi/log.c

#define FLOAT_MAX_PRECISION     6
#define LOG_BUF_SIZE            (64 * 64)
#define LOG_MAX_COLS            64
#define LOG_MAX_LINES           (LOG_BUF_SIZE / LOG_MAX_COLS)

volatile uint16_t idx_line = 0;
volatile uint16_t print_line = 0;

static uint8_t log_buffer[LOG_MAX_LINES][LOG_MAX_COLS];

#ifdef LOG_USE_COLOR
static const char *log_colors[] = {
  "\x1b[00m", // Text
  "\x1b[94m", // Blue
  "\x1b[32m", // Green
  "\x1b[36m", // Cyan
  "\x1b[33m", // Yellow
  "\x1b[31m", // Red
  "\x1b[35m"  // Magenta
};
#endif

/**
 * @brief Attempt to implement atomic compare an swap
 * TODO: Finish implementation and fix multiple bugs
 * 
 * @param p 		: pointer to variable
 * @param new 		: new value
 * @param cur 		: expected value
 * @return uint16_t : value befor swap
 */
//__attribute__((naked))
inline
uint16_t cas(volatile uint16_t *p, uint16_t new, uint16_t cur){
	
	if(*p != cur){
		return *p;
	}
	
	*p = new;
	
	return cur;
}

static uint16_t next_line(volatile uint16_t *ptr){
	uint16_t cur, new;
	do{
    	cur = *ptr;
    	new = (cur + 1) % LOG_MAX_LINES;
	}while(cas(ptr, new, cur) != cur);

    return cur;
}


/**
 * @brief Convert 32-bit integer number to array of chars, '\0' not included
 *
 * \param dst 	:	pointer to destination buffer, must have minimum of ndig
 * \param val	:	value to be converted
 * \param radix	:	base of convertion [-10,10,16]
 * \param ndig 	:	minimum number of digits, ndig > 0 pad with ' ', ndig < 0 pad with '0'
 * \return 		:	number of digits written to dst
 * */
static uint32_t xpitoa(char *dst, int32_t val, int radix, int ndig){
	uint8_t c, sgn = 0, pad;
	uint32_t v;
	char *buf;

	if (radix < 0) {
		radix = -radix;
		if (val < 0) {
			val = -val;
			sgn = '-';
		}
	}

	v = val;

    buf = dst;	

	do {
		c = (uint8_t)(v % (uint8_t)radix);
		if (c >= 10) c += 7;
		c += '0';
		*buf++ = c;
		v /= (uint8_t)radix;
	} while (v);

	if (sgn) *buf++ = sgn;

    if(ndig){
        if (ndig < 0) {
            ndig = -ndig;
            pad = '0';
        }else{
            pad = ' ';
        }

        ndig = ndig - (buf - dst);
        // add padding
        do{
		    *buf++ = pad;
	    }while (--ndig);
    }   

	ndig = buf - dst;
    // start from last written digit
    buf--;
    // swap digits
	do{ 
        c = *dst;
		*dst = *buf;
        *buf = c;
	}while((++dst) < (--buf));

	return ndig;
}

#if LOG_PRINT_FLOAT
/**
 * @brief Convert float to string
 *
 * https://en.wikipedia.org/wiki/Single-precision_floating-point_format
 * https://wirejungle.wordpress.com/2011/08/06/displaying-floating-point-numbers
 *
 * \param dst 		: pointer to destination buffer
 * \param f			: value to be converted
 * \param places	: number of decimal places
 * \return 			: number of digits written to dst
 * */
static uint32_t xpftoa(char *dst, float f, uint8_t places){
	int32_t int_part, frac_part;
	uint8_t prec;
	char *s = dst;

	int_part = (long)(f);

	if (places > FLOAT_MAX_PRECISION)
		places = FLOAT_MAX_PRECISION;

	frac_part = 0;
	prec = 0;

	while ((prec++) < places) {
		f *= 10;
		frac_part = (frac_part * 10) + (long)f - ((long)f / 10) * 10;  //((long)f%10);			
	}

    frac_part = (frac_part < 0)? -frac_part : frac_part;

	dst += xpitoa(dst, int_part, -10, 0);
	*dst++ = '.';
	dst += xpitoa(dst, frac_part, 10, -places);

	return (dst - s);
}
#endif

/**
 * @brief 
 * 
 * @param tag 
 * @param fmt 
 * @param ... 
 */
void logger(const char *tag, int level, const char *fmt, ...){
	char *p, *dst;
    int d, r, w, s, l;
#if LOG_PRINT_FLOAT
	int f;
#endif

    uint8_t new_line;
    va_list arp;

    // lock();
    new_line = next_line(&idx_line);

	va_start(arp, fmt);

	dst = (char*)&log_buffer[new_line][0];

#ifdef LOG_USE_TAG_COLOR
    for(uint8_t i = 0; i < 5; i++){
        *dst++ = log_colors[level][i];
    }
#endif

    while(*tag){
        *dst++ = *tag++;
    }

#ifdef LOG_USE_TAG_COLOR
    for(uint8_t i = 0; i < 5; i++){
        *dst++ = log_colors[LOG_TEXT][i];
    }
#endif

	*dst++ = ' ';

	while ((d = *fmt++) != '\0') {

		if (d != '%') {
			*(dst++) = d; 
			continue;
		}

		d = *fmt++;		
			
		w = r = s = l = 0;
#if LOG_PRINT_FLOAT
		f = 0;
		if (d == '.') {
			d = *fmt++; f = 1;
		}
#endif
		if (d == '0') {
			d = *fmt++; s = 1;
		}

		while ((d >= '0') && (d <= '9')) {
			w = (w * 10) + (d - '0');
			d = *fmt++;
		}

		if (d == 'l') {
			l = 1;
			d = *fmt++;
		}

		if (d == '\0'){
			break;
		}
		
		if (d == 's') {
			p = va_arg(arp, char*);
			while(*p){
				*(dst++) = *(p++);
			}
			continue;
		}

		if (d == 'c') {
			*(dst++) = (char)va_arg(arp, int);
			continue;
		}

		if (d == 'u') r = 10;
		if (d == 'd') r = -10;
		if (d == 'X' || d == 'x') r = 16;
		if (d == 'b') r = 2;
#if LOG_PRINT_FLOAT
		if (d == 'f') {
			if (!f)
				w = FLOAT_MAX_PRECISION;
			dst += xpftoa(dst, va_arg(arp, double), w);
			continue;
		}
#endif		
		if (r == 0){ // break if invalid specifier
			break;	
		} 

		if (s) w = -w;

		if (l) {
			dst += xpitoa(dst, (long)va_arg(arp, long), r, w);
		}
		else {
			if (r > 0)
				dst += xpitoa(dst, (unsigned long)va_arg(arp, int), r, w);
			else
				dst += xpitoa(dst, (long)va_arg(arp, int), r, w);
		}
	}

	*dst = '\0';
    va_end(arp);
    
    //unlock();
}

void log_flush(int (*ps)(const char*)){
    while(print_line != idx_line){
        ps((const char*)log_buffer[print_line]);
        print_line = (print_line + 1) % LOG_MAX_LINES;
    }
}