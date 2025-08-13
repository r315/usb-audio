
#if defined(DEBUG) || defined(ENABLE_DEBUG)
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include "debug.h"

#define LINE_NCOLS 0x8		// Number of columens per line

uint32_t strformater(char *dst, const char* fmt, va_list arp);

static int dummy_write(const char *str, int len){ return len; }
static int dummy_read(char *str, int len){ return 0; }
static int dummy_available(void) { return 0; }

static const stdinout_t dummy_out = {
    .available = dummy_available,
	.read = dummy_read,
	.write = dummy_write,
};

static const stdinout_t *sto = &dummy_out;

void dbg_init(const stdinout_t *stdo)
{
	if(stdo != NULL){
		sto = stdo;
	}
}

void dbg_HexDumpLine(const uint8_t *mem, uint32_t len, uint8_t print_ascii)
{
    int i;

    for(i=0; i<len; i++){
		dbg_printf("%02X ",*(mem + i));
	}

    if(print_ascii){
        for(i=0;i<len;i++){
            if(*mem > (' '-1) && *mem < 0x7F)
			dbg_printf("%c", *mem);
            else{
                dbg_putchar('.');
            }
            mem++;
        }
	}

	dbg_putchar('\n');
}


void dbg_HexDump(const uint8_t *mem, uint32_t len)
{
    //dbg_printf("\nDump address: 0x%X \n\n",(uint32_t)&mem[0]);
	for(int i=0; i<len ;i+=LINE_NCOLS){
		dbg_printf("%02X: ",i);
		dbg_HexDumpLine(mem, LINE_NCOLS, 1);
		mem += LINE_NCOLS;
	}
}

void dbg_putchar(char c)
{
    sto->write((const char*)&c, 1);
}

int dbg_println(const char *str)
{
    int len = 0;
    char dbg_out[DBG_PRINT_MAX_LEN];

    while(*str){
        dbg_out[len++] = *str++;
    }

    dbg_out[len++] ='\n';
	return sto->write(dbg_out, len);
}

int dbg_printf(const char* fmt, ...)
{
	char dbg_out[DBG_PRINT_MAX_LEN];
	va_list arp;
	va_start(arp, fmt);
	int len = strformater(dbg_out, fmt, arp);
	va_end(arp);
	return sto->write(dbg_out, len);
}

/**
 * @brief Convert 32-bit integer number to string
 *
 * \param dst 	:	pointer to destination buffer
 * \param val	:	value to be converted
 * \param radix	:	base of convertion [-10, 10, 16]
 * \param ndig 	:	minimum number of digits, ndig > 0 pad with ' ', ndig < 0 pad with '0'
 * \return 		:	number of digits written to dst
 * */
#define XPITOA_BUF_SIZE 10
uint32_t i2ia(char *dst, int32_t val, int radix, int ndig){
	char buf[XPITOA_BUF_SIZE];
	uint8_t i, c, r, sgn = 0, pad = ' ';
	uint32_t v;

	if (radix < 0) {
		radix = -radix;
		if (val < 0) {
			val = -val;
			sgn = '-';
		}
	}

	v = val;
	r = radix;

	if (ndig < 0) {
		ndig = -ndig;
		pad = '0';
	}

	if (ndig > XPITOA_BUF_SIZE) {
		ndig = XPITOA_BUF_SIZE;
	}

	ndig = XPITOA_BUF_SIZE - 1 - ndig;
	i = XPITOA_BUF_SIZE;
	buf[--i] = '\0';

	do {
		c = (uint8_t)(v % r);
		if (c >= 10) c += 7;
		c += '0';
		buf[--i] = c;
		v /= r;
	} while (v);

	if (sgn) buf[--i] = sgn;

	while (i > ndig) {
		buf[--i] = pad;
	}

	ndig = XPITOA_BUF_SIZE - 1 - i;

	while(buf[i]){
		*dst++ = buf[i++];
	}

	*dst = '\0';

	return ndig;
}

/**
 * @brief String formater
 *   %nu, %nd, %nb, %c, %s, %l, %x, %.nf
 *
 * TODO: fix print percent sign (%)
 * */
uint32_t strformater(char *dst, const char* fmt, va_list arp)
{

	int d, r, w, s, l;
#if ENABLE_FLOAT_FMT
    int f;
#endif
	char *p,*a;
	a = dst;

	while ((d = *fmt++) != '\0') {

		if (d != '%') {
			*(dst++) = d;
			continue;
		}

		d = *fmt++;

		w = r = s = l = 0;
#if ENABLE_FLOAT_FMT
        f = 0;

		if (d == '.') {
            d = *fmt++; f = 1;
		}
#endif
		if (d == '0') {
			d = *fmt++; s = 1;
		}

		while ((d >= '0') && (d <= '9')) {
			w = w * 10 + (d - '0');
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
#if ENABLE_FLOAT_FMT
		if (d == 'f') {
			if (!f)
				w = FLOAT_MAX_PRECISION;
			dst += d2da(dst, va_arg(arp, double), w); // Note: float is promoted to double for varargs
			continue;
		}
#endif
		if (r == 0){
			break;
		}

		if (s) w = -w;

		if (l) {
			dst += i2ia(dst, (long)va_arg(arp, long), r, w);
		}
		else {
			if (r > 0)
				dst += i2ia(dst, (unsigned long)va_arg(arp, int), r, w);
			else
				dst += i2ia(dst, (long)va_arg(arp, int), r, w);
		}
	}

	*dst = '\0';
	return dst - a;
}
#endif