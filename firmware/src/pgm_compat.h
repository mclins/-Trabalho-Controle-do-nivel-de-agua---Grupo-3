/*
 * pgm_compat.h - Permite compilar os modulos independentes de hardware
 * tanto no AVR (strings na memoria de programa) quanto no PC (testes).
 */
#ifndef PGM_COMPAT_H
#define PGM_COMPAT_H

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#include <string.h>
#define PROGMEM
#define PSTR(s)                 (s)
#define pgm_read_byte(p)        (*(const unsigned char *)(p))
#define pgm_read_word(p)        (*(const unsigned short *)(p))
#define pgm_read_ptr(p)         (*(void * const *)(p))
#define strcmp_P                strcmp
#endif

#endif /* PGM_COMPAT_H */
