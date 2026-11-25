#ifndef ADC_H
#define ADC_H

#include <stdint.h>

void     adc_init(void);
uint16_t adc_soma(void);     /* soma das ultimas 32 amostras (0..32736) */
uint16_t adc_ultima(void);   /* ultima amostra bruta (0..1023)          */
uint8_t  adc_pronto(void);   /* 1 apos a primeira conversao             */

#endif
