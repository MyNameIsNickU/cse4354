/*
 * utilities.h
 *
 *  Created on: Sep 1, 2022
 *      Author: insti
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#include <stdint.h>
#include <stdarg.h>

void emb_vprintf(const char * str, va_list vaArgP);
void emb_printf(const char * str, ...);

uint8_t emb_log2(uint64_t value);
uint32_t emb_pow2(int8_t value);

#endif /* UTILITIES_H_ */
