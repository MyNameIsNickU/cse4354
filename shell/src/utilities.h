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

#endif /* UTILITIES_H_ */
