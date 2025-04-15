/*
 * utils.h
 *
 *  Created on: Apr 9, 2025
 *      Author: crist
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>

void ultoa(uint32_t num, char *str, int base);
void ftoa(float n, char *res, int afterpoint);

#endif


#endif /* INC_UTILS_H_ */
