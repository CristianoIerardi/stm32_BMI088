/*
 * utils.c
 *
 *  Created on: Apr 9, 2025
 *      Author: crist
 */


#include <math.h>
#include <string.h>
#include <stdint.h>

// Inverte una stringa (utile per ultoa)
static void reverse(char *str, int len) {
    int i = 0, j = len - 1;
    while (i < j) {
        char tmp = str[i];
        str[i++] = str[j];
        str[j--] = tmp;
    }
}

// Converte un intero unsigned in stringa (base 10)
void ultoa(uint32_t num, char *str, int base) {
    int i = 0;
    if (num == 0) {
        str[i] = '0';
        str[i+1] = '\0';
        return;
    }
    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num /= base;
    }
    str[i] = '\0';
    reverse(str, i);
}

// Converte un float in stringa con `afterpoint` cifre decimali
void ftoa(float n, char *res, int afterpoint) {
    int i = 0;

    if (n < 0) {
        res[i++] = '-';
        n = -n;
    }

    int ipart = (int)n;
    float fpart = n - (float)ipart;

    char int_str[16];
    ultoa(ipart, int_str, 10);

    int int_len = strlen(int_str);
    memcpy(res + i, int_str, int_len);
    i += int_len;

    res[i++] = '.';

    fpart *= powf(10, afterpoint);
    ultoa((int)(fpart + 0.5f), int_str, 10);
    strcpy(res + i, int_str);
}
