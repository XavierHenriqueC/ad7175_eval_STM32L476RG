#include "uart_printf.h"
#include "main.h" /* for huart2 extern */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

extern UART_HandleTypeDef huart2;

/* single place to format and transmit debug messages */
void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;

    /* ensure CRLF termination if not present */
    if (n >= 2 && buf[n-2] == '\r' && buf[n-1] == '\n') {
        /* already CRLF */
    } else if (n >= 1 && buf[n-1] == '\n') {
        /* convert LF to CRLF */
        if (n + 1 < (int)sizeof(buf)) {
            buf[n-1] = '\r';
            buf[n] = '\n';
            buf[n+1] = '\0';
            n += 1;
        }
    } else {
        if (n + 2 < (int)sizeof(buf)) {
            buf[n++] = '\r';
            buf[n++] = '\n';
            buf[n] = '\0';
        }
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}
