/** @file dbg.h
 *  @brief Trivial print implementation.
 *
 *  @author Szymon (SP)
 *  @bug No known bugs.
 *  @todo Finished.
 */

#ifndef DBG_H
#define DBG_H

#include <stdarg.h>
#include <stdio.h>

#define LOG         debug_print
#define TRACE       { debug_print("%s %d\r\n", __FILE__, __LINE__); delay(100); }

void debug_print(const char *format, ...);

#endif /* DBG_H */