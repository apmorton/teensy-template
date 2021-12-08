#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>

bool wait_to_exec(unsigned long *last_exec_ms, unsigned long interval_ms);

#endif /* UTILS_H */