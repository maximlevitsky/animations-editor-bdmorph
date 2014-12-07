#ifndef CHOLMOD_COMMON_H
#define CHOLMOD_COMMON_H

#include <cholmod.h>

cholmod_common* cholmod_get_common();
void cholmod_finalize();
void cholmod_initialize();

#endif
