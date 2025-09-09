/*****************************************************************************
 *                                                                           *
 *            Log - A collection of Ixl-specific logging funcions.           *
 *                                                                           *
 * Some parts of this header still originate from the Ixy project, while the *
 * majority has been rewritten in C++ and was adapted to L4Re.               *
 *                                                                           *
 *****************************************************************************/

#pragma once

#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#ifndef IXL_NDEBUG

#define ixl_debug(fmt, ...) do {\
    fprintf(stderr, "[DEBUG] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define ixl_info(fmt, ...) do {\
    fprintf(stdout, "[INFO ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define ixl_warn(fmt, ...) do {\
    fprintf(stderr, "[WARN ] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
} while(0)

#define ixl_error(fmt, ...) do {\
    fprintf(stderr, "[ERROR] %s:%d %s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__);\
    abort();\
} while(0)

#else

#define ixl_debug(fmt, ...) do {} while(0)

#define ixl_info(fmt, ...) do {\
    fprintf(stdout, "[INFO ] " fmt "\n", ##__VA_ARGS__);\
} while(0)

#define ixl_warn(fmt, ...) do {\
    fprintf(stderr, "[WARN ] " fmt "\n", ##__VA_ARGS__);\
} while(0)

#define ixl_error(fmt, ...) do {\
    fprintf(stderr, "[ERROR] " fmt "\n", ##__VA_ARGS__);\
} while(0)

#endif

void ixl_hexdump(const void *void_ptr, size_t len);
