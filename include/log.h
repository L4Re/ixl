/*****************************************************************************
 *                                                                           *
 *          Log - A collection of Ixylon-specific logging funcions.          *
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

#undef assert
#define assert(expr) (void) (expr)

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

#define check_err(expr, op) ({\
    int64_t result = (int64_t) (expr);\
    if ((int64_t) result == -1LL) {\
        int err = errno;\
        char buf[512];\
        strerror_r(err, buf, sizeof(buf));\
        fprintf(stderr, "[ERROR] %s:%d %s(): Failed to %s: %s\n", __FILE__, __LINE__, __func__, op, buf);\
        exit(err);\
    }\
    result;\
})

void ixl_hexdump(void* void_ptr, size_t len);
