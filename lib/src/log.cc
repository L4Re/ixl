/*****************************************************************************
 *
 * Implementation of the Ixylon logging facility. The hexdump function is
 * a slightly adapted version of the original version of the ixy driver.
 *
 * Copyright (c): 2023 Till Miemietz <till.miemietz@barkhauseninstitut.org>
 */


/****************************************************************************
 *                                                                          *
 *                           include statements                             *
 *                                                                          *
 ****************************************************************************/


#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#include <l4/ixylon/log.h>

/****************************************************************************
 *                                                                          *
 *                        function implementation                           *
 *                                                                          *
 ****************************************************************************/


void ixl_hexdump(void* void_ptr, size_t len) {
    uint8_t* ptr = (uint8_t*) void_ptr;
    char ascii[17];
    for (uint32_t i = 0; i < len; i += 16) {
        fprintf(stderr, "%06x: ", i);
        int j = 0;
        for (; j < 16 && i + j < len; j++) {
            fprintf(stderr, "%02x", ptr[i + j]);
            if (j % 2) {
                fprintf(stderr, " ");
            }
            ascii[j] = isprint(ptr[i + j]) ? ptr[i + j] : '.';
        }
        ascii[j] = '\0';
        if (j < 16) {
            for (; j < 16; j++) {
                fprintf(stderr, "  ");
                if (j % 2) {
                    fprintf(stderr, " ");
                }
            }
        }
        fprintf(stderr, "  %s\n", ascii);
    }
}
