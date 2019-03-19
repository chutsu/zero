#ifndef LOG_H
#define LOG_H

#include <stdio.h>

/* DEBUG */
#ifdef NDEBUG
  #define debug(M, ...)
#else
  #define debug(M, ...) \
      fprintf(stderr, \
          "[DEBUG] %s:%d: " M "\n", \
          __func__, \
          __LINE__, \
          ##__VA_ARGS__ \
      )
#endif

/* LOG */
#define log_err(M, ...) \
    fprintf(stderr,\
        "[ERROR] [%s] " M "\n",\
        __func__,\
        ##__VA_ARGS__\
    )
#define log_warn(M, ...) \
    fprintf(stderr, "[WARN] " M "\n", ##__VA_ARGS__)
#define log_info(M, ...) \
    fprintf(stderr, "[INFO] " M "\n", ##__VA_ARGS__)

/* CHECK */
#define check(A, M, ...) \
    if (!(A)) { \
        log_err(M, ##__VA_ARGS__); \
        goto error; \
    }

#endif  // LOG_H
