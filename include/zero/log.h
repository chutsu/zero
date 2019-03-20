#ifndef LOG_H
#define LOG_H

#include <stdio.h>

/* DEBUG */
#ifdef NDEBUG
  #define DEBUG(M, ...)
#else
  #define DEBUG(M, ...) \
      fprintf(stderr, \
          "[DEBUG] %s:%d: " M "\n", \
          __func__, \
          __LINE__, \
          ##__VA_ARGS__ \
      )
#endif

/* LOG */
#define LOG_ERROR(M, ...) \
    fprintf(stderr,\
        "[ERROR] [%s] " M "\n",\
        __func__,\
        ##__VA_ARGS__\
    )
#define LOG_WARN(M, ...) \
    fprintf(stderr, "[WARN] " M "\n", ##__VA_ARGS__)
#define LOG_INFO(M, ...) \
    fprintf(stderr, "[INFO] " M "\n", ##__VA_ARGS__)

/* CHECK */
#define CHECK(A, M, ...) \
    if (!(A)) { \
        log_err(M, ##__VA_ARGS__); \
        goto error; \
    }

#endif  // LOG_H
