#ifndef _logger_h_
#define _logger_h_

#define LOG_USE_TAG_COLOR   0
#define LOG_PRINT_FLOAT     0

#ifdef LOG_USE_TAG_COLOR
#define LOG_USE_COLOR
enum { LOG_TEXT = 0, LOG_DEBUG, LOG_INFO, LOG_WARN, LOG_ERROR, LOG_FATAL };
#define log_dbg(...) logger("[DEBUG]", LOG_DEBUG, __VA_ARGS__)
#define log_inf(...) logger("[INFO]", LOG_INFO, __VA_ARGS__)
#define log_war(...) logger("[WARNING]", LOG_WARN, __VA_ARGS__)
#define log_err(...) logger("[ERROR]", LOG_ERROR, __VA_ARGS__)
#define log_fatal(...) logger("[FATAL]", LOG_FATAL, __VA_ARGS__)
#define log_tag(_TAG_, _LEVEL_, ...) logger(_TAG_, _LEVEL_, __VA_ARGS__)
#else
#define log_tag(_TAG_, ...) logger(_TAG_" ", LOG_TEXT, __VA_ARGS__)
#endif

#define log(...) logger("", LOG_TEXT, __VA_ARGS__)

void logger(const char* tag, int level, const char *fmt, ...);
void log_flush(int (*)(const char*));
#endif