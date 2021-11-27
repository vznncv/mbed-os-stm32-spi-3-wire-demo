#include "app_utils.h"

#include <cstdio>
#include <cstdarg>

void SimpleLogger::set_enabled(bool value)
{
    _enabled = value;
}

bool SimpleLogger::is_enabled() const
{
    return _enabled;
}


void SimpleLogger::log(const char *level_name, const char *msg, ...)
{
    va_list args;
    va_start(args, msg);
    vlog(level_name, msg, args);
    va_end(args);
}

void SimpleLogger::vlog(const char *level_name, const char *msg, va_list args)
{
    if (!_enabled) {
        return;
    }
    printf("[%s] ", level_name);
    vprintf(msg, args);
    printf("\n");
}

void SimpleLogger::info(const char *msg, ...)
{
    va_list args;
    va_start(args, msg);
    vlog("INFO", msg, args);
    va_end(args);
}

void SimpleLogger::error(const char *msg, ...)
{
    va_list args;
    va_start(args, msg);
    vlog("ERROR", msg, args);
    va_end(args);
}

