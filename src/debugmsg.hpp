/*
 * Copyright (c) 2017 TOYOTA MOTOR CORPORATION
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __DEBUG_MSG_H__
#define __DEBUG_MSG_H__

#include <time.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <sys/time.h>

namespace carla {

enum LOG_LEVEL{
    LOG_LEVEL_NONE = 0,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARNING,
    LOG_LEVEL_NOTICE,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
    LOG_LEVEL_MAX = LOG_LEVEL_DEBUG
};

#define LOG_PREFIX	"carla-service"

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define DBG_ERROR(prefix, args,...) _DBG_LOG(LOG_LEVEL_ERROR, __FILENAME__, __FUNCTION__, __LINE__, prefix, args, ##__VA_ARGS__)
#define DBG_WARNING(prefix, args,...) _DBG_LOG(LOG_LEVEL_WARNING, __FILENAME__, __FUNCTION__,__LINE__, prefix, args,##__VA_ARGS__)
#define DBG_NOTICE(prefix, args,...) _DBG_LOG(LOG_LEVEL_NOTICE, __FILENAME__, __FUNCTION__,__LINE__, prefix, args,##__VA_ARGS__)
#define DBG_INFO(prefix, args,...)  _DBG_LOG(LOG_LEVEL_INFO, __FILENAME__, __FUNCTION__,__LINE__, prefix, args,##__VA_ARGS__)
#define DBG_DEBUG(prefix, args,...) _DBG_LOG(LOG_LEVEL_DEBUG, __FILENAME__, __FUNCTION__,__LINE__, prefix, args,##__VA_ARGS__)

static char ERROR_FLAG[6][20] = {"NONE", "ERROR", "WARNING", "NOTICE", "INFO", "DEBUG"};

static void _DBG_LOG(enum LOG_LEVEL level, const char* file, const char* func, const int line, const char* prefix, const char* log, ...)
{
    const int log_level = (getenv("USE_HMI_DEBUG") == NULL)?LOG_LEVEL_DEBUG:atoi(getenv("USE_HMI_DEBUG"));
    if(log_level < level)
    {
        return;
    }

    char *message;
    struct timespec tp;
    unsigned int time;

    clock_gettime(CLOCK_REALTIME, &tp);
	time = (tp.tv_sec * 1000000L) + (tp.tv_nsec / 1000);
    time = time;

    struct timeval timepoint;
    gettimeofday(&timepoint,NULL); 

	va_list args;
	va_start(args, log);
	if (log == NULL || vasprintf(&message, log, args) < 0)
        message = NULL;
    // fprintf(stderr,  "[%10.3f] [%ld.%ld] [%s %s] [%s, %s(), Line:%d] >>> %s \n", time / 1000.0, timepoint.tv_sec, timepoint.tv_usec, prefix, ERROR_FLAG[level], file, func, line, message);
    fprintf(stderr,  "[%ld.%ld] [%s %s] [%s, %s(), Line:%d] >>> %s \n", timepoint.tv_sec, timepoint.tv_usec, prefix, ERROR_FLAG[level], file, func, line, message);
    va_end(args);
	free(message);
}

} // namespace carla

#endif  //__DEBUG_MSG_H__
