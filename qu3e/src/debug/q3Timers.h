//--------------------------------------------------------------------------------------------------
/**
@file    q3Timers.h

@author  Ondřej Janošík
@date    13/4/2016

    Copyright (c) 2016 Ondřej Janošík

    This software is provided 'as-is', without any express or implied
    warranty. In no event will the authors be held liable for any damages
    arising from the use of this software.

    Permission is granted to anyone to use this software for any purpose,
    including commercial applications, and to alter it and redistribute it
    freely, subject to the following restrictions:
      1. The origin of this software must not be misrepresented; you must not
         claim that you wrote the original software. If you use this software
         in a product, an acknowledgment in the product documentation would be
         appreciated but is not required.
      2. Altered source versions must be plainly marked as such, and must not
         be misrepresented as being the original software.
      3. This notice may not be removed or altered from any source distribution.
*/
//--------------------------------------------------------------------------------------------------

#ifndef Q3_TIMERS
#define Q3_TIMERS

#include <ctime>
#include <map>

#ifdef TIMERS_ENABLED
    #define q3TimerClear(name) q3Timers::clearTimer(name)
    #define q3TimerStart(name) q3Timers::startTimer(name)
    #define q3TimerStop(name) q3Timers::stopTimer(name)
    #define q3TimerPause(name) q3Timers::pauseTimer(name)
    #define q3TimerPrint(name, text) q3Timers::printTimer(name, text)
#else
    #define q3TimerClear(name)
    #define q3TimerStart(name)
    #define q3TimerStop(name)
    #define q3TimerPause(name)
    #define q3TimerPrint(name, text)
#endif

using std::string;
using std::map;
class q3Timers {

    static map<string, struct timespec> starts;
    static map<string, struct timespec> timers;

public:

    inline static void clearTimer(string name) {
        timers.erase(name);
    }

    inline static void startTimer(string name) {
        struct timespec now;

        clock_gettime(CLOCK_MONOTONIC, &now);
        starts[name] = now;
    }

    inline static void stopTimer(string name) {
        struct timespec start, now, diff;

        clock_gettime(CLOCK_MONOTONIC, &now);

        auto it = starts.find(name);
        if(it == starts.end()) {
            diff = {0, 0};
        }
        else {
            start = it->second;

            diff.tv_sec = now.tv_sec - start.tv_sec;
            diff.tv_nsec = now.tv_nsec - start.tv_nsec;

            if(diff.tv_nsec < 0) {
                diff.tv_sec -= 1;
                diff.tv_nsec += 1000000000;
            }
        }

        timers[name] = diff;
    }

    inline static void pauseTimer(string name) {
        struct timespec start, now, diff;

        clock_gettime(CLOCK_MONOTONIC, &now);

        auto it = starts.find(name);
        if(it == starts.end()) {
            start = now;
        }
        else {
            start = it->second;
        }

        auto timer = timers.find(name);
        if(timer != timers.end()) {
            diff = timer->second;
        }
        else {
            diff = {0, 0};
        }

        diff.tv_sec += now.tv_sec - start.tv_sec;
        diff.tv_nsec += now.tv_nsec - start.tv_nsec;

        if(diff.tv_nsec < 0) {
            diff.tv_sec -= 1;
            diff.tv_nsec += 1000000000;
        }

        timers[name] = diff;
    }

    inline static void printTimer(string name, string text) {
        struct timespec t;
        double ms;
        auto it = timers.find(name);
        if(it == starts.end()) {
            t = {0, 0};
        }
        else {
            t = it->second;
        }

        // Miliseconds
        ms = ((t.tv_sec * 1e3f) + (t.tv_nsec / 1e6f));

        printf("%s: %.4fms\n", text.c_str(), ms);
    }

};


#endif // Q3_TIMERS
