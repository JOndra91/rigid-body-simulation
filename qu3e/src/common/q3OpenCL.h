//--------------------------------------------------------------------------------------------------
/**
@file    q3OpenCL.h

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

#ifndef Q3OPENCL_H
#define Q3OPENCL_H

#include <cassert>

#define ALIGNED __attribute__ ((aligned))
#define PACKED __attribute__ ((packed))

#define CEIL_TO(a, ceil) ((a - 1 + ceil)/ceil) * ceil


#define assert_size(type, size) assert(sizeof(type) == size)
// #define assert_size(type, size) do { std::cout << "sizeof(" << # type << ") = " << sizeof(type) << std::endl; assert(sizeof(type) == size); } while(0)

#define CHECK_CL_ERROR(err, info) do { \
        if(err) { \
            std::cerr << "OpenCL error: \"" info "\" (" \
                << getCLErrorString(err) << ")" << std::endl; \
        } \
    } while(0)

enum class q3OpenCLDevice {
        NONE,
#ifdef WITH_OCL
        CPU,
        GPU,
#endif // WITH_OCL
};

inline bool q3IsOpenCLAccelerated(q3OpenCLDevice dev) {
    return dev != q3OpenCLDevice::NONE;
}

#endif // Q3OPENCL_H
