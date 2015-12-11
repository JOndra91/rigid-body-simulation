#pragma once

#include <string>
#include <GL/glew.h>

namespace gmu {

    using std::string;

    class Exception {
    protected:
        int errorCode;
        string message;
    public:

        Exception() : Exception(0, "Exception") {
        }

        Exception(string message) : Exception(0, message) {
        }

        Exception(int errorCode) : Exception(errorCode, "Exception") {
        }

        Exception(int errorCode, string message) : errorCode(errorCode), message(message) {
        }

        virtual ~Exception() {
        }

        inline int getCode() {
            return errorCode;
        }

        inline string getMessage() {
            return message;
        }
    };

    class GLException : public Exception {
    public:

        GLException(string message) : Exception(message) {
        }
    };

}