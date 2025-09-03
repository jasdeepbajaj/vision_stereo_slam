#pragma once
#ifndef VSLAM_EXCEPTION_H
#define VSLAM_EXCEPTION_H

#include <exception>

namespace vslam {
    /**
     * @class Exception
     * @brief Custom exception class for the vSLAM framework.
     * Inherits from `std::exception` and allows storing and returning a custom error message.
     */
    class SLAMException : public std::exception {
        public:
            /**
             * @brief Constructor with error message.
             * @param message A C-style string representing the error message.
             */
            SLAMException(const char* msg) : errMsg(msg) {} 
            
            /**
             * @brief Override of the standard what() function.
             * @return A C-style string describing the error.
             */
            virtual const char* what() const noexcept override {
                return errMsg;
            }
        
        private:
            /// Pointer to a C-style error message string.
            const char* errMsg;
    };

}  // namespace vslam

#endif  // VSLAM_EXCEPTION_H
