#ifndef PATHTOHEADERFILE_HEADERNAME_H //File path and name in all upper case. For this file: EXVECTRLIBTEMPLATE_YOURHEADERFILESHERE_H
#define PATHTOHEADERFILE_HEADERNAME_H

/**
 * 
 * Header files should be named according to their function with all lower case and _ as spaces between words. Also try to name them from
 * less specific topic to more specific. For a class to convert time from gregorian calender to julian, a recommended name would be:
 *                                      time_convertion_gregtojul.hpp
 * this way finding a header file for inclusion one can find the file easier. You can also place header files into subfolders to sort between 
 * topics like HAL implementation and Graphics processing. In this case, the source files should also be sorted inside subfolders inside src.
 * 
 * Header files only do little to no implementation/definition. They only declare a class's structure or function and important includes 
 * needed to use said class/function.
 * 
 * Everything is done inside the VCTR namespace to seperate ExVectr from other libraries used.
 * 
 * Implementation is done in the source files located inside src folder.
 * 
*/

#include "stdint.h"

namespace VCTR
{

    /**
     * @brief Remember to add documentation to all functions. This one returns the current system time in seconds.
     * @note Will overflow after ~292.47 years. If a deployed system runs longer then that and crashes, then give my great great grandchildren a call.
     *  
     * @returns time since system start in seconds.
     */
    extern float yourSpecialFunctionDefined();

    /**
     * A super duper class than has no actual use other than containing a super duper function to test.
    */
    class SomeSuperDuperClass {
        public:

        /**
         * @brief A super duper function to greet the user.
         * @param systemTime The current system time in seconds since start.
         * 
         * @returns a cool string to greet the user.
        */
        const char* aSuperDuperFunction(float systemTime);

    };

}

#endif