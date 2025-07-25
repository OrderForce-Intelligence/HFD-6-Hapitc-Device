/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <sofa/config/sharedlibrary_defines.h>
#include <sofa/config/build_option_dump_visitor.h>
#include <sofa/config/build_option_threading.h>
#include <sofa/config/build_option_bbox.h>

#include <cstddef> // For nullptr
#include <limits> // std::numeric_limits<>

// fixes CGAL plugin build errors (default value: 5)
#define BOOST_PARAMETER_MAX_ARITY 12

/* #undef SOFA_DETECTIONOUTPUT_FREEMOTION */

/* #undef SOFA_FLOAT */
#define SOFA_DOUBLE

/* #undef SOFA_WITH_FLOAT */
#define SOFA_WITH_DOUBLE

#define SOFA_WITH_DEVTOOLS

#ifdef _MSC_VER
#define EIGEN_DONT_ALIGN
#endif // _MSC_VER

#ifdef WIN32
#define UNICODE
#endif // WIN32

#ifdef SOFA_FLOAT
typedef float SReal;
#else
typedef double SReal;
#endif

/// User-defined literals allowing to convert any floating-point to a SReal
constexpr SReal operator"" _sreal(long double real)
{
    return static_cast<SReal>(real);
}
/// User-defined literals allowing to convert any integer to a SReal
constexpr SReal operator"" _sreal(unsigned long long int integer)
{
    return static_cast<SReal>(integer);
}

#if defined(_WIN32)
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#endif

// snprintf() has been provided since MSVC++ 14 (Visual Studio 2015).  For other
// versions, it is simply #defined to _snprintf().
#if (defined(_MSC_VER) && _MSC_VER < 1900)
#  define snprintf _snprintf
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#  include <windows.h>
#undef WIN32_LEAN_AND_MEAN
#endif

#ifdef BOOST_NO_EXCEPTIONS
#  include<exception>

namespace boost
{
    inline void throw_exception(std::exception const & e)
    {
        return;
    }
}
#endif // BOOST_NO_EXCEPTIONS


#ifdef _MSC_VER
#  ifndef _USE_MATH_DEFINES
#    define _USE_MATH_DEFINES 1 // required to get M_PI from math.h
#  endif
// Visual C++ does not include stdint.h
typedef signed __int8		int8_t;
typedef signed __int16		int16_t;
typedef signed __int32		int32_t;
typedef signed __int64		int64_t;
typedef unsigned __int8		uint8_t;
typedef unsigned __int16	uint16_t;
typedef unsigned __int32	uint32_t;
typedef unsigned __int64	uint64_t;
#else
#  include <stdint.h>
#endif

#define sofa_do_concat2(a,b) a ## b
#define sofa_do_concat(a,b) sofa_do_concat2(a,b)
#define sofa_concat(a,b) sofa_do_concat(a,b)

#define sofa_do_tostring(a) #a
#define sofa_tostring(a) sofa_do_tostring(a)

#if defined(_WIN32)
    #define SOFA_PRAGMA_MESSAGE(text) __pragma(message(__FILE__ "(" sofa_tostring(__LINE__) "): " text))
    #define SOFA_PRAGMA_WARNING(text) __pragma(message(__FILE__ "(" sofa_tostring(__LINE__) "): warning: " text))
    #define SOFA_PRAGMA_ERROR(text) __pragma(message(__FILE__ "(" sofa_tostring(__LINE__) "): error: " text))
#elif defined(__clang__)
    #define SOFA_DO_PRAGMA(x) _Pragma(#x)
    #define SOFA_PRAGMA_MESSAGE(text) SOFA_DO_PRAGMA(message text)
    #define SOFA_PRAGMA_WARNING(text) SOFA_DO_PRAGMA(GCC warning text)
    #define SOFA_PRAGMA_ERROR(text) SOFA_DO_PRAGMA(GCC error text)
#else
    #define SOFA_DO_PRAGMA(x) _Pragma(#x)
    #define SOFA_PRAGMA_MESSAGE(text) SOFA_DO_PRAGMA(message #text)
    #define SOFA_PRAGMA_WARNING(text) SOFA_DO_PRAGMA(GCC warning #text)
    #define SOFA_PRAGMA_ERROR(text) SOFA_DO_PRAGMA(GCC error #text)
#endif

/************* DEPRECATION MACROS *************/

// Empty class to be used to highlight deprecated objects at compilation time.
class DeprecatedAndRemoved {};

#if defined(_WIN32) || defined(__clang__)
    #define SOFA_DEPRECATED_HEADER(deprecateDate, disableDate, newHeader) \
        SOFA_PRAGMA_WARNING( \
            "This header has been DEPRECATED since " deprecateDate ". " \
            "You have until " disableDate " to fix your code. " \
            "To fix this warning you must include " newHeader " instead." )

    #define SOFA_DISABLED_HEADER(deprecateDate, disableDate, newHeader) \
        SOFA_PRAGMA_ERROR( \
            "This header has been DISABLED since " disableDate " " \
            "after being deprecated on " deprecateDate ". " \
            "To fix this error you must include " newHeader " instead. " )

    #define SOFA_DEPRECATED_HEADER_NOT_REPLACED(deprecateDate, disableDate) \
        SOFA_PRAGMA_WARNING( \
            "This header has been DEPRECATED since " deprecateDate ". " \
            "You have until " disableDate " to fix your code. " \
            "It will not be replaced by another one. " )

    #define SOFA_DISABLED_HEADER_NOT_REPLACED(deprecateDate, disableDate) \
        SOFA_PRAGMA_ERROR( \
            "This header has been DISABLED since " disableDate " " \
            "after being deprecated on " deprecateDate ". "\
            "It is not replaced by another one. " \
            "Contact the SOFA Consortium for more information. " )
#else
	#define SOFA_DEPRECATED_HEADER(deprecateDate, disableDate, newHeader) \
        SOFA_PRAGMA_WARNING( \
            This header has been DEPRECATED since deprecateDate. \
            You have until disableDate to fix your code. \
            To fix this warning you must include newHeader instead. )

	#define SOFA_DISABLED_HEADER(deprecateDate, disableDate, newHeader) \
        SOFA_PRAGMA_ERROR( \
            This header has been DISABLED since disableDate \
            after being deprecated on deprecateDate. \
            To fix this error you must include newHeader instead. )

    #define SOFA_DEPRECATED_HEADER_NOT_REPLACED(deprecateDate, disableDate) \
        SOFA_PRAGMA_WARNING( \
            This header has been DEPRECATED since deprecateDate. \
            You have until disableDate to fix your code. \
            It will not be replaced by another one. )

    #define SOFA_DISABLED_HEADER_NOT_REPLACED(deprecateDate, disableDate) \
        SOFA_PRAGMA_ERROR( \
            This header has been DISABLED since disableDate \
            after being deprecated on deprecateDate. \
            It is not replaced by another one. \
            Contact the SOFA Consortium for more information. )
#endif


#define SOFA_ATTRIBUTE_DEPRECATED(deprecateDate, removeDate, toFixMsg) \
    [[deprecated( \
        "It is still usable but has been DEPRECATED since " deprecateDate ". " \
        "You have until " removeDate " to fix your code. " toFixMsg)]]

// The following macro is empty because it is supposed to be used in conjonction
// when an attribute of type DeprecatedAndRemoved. It should not contain a [[deprecated]]
// attribute, because if it does, the "deprecated" warning is showed in every constructor's
// for every disabled attribute.
#define SOFA_ATTRIBUTE_DISABLED(deprecateDate, disableDate, toFixMsg)

#define SOFA_ATTRIBUTE_DISABLED__BASEDATA_OWNERCLASS_ACCESSOR(msg) \
    SOFA_ATTRIBUTE_DISABLED( \
        "v21.06 (PR#1890)", "v22.12 (PR#3279)", \
        "BaseData API has changed. " msg)

#define SOFA_ATTRIBUTE_DISABLED__TDATA_INTO_DATA(msg) \
    SOFA_ATTRIBUTE_DISABLED( \
        "v21.06 (PR#1753)", "v22.12 (PR#3279)", \
        "Data API has changed. " msg)


#define SOFA_ATTRIBUTE_DISABLED__TYPEMEMBER(type) \
    SOFA_ATTRIBUTE_DISABLED( \
        "v22.12 (PR#3357)", "v23.12", \
        "Class's member type alias " sofa_tostring(type) " was not needed.") \
        DeprecatedAndRemoved type

#define SOFA_ATTRIBUTE_REPLACED__TYPEMEMBER(type, newtype) \
    SOFA_ATTRIBUTE_DEPRECATED( \
        "v22.12 (PR#3357)", "v23.12", \
        "This type member " sofa_tostring(type) " is now deprecated. To update your code please replace it with " sofa_tostring(newtype)) \
        typedef newtype type

#define SOFA_ATTRIBUTE_DEPRECATED__SOFAOSTREAM() \
    SOFA_ATTRIBUTE_DEPRECATED( \
        "v21.12 (PR#2292)", "v22.06", \
        "Use the Messaging API instead of using SofaOStream and sout/serr/sendl.")

#define SOFA_ATTRIBUTE_DEPRECATED__VECTOR(msg) \
    SOFA_ATTRIBUTE_DEPRECATED( \
        "v22.12 (PR#3299)", "v23.06", \
        "Long aliases for VectorX will be removed. " msg)

#define SOFA_ATTRIBUTE_DEPRECATED__READX_PARAMS(msg) \
    SOFA_ATTRIBUTE_DEPRECATED( \
        "v22.12 (PR#3304)", "v23.06", \
        "Calling ReadX with SingleLink's argument will be removed." msg)

/**********************************************/

#define SOFA_DECL_CLASS(name) // extern "C" { int sofa_concat(class_,name) = 0; }
#define SOFA_LINK_CLASS(name) // extern "C" { extern int sofa_concat(class_,name); int sofa_concat(link_,name) = sofa_concat(class_,name); }

// Prevent compiler warnings about 'unused variables'.
// This should be used when a parameter name is needed (e.g. for
// documentation purposes) even if it is not used in the code.
#define SOFA_UNUSED(x) (void)(x)

// utility for debug tracing
#ifdef _MSC_VER
    #define SOFA_CLASS_METHOD ( std::string(this->getClassName()) + "::" + __FUNCTION__ + " " )
#else
    #define SOFA_CLASS_METHOD ( std::string(this->getClassName()) + "::" + __func__ + " " )
#endif


// The SOFA_EXTERN_TEMPLATE macro was used to control the use of extern templates in Sofa.
// It has been cleaned out in 41e0ab98bbb6e22b053b24e7bbdd31c8636336c9 "[ALL] Remove SOFA_EXTERN_TEMPLATE".
// Macro definition is kept to avoid breaking all external plugins.
#define SOFA_EXTERN_TEMPLATE

namespace sofa
{

using SignedIndex = int32_t;
using Index = uint32_t;
using Size = uint32_t;

constexpr Index InvalidID = (std::numeric_limits<Index>::max)();

}

