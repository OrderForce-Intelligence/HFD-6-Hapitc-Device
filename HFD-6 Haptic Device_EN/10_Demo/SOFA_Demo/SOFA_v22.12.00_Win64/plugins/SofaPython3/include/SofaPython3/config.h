/******************************************************************************
*                              SofaPython3 plugin                             *
*                  (c) 2021 CNRS, University of Lille, INRIA                  *
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
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#pragma once

#include <sofa/config.h>
#include <pybind11/pybind11.h>


/**
 * This macro HAS to be placed in front of function prototypes containing
 * pybind11 symbols. By default pybind exports its attributes as hidden symbols
 * which causes linking against libs using them impossible (undefined refs).
 * It will also do the job of adding the dllexport / dllimport declaration on
 * Windows Systems.
 **/

 // __attribute__(visibility("default")) && __declspec(dllexport)
#ifdef SOFA_BUILD_SOFAPYTHON3
	#define SOFAPYTHON3_API PYBIND11_EXPORT
#else
	#define SOFAPYTHON3_API SOFA_IMPORT_DYNAMIC_LIBRARY
#endif

// define own pybind version macro, much easier to test/read
#define PYBIND11_SOFA_VERSION_MAJOR 2
#define PYBIND11_SOFA_VERSION_MINOR 6
#define PYBIND11_SOFA_VERSION_PATCH 2
#define PYBIND11_SOFA_VERSION ( \
      PYBIND11_SOFA_VERSION_MAJOR * 10000 \
    + PYBIND11_SOFA_VERSION_MINOR * 100 \
    + PYBIND11_SOFA_VERSION_PATCH )

// pybind11 already bind the attributeError starting from 2.8.1 version.
// so if the version is >= 2.8.1, macro does nothing, otherwise bind attribute_error
#if PYBIND11_SOFA_VERSION >= 20801
#define SOFAPYTHON3_BIND_ATTRIBUTE_ERROR()
#else
#define SOFAPYTHON3_BIND_ATTRIBUTE_ERROR() namespace pybind11 { PYBIND11_RUNTIME_EXCEPTION(attribute_error, PyExc_AttributeError) }
#endif // PYBIND11_SOFA_VERSION >= 20801

#if PYBIND11_SOFA_VERSION >= 20600
#define SOFAPYTHON3_ADD_PYBIND_TYPE_FOR_OLD_VERSION()
#else
#define SOFAPYTHON3_ADD_PYBIND_TYPE_FOR_OLD_VERSION() namespace pybind11 { \
class type : public pybind11::object { \
public: \
    PYBIND11_OBJECT(type, pybind11::object, PyType_Check) \
    static pybind11::handle handle_of(pybind11::handle h) { return handle((PyObject*) Py_TYPE(h.ptr())); } \
    static type of(pybind11::handle h) { return type(type::handle_of(h), borrowed_t{}); } \
    template<typename T> static handle handle_of(); \
    template<typename T> static type of() {return type(type::handle_of<T>(), borrowed_t{}); } \
}; \
}
#endif // PYBIND11_SOFA_VERSION >= 20801
