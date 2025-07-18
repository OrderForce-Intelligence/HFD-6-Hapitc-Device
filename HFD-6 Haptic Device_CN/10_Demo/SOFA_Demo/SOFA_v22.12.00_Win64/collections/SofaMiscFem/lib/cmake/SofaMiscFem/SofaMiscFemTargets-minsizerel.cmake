#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SofaMiscFem" for configuration "MinSizeRel"
set_property(TARGET SofaMiscFem APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SofaMiscFem PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SofaMiscFem.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SofaMiscFem.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SofaMiscFem )
list(APPEND _IMPORT_CHECK_FILES_FOR_SofaMiscFem "${_IMPORT_PREFIX}/lib/SofaMiscFem.lib" "${_IMPORT_PREFIX}/bin/SofaMiscFem.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
