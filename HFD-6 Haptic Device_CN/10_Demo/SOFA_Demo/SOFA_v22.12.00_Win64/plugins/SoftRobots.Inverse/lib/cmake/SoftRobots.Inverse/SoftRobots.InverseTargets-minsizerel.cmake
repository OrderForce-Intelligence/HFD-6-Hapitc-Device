#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SoftRobots.Inverse" for configuration "MinSizeRel"
set_property(TARGET SoftRobots.Inverse APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SoftRobots.Inverse PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SoftRobots.Inverse.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SoftRobots.Inverse.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SoftRobots.Inverse )
list(APPEND _IMPORT_CHECK_FILES_FOR_SoftRobots.Inverse "${_IMPORT_PREFIX}/lib/SoftRobots.Inverse.lib" "${_IMPORT_PREFIX}/bin/SoftRobots.Inverse.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
