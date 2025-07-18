#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SofaMiscForceField" for configuration "MinSizeRel"
set_property(TARGET SofaMiscForceField APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SofaMiscForceField PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SofaMiscForceField.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SofaMiscForceField.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SofaMiscForceField )
list(APPEND _IMPORT_CHECK_FILES_FOR_SofaMiscForceField "${_IMPORT_PREFIX}/lib/SofaMiscForceField.lib" "${_IMPORT_PREFIX}/bin/SofaMiscForceField.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
