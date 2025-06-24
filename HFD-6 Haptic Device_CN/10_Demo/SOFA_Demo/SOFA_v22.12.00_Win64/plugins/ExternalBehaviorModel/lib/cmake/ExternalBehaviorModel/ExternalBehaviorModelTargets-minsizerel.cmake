#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ExternalBehaviorModel" for configuration "MinSizeRel"
set_property(TARGET ExternalBehaviorModel APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(ExternalBehaviorModel PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/ExternalBehaviorModel.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/ExternalBehaviorModel.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS ExternalBehaviorModel )
list(APPEND _IMPORT_CHECK_FILES_FOR_ExternalBehaviorModel "${_IMPORT_PREFIX}/lib/ExternalBehaviorModel.lib" "${_IMPORT_PREFIX}/bin/ExternalBehaviorModel.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
