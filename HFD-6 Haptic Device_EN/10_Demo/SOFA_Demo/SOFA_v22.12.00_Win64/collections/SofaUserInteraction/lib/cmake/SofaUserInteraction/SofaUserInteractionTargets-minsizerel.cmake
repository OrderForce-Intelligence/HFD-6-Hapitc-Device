#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SofaUserInteraction" for configuration "MinSizeRel"
set_property(TARGET SofaUserInteraction APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SofaUserInteraction PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SofaUserInteraction.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SofaUserInteraction.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SofaUserInteraction )
list(APPEND _IMPORT_CHECK_FILES_FOR_SofaUserInteraction "${_IMPORT_PREFIX}/lib/SofaUserInteraction.lib" "${_IMPORT_PREFIX}/bin/SofaUserInteraction.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
