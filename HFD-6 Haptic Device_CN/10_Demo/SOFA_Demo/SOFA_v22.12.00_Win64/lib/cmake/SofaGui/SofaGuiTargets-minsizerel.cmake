#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SofaGui" for configuration "MinSizeRel"
set_property(TARGET SofaGui APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SofaGui PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SofaGui.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SofaGui.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SofaGui )
list(APPEND _IMPORT_CHECK_FILES_FOR_SofaGui "${_IMPORT_PREFIX}/lib/SofaGui.lib" "${_IMPORT_PREFIX}/bin/SofaGui.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
