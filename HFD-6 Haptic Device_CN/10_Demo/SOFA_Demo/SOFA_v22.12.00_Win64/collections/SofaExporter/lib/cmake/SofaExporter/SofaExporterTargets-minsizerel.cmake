#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SofaExporter" for configuration "MinSizeRel"
set_property(TARGET SofaExporter APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SofaExporter PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SofaExporter.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SofaExporter.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SofaExporter )
list(APPEND _IMPORT_CHECK_FILES_FOR_SofaExporter "${_IMPORT_PREFIX}/lib/SofaExporter.lib" "${_IMPORT_PREFIX}/bin/SofaExporter.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
