#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SofaSimpleGUI" for configuration "MinSizeRel"
set_property(TARGET SofaSimpleGUI APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SofaSimpleGUI PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SofaSimpleGUI.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SofaSimpleGUI.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SofaSimpleGUI )
list(APPEND _IMPORT_CHECK_FILES_FOR_SofaSimpleGUI "${_IMPORT_PREFIX}/lib/SofaSimpleGUI.lib" "${_IMPORT_PREFIX}/bin/SofaSimpleGUI.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
