#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Sofa.Type" for configuration "MinSizeRel"
set_property(TARGET Sofa.Type APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Sofa.Type PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/Sofa.Type.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/Sofa.Type.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS Sofa.Type )
list(APPEND _IMPORT_CHECK_FILES_FOR_Sofa.Type "${_IMPORT_PREFIX}/lib/Sofa.Type.lib" "${_IMPORT_PREFIX}/bin/Sofa.Type.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
