#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Sofa.Geometry" for configuration "MinSizeRel"
set_property(TARGET Sofa.Geometry APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Sofa.Geometry PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/Sofa.Geometry.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/Sofa.Geometry.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS Sofa.Geometry )
list(APPEND _IMPORT_CHECK_FILES_FOR_Sofa.Geometry "${_IMPORT_PREFIX}/lib/Sofa.Geometry.lib" "${_IMPORT_PREFIX}/bin/Sofa.Geometry.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
