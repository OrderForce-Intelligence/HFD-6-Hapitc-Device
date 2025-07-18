#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Sofa.Framework" for configuration "MinSizeRel"
set_property(TARGET Sofa.Framework APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Sofa.Framework PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/Sofa.Framework.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/Sofa.Framework.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS Sofa.Framework )
list(APPEND _IMPORT_CHECK_FILES_FOR_Sofa.Framework "${_IMPORT_PREFIX}/lib/Sofa.Framework.lib" "${_IMPORT_PREFIX}/bin/Sofa.Framework.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
