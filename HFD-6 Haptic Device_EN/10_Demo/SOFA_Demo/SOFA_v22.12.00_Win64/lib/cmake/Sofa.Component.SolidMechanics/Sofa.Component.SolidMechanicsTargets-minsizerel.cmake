#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Sofa.Component.SolidMechanics" for configuration "MinSizeRel"
set_property(TARGET Sofa.Component.SolidMechanics APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Sofa.Component.SolidMechanics PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/Sofa.Component.SolidMechanics.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/Sofa.Component.SolidMechanics.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS Sofa.Component.SolidMechanics )
list(APPEND _IMPORT_CHECK_FILES_FOR_Sofa.Component.SolidMechanics "${_IMPORT_PREFIX}/lib/Sofa.Component.SolidMechanics.lib" "${_IMPORT_PREFIX}/bin/Sofa.Component.SolidMechanics.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
