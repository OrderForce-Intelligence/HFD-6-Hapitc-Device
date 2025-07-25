#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Sofa.Component.MechanicalLoad" for configuration "MinSizeRel"
set_property(TARGET Sofa.Component.MechanicalLoad APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Sofa.Component.MechanicalLoad PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/Sofa.Component.MechanicalLoad.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/Sofa.Component.MechanicalLoad.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS Sofa.Component.MechanicalLoad )
list(APPEND _IMPORT_CHECK_FILES_FOR_Sofa.Component.MechanicalLoad "${_IMPORT_PREFIX}/lib/Sofa.Component.MechanicalLoad.lib" "${_IMPORT_PREFIX}/bin/Sofa.Component.MechanicalLoad.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
