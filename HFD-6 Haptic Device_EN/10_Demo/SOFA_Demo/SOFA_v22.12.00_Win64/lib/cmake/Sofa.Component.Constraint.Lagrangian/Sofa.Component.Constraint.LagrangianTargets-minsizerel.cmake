#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Sofa.Component.Constraint.Lagrangian" for configuration "MinSizeRel"
set_property(TARGET Sofa.Component.Constraint.Lagrangian APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Sofa.Component.Constraint.Lagrangian PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/Sofa.Component.Constraint.Lagrangian.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/Sofa.Component.Constraint.Lagrangian.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS Sofa.Component.Constraint.Lagrangian )
list(APPEND _IMPORT_CHECK_FILES_FOR_Sofa.Component.Constraint.Lagrangian "${_IMPORT_PREFIX}/lib/Sofa.Component.Constraint.Lagrangian.lib" "${_IMPORT_PREFIX}/bin/Sofa.Component.Constraint.Lagrangian.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
