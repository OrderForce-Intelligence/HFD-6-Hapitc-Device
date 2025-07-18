#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SofaExplicitOdeSolver" for configuration "MinSizeRel"
set_property(TARGET SofaExplicitOdeSolver APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SofaExplicitOdeSolver PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SofaExplicitOdeSolver.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SofaExplicitOdeSolver.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SofaExplicitOdeSolver )
list(APPEND _IMPORT_CHECK_FILES_FOR_SofaExplicitOdeSolver "${_IMPORT_PREFIX}/lib/SofaExplicitOdeSolver.lib" "${_IMPORT_PREFIX}/bin/SofaExplicitOdeSolver.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
