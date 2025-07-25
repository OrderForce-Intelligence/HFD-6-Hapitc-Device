#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "SceneCreator" for configuration "MinSizeRel"
set_property(TARGET SceneCreator APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(SceneCreator PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/SceneCreator.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/SceneCreator.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS SceneCreator )
list(APPEND _IMPORT_CHECK_FILES_FOR_SceneCreator "${_IMPORT_PREFIX}/lib/SceneCreator.lib" "${_IMPORT_PREFIX}/bin/SceneCreator.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
