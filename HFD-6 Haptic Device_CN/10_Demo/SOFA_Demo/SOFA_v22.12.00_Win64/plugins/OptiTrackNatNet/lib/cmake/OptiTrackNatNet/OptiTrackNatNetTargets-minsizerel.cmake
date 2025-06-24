#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OptiTrackNatNet" for configuration "MinSizeRel"
set_property(TARGET OptiTrackNatNet APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(OptiTrackNatNet PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/OptiTrackNatNet.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/OptiTrackNatNet.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS OptiTrackNatNet )
list(APPEND _IMPORT_CHECK_FILES_FOR_OptiTrackNatNet "${_IMPORT_PREFIX}/lib/OptiTrackNatNet.lib" "${_IMPORT_PREFIX}/bin/OptiTrackNatNet.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
