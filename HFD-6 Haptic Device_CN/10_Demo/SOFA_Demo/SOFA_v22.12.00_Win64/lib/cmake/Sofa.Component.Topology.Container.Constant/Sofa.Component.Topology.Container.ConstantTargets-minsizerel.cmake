#----------------------------------------------------------------
# Generated CMake target import file for configuration "MinSizeRel".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Sofa.Component.Topology.Container.Constant" for configuration "MinSizeRel"
set_property(TARGET Sofa.Component.Topology.Container.Constant APPEND PROPERTY IMPORTED_CONFIGURATIONS MINSIZEREL)
set_target_properties(Sofa.Component.Topology.Container.Constant PROPERTIES
  IMPORTED_IMPLIB_MINSIZEREL "${_IMPORT_PREFIX}/lib/Sofa.Component.Topology.Container.Constant.lib"
  IMPORTED_LOCATION_MINSIZEREL "${_IMPORT_PREFIX}/bin/Sofa.Component.Topology.Container.Constant.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS Sofa.Component.Topology.Container.Constant )
list(APPEND _IMPORT_CHECK_FILES_FOR_Sofa.Component.Topology.Container.Constant "${_IMPORT_PREFIX}/lib/Sofa.Component.Topology.Container.Constant.lib" "${_IMPORT_PREFIX}/bin/Sofa.Component.Topology.Container.Constant.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
