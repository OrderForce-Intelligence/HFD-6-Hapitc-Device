# CMake package configuration file for ExternalBehaviorModel
### Expanded from @PACKAGE_GUARD@ by SofaMacrosInstall.cmake ###
include_guard()
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../bin")
list(APPEND CMAKE_LIBRARY_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../lib")
################################################################

####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was ExternalBehaviorModelConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

find_package(Sofa.Component.LinearSolver.Iterative REQUIRED)
find_package(Sofa.Component.Topology REQUIRED)
find_package(Sofa.Component.Constraint.Projective REQUIRED)
find_package(SofaGraphComponent REQUIRED)
find_package(Sofa.Component.ODESolver.Backward REQUIRED)
find_package(Sofa.Component.SolidMechanics.FEM.Elastic REQUIRED)

if(NOT TARGET ExternalBehaviorModel)
    include("${CMAKE_CURRENT_LIST_DIR}/ExternalBehaviorModelTargets.cmake")
endif()

check_required_components(ExternalBehaviorModel)
