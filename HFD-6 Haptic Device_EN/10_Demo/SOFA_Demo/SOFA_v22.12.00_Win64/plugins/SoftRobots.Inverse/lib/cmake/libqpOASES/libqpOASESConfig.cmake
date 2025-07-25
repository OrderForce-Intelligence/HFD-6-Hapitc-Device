# CMake package configuration file for the plugin 'libqpOASES'


####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was libqpOASESConfig.cmake.in                            ########

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

if(NOT TARGET libqpOASES)
    include("${CMAKE_CURRENT_LIST_DIR}/libqpOASESTargets.cmake")
endif()

check_required_components(libqpOASES)

set(libqpOASES_LIBRARY libqpOASES)
set(libqpOASES_LIBRARIES libqpOASES)
set(libqpOASES_INCLUDE_DIRS J:/jenkins2/workspace/sofa-custom/refs/heads/v22.12/windows_vs2019_release_full_python3.8/plugins/SoftRobots.Inverse/extlibs/qpOASES-3.2.0/include)
