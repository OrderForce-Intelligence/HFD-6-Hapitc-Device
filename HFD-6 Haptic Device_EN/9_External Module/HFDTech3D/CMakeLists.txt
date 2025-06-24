#  $Author: SunRenKai $
#  $Date: 2023-10-12 $
cmake_minimum_required(VERSION 3.17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)

PROJECT(HFDTech3D)
set(CMAKE_DEBUG_POSTFIX d)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
FIND_PACKAGE(HFDTech3D REQUIRED)

include_directories(
${CMAKE_CURRENT_SOURCE_DIR}/include
${CMAKE_CURRENT_SOURCE_DIR}/ext/HFD/include
)

add_library(HFDTech3D SHARED ${CMAKE_CURRENT_SOURCE_DIR}/src/HFDTech3D.cpp ${CMAKE_CURRENT_SOURCE_DIR}/include/HFDTech3D.h)

target_link_directories(
${PROJECT_NAME} 
PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/include
${CMAKE_CURRENT_SOURCE_DIR}/ext
)

target_link_libraries(
${PROJECT_NAME} 
PRIVATE
debug ${HFDTech3D_LIBRARY_Debug}
optimized ${HFDTech3D_LIBRARY_Release} 
)