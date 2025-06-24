# 指定库头文件所在路径
FIND_PATH(HFDTech3D_INCLUDE_DIR HFD_OPEN.h hfdDefines.h hfdVector.h hfdVector.inl ${CMAKE_CURRENT_SOURCE_DIR}/ext/HFD/include)

# 指定库文件所在路径
FIND_LIBRARY(HFDTech3D_LIBRARY_Debug HFD_API64d.lib ${CMAKE_CURRENT_SOURCE_DIR}/ext/HFD/lib/Debug)
FIND_LIBRARY(HFDTech3D_LIBRARY_Release HFD_API64.lib ${CMAKE_CURRENT_SOURCE_DIR}/ext/HFD/lib/Release)

# 为了下游可以继续使用
set(HFDTech3D_FOUND FALSE)
if (HFDTech3D_INCLUDE_DIR AND HFDTech3D_LIBRARY_Debug AND HFDTech3D_LIBRARY_Release)
	set(HFDTech3D_FOUND TRUE)
endif()