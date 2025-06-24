请将该工程移入sofa-22.12.00\applications\plugins文件夹内
然后打开sofa-22.12.00\applications\plugins\CMakeLists.txt
添加sofa_add_subdirectory(plugin HFD HFD)
接着返回cmake的plugins选项选择plugin_HFD
就可以正常编译运行