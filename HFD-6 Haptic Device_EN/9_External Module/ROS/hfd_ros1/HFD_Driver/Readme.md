1、创建catkin工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ..
catkin_make

2、编译hfd_driver文件夹
将hfd_driver复制到~/catkin_ws/src
然后在~/catkin_ws内运行catkin_make

注意：catkin_make后需要运行rospack list检查ros是否能索引
若检索不到请用source devel/setup.bash进行更新

3、运行hfd_driver
roscore
chmod 777 /dev/ttyACM1
rosrun hfd_driver hfd_driver
