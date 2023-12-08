#!/bun/bash
###该脚本用于自动检索ros工作目录下缺失的依赖文件并自动安装

#定义rosdep执行扫描目录
workdir="/home/autoware/shared_dir/beatcat/src"
pkg=`rosdep keys --from-paths /home/autoware/shared_dir/beatcat/src  --rosdistro $ROS_DISTRO | sed "s/\_/\-/g"`

for i in $pkg
do
  pg="ros-melodic-$i"
  echo "\n>>>>>安装$pg>>>>>>\n"
  echo -e "root1234" | sudo apt-get install -y $pg
  echo "\n<<<<<$pg安装完成<<<<<<\n"
done