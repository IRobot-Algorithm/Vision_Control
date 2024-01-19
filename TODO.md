短期可以解决：
1.进入docker容器内的终端后字体全为白色，需要手动输入source /etc/bash.bashrc才能变彩色，需要将此过程自动化

暂时不清楚的问题：
1.skider_control包launch文件的参数路径问题
如果采用绝对路径，部署时要更改一次绝对路径。
如果采用get_package_share_directory的方式寻找路径，则每次调整pid参数都要编译，较为麻烦

代码重构：
1.需要重新写PID类并调整PID类的文件位置
2.修改姿态解算部分代码实现编译时不输出warning
