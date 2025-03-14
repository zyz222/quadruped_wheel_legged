这是一个纯python的四轮足机器人位置控制项目，在gazebo中进行仿真。代码有些潦草 只有基础的平衡控制和键盘控制前后左右。 只适用于新手了解一些怎么入门。
C++版本的力控项目也很快开源。
使用方法： roslaunch xingtian xingtianlaunch 启动仿真器
python control.py  启动电机控制器
python pos_control_ros.py  启动姿态控制器
python teleop_control.py 启动键盘控制器
