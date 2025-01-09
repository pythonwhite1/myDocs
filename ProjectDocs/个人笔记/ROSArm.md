# ROS机械臂

## 一、自定义Joint坐标系和Link运动参考轴，在Solid works使用sw2urdf导出

## 二、创建新的ROS工作空间

```
mkdir -p myArm_ws/src
cd myArm_ws
catkin_make
```

### 将步骤1导出的功能包放入src目录下

```
catkin_make
source devel/setup.bash
roslaunch 机械臂功能包名 display.launch
```

### 配置rviz后可以看到模型

```
将display.launch文件内的
<arg
  name="gui"
  default="False" />
False 改为 Ture可以控制机械臂各个关机的运动
```

## 三、自定义关节运动发布节点

> 回到myArm_ws下，创建发布节点的功能包

```
catkin_create_pkg myPub roscpp rospy std_msgs sensor_msgs tf
```

> 在新创建的功能包下创建scripts文件夹，进入文件夹新建py文件joint_states_pub.py

```python
#! /usr/bin/env python
#coding=utf-8
import rospy
from sensor_msgs.msg import JointState
def demo():
    joint_state_pub = rospy.Publisher('/joint_states',JointState,queue_size=10)
    joint_state = JointState()
    joint_state.name = ["Joint1","joint2","joint3","joint4","joint5","joint6","gipperLeftJoint","gipperRightJonit"]
    joint_state.position = [0,0,0,0,0,0,0,0]
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        for num in range(0,100):
            joint_state.header.stamp = rospy.Time.now()
            joint_state.position[0] = joint_state.position[0] + 0.02
            joint_state.position[1] = joint_state.position[1] + 0.015
            joint_state.position[2] = joint_state.position[2] + 0.015
            joint_state.position[3] = joint_state.position[3] - 0.015
            joint_state.position[6] = joint_state.position[6] + 0.00030
            joint_state.position[7] = joint_state.position[7] - 0.00030
            joint_state_pub.publish(joint_state)
            rate.sleep()
        for num in range(0,100):
            joint_state.header.stamp = rospy.Time.now()
            joint_state.position[0] = joint_state.position[0] - 0.02
            joint_state.position[1] = joint_state.position[1] - 0.015
            joint_state.position[2] = joint_state.position[2] - 0.015
            joint_state.position[3] = joint_state.position[3] + 0.015
            joint_state.position[6] = joint_state.position[6] - 0.00030
            joint_state.position[7] = joint_state.position[7] + 0.00030
            joint_state_pub.publish(joint_state)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('joint_states_pub')
    try:
        rospy.loginfo('joint_states_pub node...')
        demo()
    except rospy.ROSInterruptException:
        rospy.loginfo('joint_states_pub node initialize failed,please retry')

```

> py文件创建完成后，需要给予可执行权限
>
> ```
> chmod +x joint_states_pub.py
> ```



> 修改配置文件，CMakeLists.txt

```
 catkin_install_python(PROGRAMS
   scripts/joint_states_pub.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
```

> 创建launch文件夹，并在文件夹中新建launch文件，urdf_states_python.launch

```
<launch>
  <param
    name="robot_description"
    textfile="$(find MyArm_C)/urdf/MyArm_C.urdf" />
  <node
    name="joint_states_pub"
    pkg="myPub"
    type="joint_states_pub.py"
    output="screen" />
  <node
    name="robot_state_publisher"
    pkg="robot_state_publisher"
    type="state_publisher" />
  <node
    name="rviz"
    pkg="rviz"
    type="rviz"
    args="-d $(find MyArm_C)/urdf.rviz" />
</launch>
```

## 四、ros与STM32通过串口通信

### 4.1 在树莓派安装串口助手

```
sudo apt-get install cutecom
sudo cutecom
```

### 4.2 打开串口助手，将USB转TTL连接树莓派的USB接口和STM32的串口

> cutecom的Decive选择/dev/ttyUSB*，*可以通过插拔USB查看

### 4.3 测试是否正常通信

### 4.4 安装rosserial

```
sudo apt-get install ros-melodic-serial
```

### 4.5 创建ros工作空间，创建功能包

> 注意功能包中需包含serial

### 4.6 新建cpp文件

```c++
//serial_port.cpp
#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;
    
    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);
 
    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    
    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }
    
    ros::Rate loop_rate(500);
    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        if(n!=0)
        {
            uint8_t buffer[1024];
            //读出数据
            n = sp.read(buffer, n);
            
            for(int i=0; i<n; i++)
            {
                //16进制的方式打印到屏幕
                std::cout << std::hex << (buffer[i] & 0xff) << " ";
            }
            std::cout << std::endl;
            //把数据发送回去
            sp.write(buffer, n);
        }
        loop_rate.sleep();
    }
    
    //关闭串口
    sp.close();
 
    return 0;
}

```

### 4.7 修改cMakeList.txt

```
cmake_minimum_required(VERSION 3.0.2)
project(serial_node)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES serial_node
#  CATKIN_DEPENDS roscpp rospy stdmsgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/serial_node.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(serial_node src/serialTest.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(serial_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(serial_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_serial_node.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

```



### 4.8 无法打开串口

>运行时如果提示串口打开失败，有两种原因，一是串口号不对，使用dmesg | grep ttyS*列出检测到的串口号，逐个测试；二是没有操作权限，使用sudo chmod 666 /dev/ttyUSB0即可解决，也可以使用sudo usermod -aG dialout 用户名   来获得永久权限，用户名可使用whoami查看
>
>



## 五、连接真实机械臂

### 5.1 修改机械臂配置文件

#### 5.5.1 修改demo.launch文件

```
<arg name="fake_execution" value="true"/>
true改为false
```

```
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" unless="$(arg use_gui)">
    <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
  </node>
  改为
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" unless="$(arg use_gui)">
    <rosparam param="/source_list">[/joint_states]</rosparam >
  </node>
```

#### 5.5.2 修改trajectory_execution.launch.xml

```
  <!-- When determining the expected duration of a trajectory, this multiplicative factor is applied to get the allowed duration of execution -->
  <param name="trajectory_execution/allowed_execution_duration_scaling" value="6"/> <!-- default 1.2 -->
  <!-- Allow more than the expected execution time before triggering a trajectory cancel (applied after scaling) -->
  <param name="trajectory_execution/allowed_goal_duration_margin" value="6"/> <!-- default 0.5 -->
  <!-- Allowed joint-value tolerance for validation that trajectory's first point matches current robot state -->
  <param name="trajectory_execution/allowed_start_tolerance" value="6"/> <!-- default 0.01 -->
  
  适当增加value
```

### 5.2 创建action服务端

> 依赖如下
>
>  actionlib
>
>  actionlib_msgs
>
>  control_msgs
>
>  roscpp
>
>  rospy
>
>  sensor_msgs
>
>  std_msgs

```c++
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryActionGoal.h>
#include <control_msgs/FollowJointTrajectoryActionResult.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <stdio.h>

#define scaler 423
#define offset 1500

using namespace std;

class FollowJointTrajectoryAction

{
    protected:
        sensor_msgs::JointState js;
        ros::NodeHandle nh;
        std::string action_name_;
        ros::Publisher pub_joint;

        //与moveit中action client通讯的action server
        actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
        control_msgs::FollowJointTrajectoryActionResult result_;
        control_msgs::FollowJointTrajectoryActionGoal goal_;
    public:
        FollowJointTrajectoryAction(std::string name) :
        as_(nh,name,false),
        action_name_(name)
        {
            as_.registerGoalCallback(boost::bind(&FollowJointTrajectoryAction::goalCB, this));
            as_.registerPreemptCallback(boost::bind(&FollowJointTrajectoryAction::preemptCB, this));
            as_.start();
            pub_joint = nh.advertise<sensor_msgs::JointState>("/joint_plan", 10);

            js.name.resize(6);
            js.position.resize(6);
            //名字要与关节定义的名字一致
            js.name[0] = "joint1";
            js.name[1] = "joint2";
            js.name[2] = "joint3";
            js.name[3] = "joint4";
            js.name[4] = "joint5";
            js.name[5] = "joint6";//----------------------
            ROS_INFO("-------action start!-------");
        }
    ~FollowJointTrajectoryAction(void)
    {
    }

    void goalCB()
    {
        ROS_INFO("goalCB");
        std::vector<trajectory_msgs::JointTrajectoryPoint> points_;
        double points_end[6];
        double Pos_length;
        if(as_.isNewGoalAvailable()){
            js.position.clear();
            points_ = as_.acceptNewGoal()->trajectory.points;
            Pos_length = points_.size();
            for(int i=0;i<6;i++){
                points_end[i] = points_.at(Pos_length - 1).positions[i];
                js.position.push_back(points_end[i]);
            }
            js.header.stamp = ros::Time::now();
            //向move_group节点发布规划得到的关节值
            pub_joint.publish(js);

            ROS_INFO("-------goal is receive done!-------");
        }else{
            ROS_INFO("-------goal is not availabel!-------");
        }
        control_msgs::FollowJointTrajectoryResult result;
        result.error_code = 0;
        as_.setSucceeded(result);

    }
    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        as_.setPreempted();
    }
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "arm_driver");
    FollowJointTrajectoryAction followjointtrajectory("cuteArm_controller/follow_joint_trajectory");//名称要与yaml配置一致
    ros::spin();
    return 0;
}

```

## 六、网络代理

```
1、在终端输入：$HOME/.clash/start_clash.sh
2、配置系统默认代理方式：系统设置->网络->网络代理->手动->HTTP(127.0.0.1 7890)->HTTPS(127.0.0.1 7890)
```

## 七、使用自定义轨迹规划算法

### 1. 通过源码安装moveit

```
# 删除通过binary安装的moveit
sudo apt-get remove ros-melodic-moveit-*
#更新软件包
rosdepc update
sudo apt-get update
sudo apt-get dist-upgrade
#安装依赖
sudo apt-get install python-wstool python-catkin-tools clang-format-3.9 
#创建moveit的工作空间
mkdir ~/ws_moveit
cd ~/ws_moveit
#加载环境变量
source /opt/ros/melodic/setup.bash
#下载moveit源码并编译
wstool init src
wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool update -t src
rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
sudo catkin build

```

### 2. 通过源码安装ompl

```
# 卸载原来的ompl
sudo apt-get purge ros-melodic-ompl

# 源码安装ompl
cd ~/ws_moveit/src
git clone https://github.com/ompl/ompl
sudo catkin build


```

### 3. 自定义路径规划算法

```
# 创建规划算法
> 在ws_moveit/src/ompl/src/ompl/geometric/planners/rrt目录下，把RRTconnect.h文件复制并重命名为cqRRTconnect.h

> 同样，在ws_moveit/src/ompl/src/ompl/geometric/planners/rrt/src目录下把RRTconnect.cpp文件复制并重命名为cqRRTconnect.cpp。

> 然后在这两个文件中把RRTconnect全部替换为cqRRTconnect

# 修改 planning_context_manager.cpp   （路径：moveit_ws/src/moveit/moveit_planners/ompl/ompl_interface/src）
# 添加头文件
include <ompl/geometric/planners/rrt/cqRRTConnect.h>   # 注意这里添加绝对路径
#在下面函数中仿照其他算法进行注册
void ompl_interface::PlanningContextManager::registerDefaultPlanners()
```

```
#修改机械臂功能包中的ompl_planning.yaml（路径：/ur5_moveit_config/config），仿照添加两处
cqRRTConnect:
    type: geometric::cqRRTConnect
    range: 0.0  # Max motion added to tree. ==> maxDistance_ default: 0.0, if 0.0, set on setup()


 - cqRRTConnect

```





### 解决cmake版本低

https://blog.csdn.net/weixin_44760904/article/details/134249109



### 参考

```
https://blog.csdn.net/road_of_god/article/details/122352259?spm=1001.2101.3001.6650.3&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-3-122352259-blog-133911701.235%5Ev43%5Econtrol&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-3-122352259-blog-133911701.235%5Ev43%5Econtrol&utm_relevant_index=4


https://blog.csdn.net/StephenZzzzz/article/details/133911701?spm=1001.2101.3001.6650.6&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-6-133911701-blog-105312020.235%5Ev43%5Econtrol&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7EBlogCommendFromBaidu%7ERate-6-133911701-blog-105312020.235%5Ev43%5Econtrol&utm_relevant_index=11


https://blog.csdn.net/weixin_36965307/article/details/105312020
```

## 八、相机标定

### 1.下载标定板文件

http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration?action=AttachFile&do=view&target=check-108.pdf

### 2.安装camera-calibration

sudo apt install ros-melodic-camera-calibration

### 3.启动相机

roslaunch usb_cam usb_cam.launch

### 4.启动校准

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0245 image:=/usb_cam/image_raw camera:=/usb_cam

## 手眼标定

sudo apt-get install ros-melodic-visp

git clone -b melodic-devel https://github.com/lagadic/vision_visp.git

git clone -b melodic-devel https://github.com/pal-robotics/aruco_ros.git

git clone https://github.com/IFL-CAMP/easy_handeye.git

