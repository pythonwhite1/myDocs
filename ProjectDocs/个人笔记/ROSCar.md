# Ros车

## 1.双轮差速小车导出模型

![image-20241104165900400](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104165900400.png)

![image-20241104165932148](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104165932148.png)

![image-20241104170129803](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170129803.png)

![image-20241104170144151](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170144151.png)

![image-20241104170155971](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170155971.png)

![image-20241104170206814](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170206814.png)

![image-20241104170215696](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170215696.png)

![image-20241104170400217](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170400217.png)

![image-20241104170412200](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170412200.png)

![image-20241104170420908](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170420908.png)

![image-20241104170429016](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170429016.png)

![image-20241104170439179](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170439179.png)

![image-20241104170447615](C:\Users\why\AppData\Roaming\Typora\typora-user-images\image-20241104170447615.png)

## 2. 导入依赖

> CMakeLists.txt
>
> ```
> find_package(catkin REQUIRED COMPONENTS
>   gazebo_plugins
>   gazebo_ros
>   gazebo_ros_control
>   urdf
>   xacro
>   amcl
>   gmapping
>   map_server
>   move_base
>   roscpp
>   rospy
>   std_msgs
> )
> ```

>package.xml
>
>```
>      <build_depend>gazebo_plugins</build_depend>
>  <build_depend>gazebo_ros</build_depend>
>  <build_depend>gazebo_ros_control</build_depend>
>  <build_depend>urdf</build_depend>
>  <build_depend>xacro</build_depend>
>  <build_export_depend>gazebo_plugins</build_export_depend>
>  <build_export_depend>gazebo_ros</build_export_depend>
>  <build_export_depend>gazebo_ros_control</build_export_depend>
>  <build_export_depend>urdf</build_export_depend>
>  <build_export_depend>xacro</build_export_depend>
>  <exec_depend>gazebo_plugins</exec_depend>
>  <exec_depend>gazebo_ros</exec_depend>
>  <exec_depend>gazebo_ros_control</exec_depend>
>  <exec_depend>urdf</exec_depend>
>  <exec_depend>xacro</exec_depend>
>    <build_depend>amcl</build_depend>
>  <build_depend>gmapping</build_depend>
>  <build_depend>map_server</build_depend>
>  <build_depend>move_base</build_depend>
>  <build_depend>roscpp</build_depend>
>  <build_depend>rospy</build_depend>
>  <build_depend>std_msgs</build_depend>
>  <build_export_depend>amcl</build_export_depend>
>  <build_export_depend>gmapping</build_export_depend>
>  <build_export_depend>map_server</build_export_depend>
>  <build_export_depend>move_base</build_export_depend>
>  <build_export_depend>roscpp</build_export_depend>
>  <build_export_depend>rospy</build_export_depend>
>  <build_export_depend>std_msgs</build_export_depend>
>  <exec_depend>amcl</exec_depend>
>  <exec_depend>gmapping</exec_depend>
>  <exec_depend>map_server</exec_depend>
>  <exec_depend>move_base</exec_depend>
>  <exec_depend>roscpp</exec_depend>
>  <exec_depend>rospy</exec_depend>
>  <exec_depend>std_msgs</exec_depend>
>```
>
>

## 3.修改myCarV9.urdf

```
1.<robot name="myCarV9">
改为
<robot name="myCarV8" xmlns:xacro="http://www.ros.org/wiki/xacro">
2.重命名文件
myCarV9.urdf.xacro
3.修改base_footprint为
  <link
    name="base_footprint">
    <visual>
      <geometry>
        <sphere radius="0.001" />
      </geometry>
    </visual>
  </link>

```

## 4.新建文件demo.urdf.xacro

```
<robot name="mycar" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="head.xacro" />
    <xacro:include filename="myCarV9.urdf.xacro" />
    <xacro:include filename="demo06_laser.urdf.xacro" />
    <xacro:include filename="gazebo/mymove.urdf.xacro" /> 
</robot>
```

## 5.新建文件laser.urdf.xacro

```
<robot name="my_sensors" xmlns:xacro="http://wiki.ros.org/xacro">

  <gazebo reference="laser">
    <sensor type="ray" name="rplidar">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>5.5</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>-3</min_angle>
            <max_angle>3</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_rplidar" filename="libgazebo_ros_laser.so">
        <topicName>/scan</topicName>
        <frameName>laser</frameName>
      </plugin>
    </sensor>
  </gazebo>

</robot>

```

## 6. 新建文件head.xacro

```
<robot name="base" xmlns:xacro="http://wiki.ros.org/xacro">
    <!-- Macro for inertia matrix -->
    <xacro:macro name="sphere_inertial_matrix" params="m r">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${2*m*r*r/5}" ixy="0" ixz="0"
                iyy="${2*m*r*r/5}" iyz="0" 
                izz="${2*m*r*r/5}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertial_matrix" params="m r h">
        <inertial>
            <mass value="${m}" />
            <inertia ixx="${m*(3*r*r+h*h)/12}" ixy = "0" ixz = "0"
                iyy="${m*(3*r*r+h*h)/12}" iyz = "0"
                izz="${m*r*r/2}" /> 
        </inertial>
    </xacro:macro>

    <xacro:macro name="Box_inertial_matrix" params="m l w h">
       <inertial>
               <mass value="${m}" />
               <inertia ixx="${m*(h*h + l*l)/12}" ixy = "0" ixz = "0"
                   iyy="${m*(w*w + l*l)/12}" iyz= "0"
                   izz="${m*(w*w + h*h)/12}" />
       </inertial>
   </xacro:macro>
   <xacro:property name="PI" value="3.1415926"/>
</robot>
```

## 7.新建文件夹gazebo，新建文件mymove.urdf.xacro

```
<robot name="my_car_move" xmlns:xacro="http://wiki.ros.org/xacro">

    <xacro:macro name="joint_trans" params="joint_name">
        <!-- Transmission is important to link the joints and the controller -->
        <transmission name="${joint_name}_trans">
            <type>transmission_interface/SimpleTransmission</type>
            <joint name="${joint_name}">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
            </joint>
            <actuator name="${joint_name}_motor">
                <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
                <mechanicalReduction>1</mechanicalReduction>
            </actuator>
        </transmission>
    </xacro:macro>

    <xacro:joint_trans joint_name="left_wheel2base_link" />
    <xacro:joint_trans joint_name="right_wheel2base_link" />

    <gazebo>
        <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
            <rosDebugLevel>Debug</rosDebugLevel>
            <publishWheelTF>true</publishWheelTF>
            <robotNamespace>/</robotNamespace>
            <publishTf>1</publishTf>
            <publishWheelJointState>true</publishWheelJointState>
            <alwaysOn>true</alwaysOn>
            <updateRate>100.0</updateRate>
            <legacyMode>true</legacyMode>
            <leftJoint>left_wheel2base_link</leftJoint> 
            <rightJoint>right_wheel2base_link</rightJoint> 
            <wheelSeparation>0.389</wheelSeparation> 
            <wheelDiameter>0.150</wheelDiameter> 
            <broadcastTF>1</broadcastTF>
            <wheelTorque>30</wheelTorque>
            <wheelAcceleration>1.8</wheelAcceleration>
            <commandTopic>cmd_vel</commandTopic> 
            <odometryFrame>odom</odometryFrame> 
            <odometryTopic>odom</odometryTopic> 
            <robotBaseFrame>base_footprint</robotBaseFrame> 
        </plugin>
    </gazebo>

</robot>
```

## 8.新建launch文件

```
<launch>

    <param name="robot_description" command="$(find xacro)/xacro $(find myCarV8)/urdf/demo.urdf.xacro" />

    <include file="$(find gazebo_ros)/launch/empty_world.launch" >
            <arg name="world_name" value="$(find myCarV8)/worlds/hello.world" />
    </include>

    <node pkg="gazebo_ros" type="spawn_model" name="model" args="-urdf -model mycar -param robot_description" />
</launch>

```

