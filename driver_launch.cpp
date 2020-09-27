<!--
	机械臂与摄像头之间的校准功能。注意摄像头需要反着装，垂直向下。使用一个蓝色圆贴在吸盘
	的正上方当校准点。
-->
<launch>

	<!--UARM机械臂-->
	<include file="$(find swiftpro)/launch/pro_control_nomoveit.launch"/> // 调用pro_control_nomoveit.launch

	<!--spark驱动，机器人描述，相机，底盘-->
	<include file="$(find spark_bringup)/launch/driver_bringup.launch"/> // 调用driver_bringup.launch

	 <arg name="rvizconfig" default="$(find spark_teleop)/rviz/teleop.rviz" />
	<node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" // 启动rviz
	required="true" />

</launch>