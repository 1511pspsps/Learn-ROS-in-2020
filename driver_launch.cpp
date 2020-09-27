<!--
	��е��������ͷ֮���У׼���ܡ�ע������ͷ��Ҫ����װ����ֱ���¡�ʹ��һ����ɫԲ��������
	�����Ϸ���У׼�㡣
-->
<launch>

	<!--UARM��е��-->
	<include file="$(find swiftpro)/launch/pro_control_nomoveit.launch"/> // ����pro_control_nomoveit.launch

	<!--spark���������������������������-->
	<include file="$(find spark_bringup)/launch/driver_bringup.launch"/> // ����driver_bringup.launch

	 <arg name="rvizconfig" default="$(find spark_teleop)/rviz/teleop.rviz" />
	<node name="rviz" pkg="rviz" type="rviz" args="-d $(arg rvizconfig)" // ����rviz
	required="true" />

</launch>