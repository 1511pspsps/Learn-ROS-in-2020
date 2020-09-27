#ifndef _LINE_FOLLOWER_H
#define _LINE_FOLLOWER_H	//�����ͷ�ļ�

#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<geometry_msgs/Twist.h> //�������ͷ�ļ�

#define LINE_ROI_WIDTH 250  //ROIͼ����
#define LINE_ROI_HIGH 150	//ROIͼ��߶�
#define LINE_ROI_HIGH_BIAS 20	//ROI����Ȥ�ļ������
#define CORNER_DETE_THRESH 100	
#define LINE_CONTROLLER_POS 20
#define CONTROLLER_KP 0.025  //PID�㷨��P��������0.015����Ϊ0.025�������û����˽������Ѳ�ߵ�ת�䣬��С��Ȧ�Ĵ�С����������ʱ�䡣
#define CONTROLLER_KD 0.007	 //PID�㷨�е�D������


using namespace cv;	//����cv�����ռ�

class LineFollower  //����LineFollower��
{
private:
	ros::NodeHandle nh_;	//����˽�б��� ros��� nh_

	// ????publisher
	ros::Publisher car_tw_pub_;	//������Ϣ�ķ�����

public:
	LineFollower(ros::NodeHandle& nh); //���캯��
	~LineFollower();	//��������
	int count=0;	//��������count
	void go_straight();
	void turn_90_degree(int direct);
	void turn_around();
	void stop();	//������������

	// Get ROI
	Mat get_ROI(const Mat& orig);	//��������

	// Line follow control function
	bool follow_line_controller(const Mat& roi);	//��������
};
#endif 
