#ifndef _LINE_FOLLOWER_H
#define _LINE_FOLLOWER_H	//定义该头文件

#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<geometry_msgs/Twist.h> //引入各个头文件

#define LINE_ROI_WIDTH 250  //ROI图像宽度
#define LINE_ROI_HIGH 150	//ROI图像高度
#define LINE_ROI_HIGH_BIAS 20	//ROI感兴趣的检测行数
#define CORNER_DETE_THRESH 100	
#define LINE_CONTROLLER_POS 20
#define CONTROLLER_KP 0.025  //PID算法中P参数，由0.015增加为0.025，可以让机器人较早进行巡线的转弯，减小绕圈的大小，减少所用时间。
#define CONTROLLER_KD 0.007	 //PID算法中的D参数。


using namespace cv;	//引入cv命名空间

class LineFollower  //定义LineFollower类
{
private:
	ros::NodeHandle nh_;	//定义私有变量 ros句柄 nh_

	// ????publisher
	ros::Publisher car_tw_pub_;	//定义信息的发布者

public:
	LineFollower(ros::NodeHandle& nh); //构造函数
	~LineFollower();	//析构函数
	int count=0;	//定义整形count
	void go_straight();
	void turn_90_degree(int direct);
	void turn_around();
	void stop();	//声明各个函数

	// Get ROI
	Mat get_ROI(const Mat& orig);	//声明函数

	// Line follow control function
	bool follow_line_controller(const Mat& roi);	//声明函数
};
#endif 
