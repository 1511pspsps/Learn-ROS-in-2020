#include<ros/ros.h>
#include<image_transport/image_transport.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include "line_follower.h"
#include<iostream>
#include "swiftpro/position.h"

//Image value
Mat origin;
//当前执行任务，当接收到第一张图才变
int mode = 0; //设置为模式0

//图像接受回调函数
void imageCb(const sensor_msgs::ImageConstPtr& msg); //回调函数
int color_recognition(Mat img);
void color9(Mat img, int& x, int& y);
cv::Rect find_contours(cv::Mat img);
void arm_draw_triangle(ros::Publisher& pub);
void arm_draw_square(ros::Publisher& pub);
//声明这些函数，方便后面定义

int main(int argc, char** argv)
{
	//初始化节点
	ros::init(argc, argv, "final_homework");
	//创建句柄
	ros::NodeHandle nh;

	//cv_bridge image subscriber
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_raw", 1, &imageCb); //定义发布者和接受者对象
	image_transport::Publisher image_pub = it.advertise("/image_converter/output_video", 1); //定义发布者和接受者对象

	ros::Publisher swiftpro_pub = nh.advertise<swiftpro::position>("/position_write_topic", 1);  //创建发布消息的接口
	
	ros::spinOnce(); //回调处理，接受订阅的消息，执行下面程序。

	//Line follow ROI，定义一个Mat类成员
	Mat line_roi;

	//Reach flag 定义一个布尔类型，确定是否到达巡线终点。
	bool reach_flag = false;
	
	LineFollower linefollower(nh);

	ros::Rate loop_rate(0.25); //写在循环外部，与loop_rate.sleep()同时使用，0.25指两次发布消息间隔4s

	while (nh.ok())
	{
		switch (mode)
		{
			//任务一：识别图形控制机械臂
			case 1:
			{
				ROS_INFO_THROTTLE(1, "Color recognizing!!!");//发布消息，Color recognizing!!!
				int color = color_recognition(origin);//定义整形color，color为1对应蓝色，color为2对应绿色，color为3对应红色。
				if (color)
				{
					mode = 2;
					ROS_INFO("Pass target one!,The color is %d", color); //发布消息Pass target one!,The color is "color的数值".

					switch (color)
					{
					case 1: arm_draw_triangle(swiftpro_pub); break;//画三角
					case 2: arm_draw_square(swiftpro_pub); break;//画正方形
					case 3: arm_draw_circle(swiftpro_pub); break;//画圆
					}
					swiftpro::position finish;//定义finish，控制结束时机械臂的位置
					finish.x = 50;
					finish.y = 200;
					std::cout << "x=50,y=200" << std::endl;//显示机械臂终止位置
					swiftpro_pub.publish(finish);//发布结束时机械臂的位置
					//根据所需不同的调整时间来确定休眠时间，确保机械臂归位后再进行移动
					if (color == 1)
						ros::Duration(6.5).sleep();//休眠6.6秒
					else if (color == 2)
						ros::Duration(5.5).sleep();//休眠5.5秒
					else if (color == 3)
						ros::Duration(2).sleep();//休眠2秒
				}
				break;
			}
			//任务二：巡线
			case 2:
			{
				line_roi = linefollower.get_ROI(origin); //得到初始图像
				imshow("line_roi", line_roi); //显示line_roi图像
				reach_flag = linefollower.follow_line_controller(line_roi); //巡线
				if (reach_flag)
				{
					destroyAllWindows(); //关闭所有窗口
					mode = 3;
				}
				break;
			}
			//任务三：识别九宫格
			case 3:
			{
				int x = 0, y = 0;
				color9(origin, x, y); //识别九宫格
				ROS_INFO("Object is on (%d, %d)", x, y); //发布消息，Object is on（位置）
				loop_rate.sleep(); //写在循环内部，与ros::Rate loop_rate同时使用。
				break;
			}
		}
		ros::spinOnce(); //回调处理，接受订阅的消息，执行下面程序。
		waitKey(1); //显示窗口，一毫秒后关闭
	}
}

void imageCb(const sensor_msgs::ImageConstPtr& msg) // 回调函数
{
	static bool first_time_flag = true; // 第一次启动的标记
	if (first_time_flag == true) // 判断是否为第一次启动
	{
		first_time_flag = false; // 将标记改为false
		mode = 1; // 进入模式一
	}
	cv_bridge::CvImagePtr cv_ptr; // 定义一个图像指针
	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); // 复制图像数据

	origin = cv_ptr->image; // 将图像数据存入origin中
	//cv::imshow("Origin image",origin);

	//waitKey(3);
}

int color_recognition(Mat img) // 识别颜色
{
	enum colors { white, blue, green, red }; // 定义枚举  white=0, blue=1, green=2, red=3
	cv::Mat img1; // 定义一个cv中常用的用于存图像数据的矩阵
	cv::cvtColor(img, img1, CV_BGR2GRAY); // 把图像从BGR空间转换到GRAY（灰度）空间
	cv::GaussianBlur(img1, img1, Size(5, 5), 2, 2); // 对图像进行高斯滤波处理
	std::vector<Vec3f> circles;
	cv::HoughCircles(img1, circles, CV_HOUGH_GRADIENT, 1, 30, 180, 60, 20, 500); // 寻找图像中的霍夫圆
	cv::Mat img2;
	cv::cvtColor(img, img2, CV_BGR2HSV); // 把图像从BGR空间转换到GRAY（灰度）空间
	std::vector<cv::Mat> channels; // 创建颜色通道
	cv::split(img2, channels); // 将图像分成多个颜色通道

	for (size_t i = 0;i < circles.size();i++)
	{
		// cvRound是四舍五入函数
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1])); // 确定圆心
		int radius = cvRound(circles[i][2]); // 确定半径
		cv::circle(img, center, radius, cv::Scalar(255, 255, 255), 1, 8, 0); // 在原视图像上绘制圆形
	}
	cv::imshow("hough", img); // 显示图像

	if (circles.size() != 0) // 当存在圆时
	{
		colors color;
		int hue = channels[0].at<uchar>(circles[0][1], circles[0][0]); // 取出圆心处数据
		std::cout << hue << std::endl;
		// 使用色调来区分颜色
		if (hue >= 100 && hue <= 140)color = blue; // 色调值100~140为蓝色
		if (hue >= 35 && hue <= 95)color = green; // 色调值35~95为绿色
		if (hue <= 30 || hue >= 150)color = red; // 色调值<=30或>=150为红色
		circles.clear(); // 清除cirles中的数据
		channels.clear(); // 清楚channels中的数据
		return color; // 返回所识别出的颜色
	}
	circles.clear(); // 清除cirles中的数据
	channels.clear(); // 清楚channels中的数据
	return 0; // 函数结束
}

void color9(cv::Mat img, int& x, int& y) //识别九宫格
{
	cv::Mat img1;
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(8, 8));//得到 8*8 的矩形结构 
	cv::cvtColor(img, img1, CV_BGR2HSV);//将RGB空间的图片img转化为HSV颜色空间的图片img1
	std::vector<cv::Mat> channels;//用vector储存图像
	cv::split(img1, channels);//利用图片对象img1分离
	cv::imshow("001", channels[1]);//显示绿色通道的图像

	cv::Mat img2;
	cv::threshold(channels[1], img2, 100, 255, cv::THRESH_BINARY);//将灰度值大于100的设置为255，不大于thresh设置为0。
	cv::morphologyEx(img2, img2, cv::MORPH_OPEN, element);//形态学变化：先进行腐蚀操作，再进行膨胀操作，消除小黑点
	cv::morphologyEx(img2, img2, cv::MORPH_CLOSE, element);//形态学变化：先进行膨胀，再进行腐蚀操作，排除小黑洞
	cv::Canny(img2, img2, 220, 255);//使用Canny边界检测， 后面两个参数代表双阈值检测的minVal和maxVal
	imshow("003", img2);
	cv::Rect maxRect = find_contours(img2);//寻找img2的轮廓,返回到maxRect中

	cv::Mat roi = channels[0](maxRect);//返回蓝色通道图像中的ROI
	cv::threshold(roi, roi, 30, 255, cv::THRESH_BINARY_INV);//将灰度值大于30的设置为0，不大于thresh设置为255。
	cv::morphologyEx(roi, roi, cv::MORPH_CLOSE, element);//形态学变化：先进行膨胀，再进行腐蚀操作，排除小黑洞
	cv::morphologyEx(roi, roi, cv::MORPH_OPEN, element);//形态学变化：先进行腐蚀操作，再进行膨胀操作，消除小黑点
	//画出图像中的矩形,(10,10)为矩形左上角，宽度，长度分别为maxRect宽度，长度的一半
	cv::Mat roil = roi(cv::Rect(10, 10, maxRect.width - 20, maxRect.height - 20));

	cv::Canny(roil, roil, 250, 255);//使用Canny边界检测， 后面两个参数代表双阈值检测的minVal和maxVal
	cv::Rect roiRect = find_contours(roil);//寻找roil的轮廓, 返回到roilRect中
	//画出图像中的矩形,(maxRect.x + roiRect.x, maxRect.y + roiRect.y,)为矩形左上角，宽度，长度分别等于roilRect宽度，长度
	cv::Rect rect(maxRect.x + roiRect.x, maxRect.y + roiRect.y, roiRect.width, roiRect.height);
	cv::rectangle(img1, rect, Scalar(0, 255, 255));//在img1上绘制矩形rect
	x = cvRound(roiRect.x / (roi.size[1] / 3.0)) + 1;//cvRound 四舍五入计算x的坐标
	y = cvRound(roiRect.y / (roi.size[0] / 3.0)) + 1;//cvRound 四舍五入计算y的坐标
	cv::imshow("roi", roi);
	cv::imshow("img1", img1);
	channels.clear();
}


cv::Rect find_contours(cv::Mat img)//寻找图像的轮廓
{
	std::vector<std::vector<cv::Point> > contours;//vector存储图像轮廓
	std::vector<cv::Vec4i> hierarchy;//vector存储图像层次
	cv::Rect maxRect;
	cv::findContours(//findContours 函数用于检测图片轮廓
		img,
		contours,//保存轮廓的向量
		hierarchy,//保存层次的向量
		// 检测所有的轮廓，但所有轮廓只建立两个等级关系，外围为顶层，若外围内的内围轮廓还包含了
		// 其他的轮廓信息，则内围内的所有轮廓均归属于顶层
		CV_RETR_CCOMP,
		CV_CHAIN_APPROX_NONE//保存物体边界上所有连续的轮廓点到contours向量内
	);
	if (contours.size() != 0)
	{
		maxRect = cv::boundingRect(contours[0]);//用一个最小的矩形，把找到的形状包起来
	}
	return maxRect;
}



void arm_draw_triangle(ros::Publisher &pub)
{
	float points[3][2]; //确定点的个数
	points[0][0] = 250; points[0][1] = 0;
	points[1][0] = 150; points[1][1] = -80;
	points[2][0] = 150; points[2][1] = 80; //取点
	swiftpro::position pos;  //定义位置对象
	for(int i=0; i<3; ++i)
	{
		pos.x = points[i][0];
		pos.y = points[i][1];
		pub.publish(pos);  //发布位置，机械臂描点
		ros::Duration(1.5).sleep();  //休眠1.5秒
	}
}


void arm_draw_square(ros::Publisher &pub)
{
	float points[4][2];  //确定点的个数
	points[0][0] = 200; points[0][1] = 50;
	points[1][0] = 200; points[1][1] = -50;
	points[2][0] = 150; points[2][1] = -50;
	points[3][0] = 150; points[3][1] = 50;  //取点
	swiftpro::position pos;  //定义位置对象
	for(int i=0;i<4;++i)
	{
		pos.x = points[i][0];
		pos.y = points[i][1];
		pub.publish(pos);  //发布位置，机械臂描点
		ros::Duration(1.5).sleep();  //休眠1.5秒
	}
}
void arm_draw_circle(ros::Publisher& pub) //画圆
{
	float points[10][2]; //确定点的个数
	swiftpro::position pos; //定义位置对象
	for (int i = 0;i < 10;++i)
	{
		points[i][0] = 100 + i * 10;
		points[i][1] = 150 + sqrt(2500 - (points[i][0] - 150) * (points[i][0] - 150)); //取点
		pos.x = points[i][0];
		pos.y = points[i][1];
		pub.publish(pos);  //发布位置，机械臂描点
		ros::Duration(0.4).sleep(); //休眠0.4秒
	}
	for (int i = 0;i < 10;++i)
	{
		points[i][0] = 199 - i * 10;
		points[i][1] = 150 - sqrt(2500 - (points[i][0] - 150) * (points[i][0] - 150));  //取点
		pos.x = points[i][0];
		pos.y = points[i][1];
		pub.publish(pos);  //发布位置，机械臂描点
		ros::Duration(0.4).sleep();  //休眠0.4秒
	}
}
