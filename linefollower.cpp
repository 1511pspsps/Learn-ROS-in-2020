#include "line_follower.h"

LineFollower::LineFollower(ros::NodeHandle& nh):nh_(nh)
{
	car_tw_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

//disconsructer
LineFollower::~LineFollower()
{

}

void LineFollower::go_straight()
{
	std::cout<<"zhixian"<<std::endl;
	std::cout<<"zhixian"<<std::endl;

	geometry_msgs::Twist tw;
	tw.linear.x = 3.12;   //gai su du
	double start = ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - start) < 0.8)
	{
		car_tw_pub_.publish(tw);
		ros::spinOnce();
	}
}

void LineFollower::turn_90_degree(int direct)
{
	std::cout<<"90degree"<<std::endl;
	std::cout<<"90degree"<<std::endl;

	geometry_msgs::Twist tw;
	tw.linear.x=1.57; // jia 0.5
	tw.angular.z = 0.3 * direct;
	double start = ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - start) < 2)
	{
		car_tw_pub_.publish(tw);
		ros::spinOnce();
	}
}

void LineFollower::turn_around()
{
	std::cout<<"turn_around"<<std::endl;
	std::cout<<"turn_around"<<std::endl;


	geometry_msgs::Twist tw;
	tw.linear.x=1.57; //shan chu
	tw.angular.z = 0.3;
	double start = ros::Time::now().toSec();
	while ((ros::Time::now().toSec() - start) < 5.5)
	{
		car_tw_pub_.publish(tw);
		ros::spinOnce();
	}
}

void LineFollower::stop()
{
	std::cout<<"stop"<<std::endl;

	geometry_msgs::Twist tw;
	/*double a =0.1;
	double start = ros::Time::now().toSec();
	ros::Rate r(10);
	while((ros::Time::now().toSec-start)<0.4)
	{
		if(a>0) a=a-0.025;
		if(a<0) a=0;
		tw.linear.x = a;
		std::cout<<"speed:"<<a<<std::endl;
		car_tw_pub_.publish(tw);
		r.sleep();
	}
	*/
	tw.linear.x = 0;
	tw.angular.z = 0;
	car_tw_pub_.publish(tw);
}

Mat LineFollower::get_ROI(const Mat& orig)
{
	Mat src = orig.clone();
	//??ROI??
	Rect
	line_roi_rect((src.cols / 2) - (LINE_ROI_WIDTH / 2), src.rows - LINE_ROI_HIGH - LINE_ROI_HIGH_BIAS, LINE_ROI_WIDTH, LINE_ROI_HIGH);
	rectangle(src, line_roi_rect, cv::Scalar(0, 0, 255), 2);
	//cv::imshow("Line ROI", src);

	Mat line_roi = orig(line_roi_rect).clone();
	cvtColor(line_roi, line_roi, CV_BGR2GRAY); //????
	threshold(line_roi, line_roi, 80, 255, THRESH_BINARY_INV); //???

	//????
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
	morphologyEx(line_roi, line_roi, MORPH_OPEN, element);
	morphologyEx(line_roi, line_roi, MORPH_CLOSE, element);
	morphologyEx(line_roi, line_roi, MORPH_OPEN, element);
	//imshow("Line Control Image", line_roi);
	return line_roi;
}

bool LineFollower::follow_line_controller(const Mat& roi)
{
	std::cout<<"controller"<<std::endl;


	Mat line_roi = roi.clone();
	int line_pos = LINE_ROI_HIGH - LINE_CONTROLLER_POS;
	line(line_roi, Point(0, line_pos), Point(LINE_ROI_WIDTH, line_pos), Scalar(225), 1);

	//??????????????
	std::vector<Point> line_edge;
	int line_center;
	for (int i = 0;i < roi.cols - 1;i++)
	{
		if(((int)roi.at<uchar>(line_pos, i)) != ((int)roi.at<uchar>(line_pos, i + 1)))
			line_edge.push_back(Point(i, line_pos));
	}
	
	static int error = 0;
	if (line_edge.size() == 2)
	{
		line_center = (line_edge[0].x + line_edge[1].x) / 2;
		circle(line_roi, Point(line_center, line_pos), 5, Scalar(0), 2, 8, 0);
		//??????ROI??????????
		geometry_msgs::Twist tw;
		tw.linear.x = 0.32; //sudu 0.32
		//PD
		int derror = (roi.cols / 2 - line_center) - error;
		error = (roi.cols / 2 - line_center);
		tw.angular.z = error * CONTROLLER_KP + derror * CONTROLLER_KD;

		car_tw_pub_.publish(tw);
	}
	else if (line_edge.size() == 0)
	{
		stop();
		error = 0;
		return true;
	}
	else error = 0;
	//imshow("line",line_roi);
	return false;
}
