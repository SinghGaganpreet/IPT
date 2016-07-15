#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include "opencv2/core/core.hpp"
#include <iostream>
#include <stdio.h>
#include <ctime>
#include "IPM.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";
static const std::string INVERSE_WINDOW = "Inverse Perspective Transform window";

class ImageConverter
{
	//---Function Decleration
	//void inversePerspectiveTransform(void);

	//---Variable Decleration
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

	//---Images
	cv::Mat inputImg, inputImgGray;
	cv::Mat outputImg;
  
public:
  
	ImageConverter()
	: it_(nh_)
	{
		//---Subscrive to input video feed and publish output video feed
		image_sub_ = it_.subscribe("/usb_cam/image_rect_color", 1, 
		&ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);

		cv::namedWindow(OPENCV_WINDOW);
		cv::namedWindow(INVERSE_WINDOW);
	}

	~ImageConverter()
	{
		//cv::destroyWindow(OPENCV_WINDOW);
		//cv::destroyWindow(INVERSE_PERSPECTIVE_WINDOW);
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		//---Draw an example circle on the video stream
		//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
		//	cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

		inputImg = cv_ptr->image;

		inversePerspectiveTransform();

		//---Update GUI Window
		//cv::imshow(OPENCV_WINDOW, cv_ptr->image);
		cv::waitKey(3);

		//---Output modified video stream
		image_pub_.publish(cv_ptr->toImageMsg());
	}	

	void inversePerspectiveTransform()
	{
		//---Get video information
		int width = 0, height = 0;
		cv::Size inputImgSize = inputImg.size();
		height = inputImgSize.height;
		width = inputImgSize.width;

		//width = static_cast<int>(video.get(CV_CAP_PROP_FRAME_WIDTH));
		//height = static_cast<int>(video.get(CV_CAP_PROP_FRAME_HEIGHT));

		//---The 4-points at the input image	
		vector<Point2f> origPoints;
		origPoints.push_back( Point2f(50, height) );
		origPoints.push_back( Point2f(width-50, height) );
		origPoints.push_back( Point2f(width/2+60, 100) ); 
		origPoints.push_back( Point2f(width/2-60, 100) );

		//---The 4-points correspondences in the destination image
		vector<Point2f> dstPoints;
		dstPoints.push_back( Point2f(100, height) );
		dstPoints.push_back( Point2f(width-100, height) );
		dstPoints.push_back( Point2f(width-100, 0) );
		dstPoints.push_back( Point2f(100, 0) );

		//---IPM object
		IPM ipm( Size(width, height), Size(width, height), origPoints, dstPoints );

		//---Color Conversion
		 if(inputImg.channels() == 3)		 
			 cvtColor(inputImg, inputImgGray, CV_BGR2GRAY);				 		 
		 else	 
			 inputImg.copyTo(inputImgGray);			 		 

		 //---Process		 
		 ipm.applyHomography( inputImg, outputImg );		 		 
		 ipm.drawPoints(origPoints, inputImg );

		 //---View		
		 cv::imshow(OPENCV_WINDOW, inputImg);
		 cv::imshow(INVERSE_WINDOW, outputImg);			
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
