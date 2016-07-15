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
#include <math.h>

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
	cv::Mat biggerOutputImg;
	
	double degree; //--Angle at which camera is mounted
	int counter;
  
public:
	ImageConverter()
	: it_(nh_)
	{
		biggerOutputImg.create(1000,1000, CV_8UC3);
		ROS_INFO("---Constructor started");
		degree=0;counter=0;
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

		inputImg = cv_ptr->image;
		inversePerspectiveTransform();

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
		float Q = 0;	//--Angle in radians
		/*
		if (counter != 20)
		{
			counter++;
			return;
		}
		else if (counter == 20)
		{
			degree += 1;			
			Q = (M_PI * degree)/180;
			counter = 0;
			//if (degree >= -8)
			//	degree = -18;
		}*/
		degree = 50; //32..16
		Q = (M_PI * degree)/180;
		ROS_INFO(" ----Angle in Degrees %lf ", degree);
		ROS_INFO(" ----Angle in Radians %lf ", Q);

		//width = static_cast<int>(video.get(CV_CAP_PROP_FRAME_WIDTH));
		//height = static_cast<int>(video.get(CV_CAP_PROP_FRAME_HEIGHT));

		//---The 4-points at the input image	
		vector<Point2f> origPoints;

		//---The 4-points correspondences in the destination image
		vector<Point2f> dstPoints;

		//ROS_INFO("---- Width %d, height %d", width, height);

		int ui=0, vi=0, xi=0, yi=0; //---Q is angle of camera with respect to z of world frame (towards front)

		//---s is size of pixel in mm, from http://www.arducam.com/camera-modules/0-3mp-ov7725/
		//---pixel is 6um = 0.006mm and size should be 0.006*0.006 = 0.000036
		double s = 0.000036;

		//---f is focal lenght in mm, which is 6mm for ELP camera		
		double f =  0.05; //0.006; //1113.26215; //6; //280.5419;  //6;					

		//---H is the height of camera
		int H = 0.7;

		//---d as constant which is |H(sinQ - cosQ)/(fsinQ - cosQ)|
		double d = abs( H * ( (sin(Q) + (f * cos(Q)) ) / ( (f * sin(Q)) - cos(Q) ) ) 	) + 1;
		//ROS_INFO("---------constant d = %lf", d);			
			
		char IPT = 'A';
		
		//------------Transforming First Point--------//
		yi = 300; xi = 200; //height;
		origPoints.push_back( Point2f(yi, xi) );
		if (IPT == 'A')		//--Transform from top view to some angle
		{
			ROS_INFO("------------Transforming from Top to angle %lf", Q);
			ui = (xi + ((f/s) * tan(Q))) / (1 - ((s/f) * xi * tan(Q)));
			vi = yi * (((s/f) * xi * sin(Q)) + cos(Q));
		}
		else if (IPT == 'N') 	//--New IPT
		{
			ROS_INFO("------------Transforming using New Transform equations");
			ui = (H * ( ( (xi * sin(Q)) + (f * cos(Q) ) ) / ( (-yi * cos(Q)) + (f * sin(Q) ) ) ) ) + d;
			vi = (H * ( ( (yi * sin(Q)) + (f * cos(Q) ) ) / ( (-yi * cos(Q)) + (f * sin(Q) ) ) ) ) + d;			
		}
		else if (IPT == 'O')	//--Old IPT
		{
			ROS_INFO("------------Transforming using Old Transform equations");
			ui = (xi - ((f/s) * tan(Q))) / (1 + ((s/f) * xi * tan(Q)));
			vi = yi / ( ( (s/f) * xi * sin(Q) ) + cos(Q) );
		}
		else if (IPT == 'M')  //--My IPT
		{
			ROS_INFO("------------Transforming using Gagan's Transform equations");					
			ui = ((xi * s * cos(Q) ) + (f * sin(Q))) / (cos(Q) - (xi * ((s*s)/f) * sin(Q)));
			vi = yi / ( cos(Q) - ( (s/f) * xi * sin(Q) ));	
		}
		else if (IPT == 'G')  //--My New IPT Equations
		{
			ROS_INFO("------------Transforming using Gagan's New Transform equations");					
			ui = (f/s) * ( ( (xi * cos(Q)) - ( (f/s) * sin(Q) ) ) / ( (-xi * sin(Q)) * ((f/s) * cos(Q)) ) );
			vi = ((f/s) * yi) / ( (-xi * sin(Q)) + ((f/s) * cos(Q)));
		}
		dstPoints.push_back( Point2f(vi, ui) );
		ROS_INFO("------- 1 points xi=%d, yi=%d ui= %d, vi= %d", xi, yi, ui, vi);

		//------------Transforming Second Point--------//
		yi = width-300; xi = 200;//height;
		origPoints.push_back( Point2f(yi, xi) );
		if (IPT == 'A')		//--Transform from top view to some angle
		{
			ui = (xi + ((f/s) * tan(Q))) / (1 - ((s/f) * xi * tan(Q)));
			vi = yi * (((s/f) * xi * sin(Q)) + cos(Q));
		}
		else if (IPT == 'N') 	//--New IPT
		{
			ui = (H * ( ( (xi * sin(Q)) + (f * cos(Q) ) ) / ( (-yi * cos(Q)) + (f * sin(Q) ) ) ) ) + d;
			vi = (H * ( ( (yi * sin(Q)) + (f * cos(Q) ) ) / ( (-yi * cos(Q)) + (f * sin(Q) ) ) ) ) + d;			
		}
		else if (IPT == 'O')	//--Old IPT
		{
			ui = (xi - ((f/s) * tan(Q))) / (1 + ((s/f) * xi * tan(Q)));
			vi = yi / ( ( (s/f) * xi * sin(Q) ) + cos(Q) );
		}
		else if (IPT == 'M')  //--My IPT
		{			
			ui = ((xi * s * cos(Q) ) + (f * sin(Q))) / (cos(Q) - (xi * ((s*s)/f) * sin(Q)));
			vi = yi / ( cos(Q) - ( (s/f) * xi * sin(Q) ));	
		}
		else if (IPT == 'G')  //--My New IPT Equations
		{			
			ui = (f/s) * ( ( (xi * cos(Q)) - ( (f/s) * sin(Q) ) ) / ( (-xi * sin(Q)) * ((f/s) * cos(Q)) ) );
			vi = ((f/s) * yi) / ( (-xi * sin(Q)) + ((f/s) * cos(Q)));
		}
		dstPoints.push_back( Point2f(vi, ui) );
		ROS_INFO("------- 2 points xi=%d, yi=%d ui= %d, vi= %d", xi, yi, ui, vi);

		//------------Transforming Third Point--------//
		yi = 300;//width/2 + 50; 
		xi = 100;
		origPoints.push_back( Point2f(yi, xi) );
		if (IPT == 'A')		//--Transform from top view to some angle
		{
			ui = (xi + ((f/s) * tan(Q))) / (1 - ((s/f) * xi * tan(Q)));
			vi = yi * (((s/f) * xi * sin(Q)) + cos(Q));
		}
		else if (IPT == 'N') 	//--New IPT
		{
			ui = (H * ( ( (xi * sin(Q)) + (f * cos(Q) ) ) / ( (-yi * cos(Q)) + (f * sin(Q) ) ) ) ) + d;
			vi = (H * ( ( (yi * sin(Q)) + (f * cos(Q) ) ) / ( (-yi * cos(Q)) + (f * sin(Q) ) ) ) ) + d;			
		}
		else if (IPT == 'O')	//--Old IPT
		{
			ui = (xi - ((f/s) * tan(Q))) / (1 + ((s/f) * xi * tan(Q)));
			vi = yi / ( ( (s/f) * xi * sin(Q) ) + cos(Q) );
		}
		else if (IPT == 'M')  //--My IPT
		{			
			ui = ((xi * s * cos(Q) ) + (f * sin(Q))) / (cos(Q) - (xi * ((s*s)/f) * sin(Q)));
			vi = yi / ( cos(Q) - ( (s/f) * xi * sin(Q) ));	
		}
		else if (IPT == 'G')  //--My New IPT Equations
		{			
			ui = (f/s) * ( ( (xi * cos(Q)) - ( (f/s) * sin(Q) ) ) / ( (-xi * sin(Q)) * ((f/s) * cos(Q)) ) );
			vi = ((f/s) * yi) / ( (-xi * sin(Q)) + ((f/s) * cos(Q)));
		}
		dstPoints.push_back( Point2f(vi, ui) );
		//cv::circle(inputImg, cv::Point(yi, xi), 10, CV_RGB(255,0,0));
		ROS_INFO("------- 3 points xi=%d, yi=%d ui= %d, vi= %d", xi, yi, ui, vi);

		//------------Transforming Fourth Point--------//
		yi = width-300;///2 - 50; 
		xi = 100;
		origPoints.push_back( Point2f(yi, xi) );
		if (IPT == 'A')		//--Transform from top view to some angle
		{
			ui = (xi + ((f/s) * tan(Q))) / (1 - ((s/f) * xi * tan(Q)));
			vi = yi * (((s/f) * xi * sin(Q)) + cos(Q));
		}
		else if (IPT == 'N') 	//--New IPT
		{
			ui = (H * ( ( (xi * sin(Q)) + (f * cos(Q) ) ) / ( (-yi * cos(Q)) + (f * sin(Q) ) ) ) ) + d;
			vi = (H * ( ( (yi * sin(Q)) + (f * cos(Q) ) ) / ( (-yi * cos(Q)) + (f * sin(Q) ) ) ) ) + d;			
		}
		else if (IPT == 'O')	//--Old IPT
		{
			ui = (xi - ((f/s) * tan(Q))) / (1 + ((s/f) * xi * tan(Q)));
			vi = yi / ( ( (s/f) * xi * sin(Q) ) + cos(Q) );
		}
		else if (IPT == 'M')  //--My IPT
		{			
			ui = ((xi * s * cos(Q) ) + (f * sin(Q))) / (cos(Q) - (xi * ((s*s)/f) * sin(Q)));
			vi = yi / ( cos(Q) - ( (s/f) * xi * sin(Q) ));	
		}
		else if (IPT == 'G')  //--My New IPT Equations
		{			
			ui = (f/s) * ( ( (xi * cos(Q)) - ( (f/s) * sin(Q) ) ) / ( (-xi * sin(Q)) * ((f/s) * cos(Q)) ) );
			vi = ((f/s) * yi) / ( (-xi * sin(Q)) + ((f/s) * cos(Q)));
		}
		dstPoints.push_back( Point2f(vi, ui) );
		//cv::circle(inputImg, cv::Point(yi, xi), 10, CV_RGB(0,255,0));
		ROS_INFO("------- 4 points xi=%d, yi=%d ui= %d, vi= %d", xi, yi, ui, vi);

		//---IPM object
		//IPM ipm( Size(width, height), Size(width, height), origPoints, dstPoints );
		//ipm.drawPoints(origPoints, inputImg );
		//rectangle(inputImg, origPoints[0], origPoints[2], (0,0,255), 1);
		//rectangle(inputImg, dstPoints[0], dstPoints[2], (255,0,0), 1);

//		Mat D (A, Rect(10, 10, 100, 100) );
		// Calculate Homography
		Mat h = findHomography(origPoints, dstPoints);
		// Warp source image to destination based on homography
//		warpPerspective(inputImg, outputImg, h, outputImg.size());
		warpPerspective(inputImg, biggerOutputImg, h, biggerOutputImg.size());
		resize(biggerOutputImg, outputImg, outputImg.size(), 0.5, 0.5, INTER_LINEAR);
		 //---View		
		cv::imshow(OPENCV_WINDOW, inputImg);
		cv::imshow(INVERSE_WINDOW, outputImg);			
//		cv:imshow("Bigger Image", biggerOutputImg);
		
	}

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}
