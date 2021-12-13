#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <vmxpi_ros_cam/OpenCVSettingsConfig.h>

#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <cmath>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher rgb_pub;
	ros::Publisher lmotor_speed, rmotor_speed, bmotor_speed;
	ros::ServiceClient stop_motors_client;
	int iLowH = 0, iHighH = 179, iLowS = 0, iHighS = 255, iLowV = 0, iHighV = 255;
	double leftSpeed = 0.0, rightSpeed = 0.0, backSpeed = 0.0, factor = 0.5;
	bool tracking_on;
	bool arrived, found = false;
	const double TOL = 0.1; // tolerance levels

public:
	ImageConverter()
		: it_(nh_)
	{
		// Subscribe to input video feed and publish output video feed
		image_sub_ = it_.subscribe("camera/image", 1,
		&ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("image_converter/output_video", 10);
		
		rgb_pub = nh_.advertise<std_msgs::Float32MultiArray>("camera/rgb", 10);
		
		lmotor_speed = nh_.advertise<std_msgs::Float32>("auto_motor_left", 1);
		rmotor_speed = nh_.advertise<std_msgs::Float32>("auto_motor_right", 1);
		bmotor_speed = nh_.advertise<std_msgs::Float32>("auto_motor_back", 1);
		
		stop_motors_client = nh_.serviceClient<std_srvs::Trigger>("titan/stop_motors");
		
		cv::namedWindow(OPENCV_WINDOW);
	}

	~ImageConverter()
	{
		cv::destroyWindow(OPENCV_WINDOW);
	}

	void cfgCb(vmxpi_ros_cam::OpenCVSettingsConfig &config, uint32_t level) {
		iLowH = config.iLowH;
		iHighH = config.iHighH;
		iLowS = config.iLowS;
		iHighS = config.iHighS;
		iLowV = config.iLowV;
		iHighV = config.iHighV;
	
		tracking_on = config.tracking_on;
	}

	void holonomicDrive(double x, double y, double z)
	{
		rightSpeed = (x / 2) + (-(y * (sqrt(3) / 2))) + z;
        leftSpeed = (x / 2) + (y * sqrt(3) / 2) + z;
        backSpeed = -x / 1.05 + z;
        
        double max = abs(rightSpeed);
        if (abs(leftSpeed) > max)
        {
            max = abs(leftSpeed);
        }
        if (abs(backSpeed) > max)
        {
            max = abs(backSpeed);
        }
        if (max > 1)
        {
            rightSpeed /= max;
            leftSpeed /= max;
            backSpeed /= max;
        }
	}
	
	void publishMotors()
	{
		std_msgs::Float32 msg;
		msg.data = leftSpeed * factor;
		lmotor_speed.publish(msg);
		
		msg.data = rightSpeed * factor;
		rmotor_speed.publish(msg);	
			
		msg.data = backSpeed * factor;
		bmotor_speed.publish(msg);
	}
	
	
	void resetMotors()
	{
		rightSpeed = 0.0;
		leftSpeed = 0.0;
		backSpeed = 0.0;
		publishMotors();
	}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr, threshold;
		try
		{
		  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		  threshold = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("cv_bridge exception: %s", e.what());
		  return;
		}
	
		cv:cvtColor(cv_ptr->image, threshold->image, cv::COLOR_BGR2HSV);
		cv::inRange(threshold->image, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), threshold->image);
		
		//Uncomment Below to flip output image
		//flip(threshold->image, threshold->image, -1);
		cv::imshow(OPENCV_WINDOW, threshold->image);
		cv::waitKey(30);
		
		// Morphological closing
		cv::erode(threshold->image, threshold->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::dilate(threshold->image, threshold->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		
		cv::dilate(threshold->image, threshold->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		cv::erode(threshold->image, threshold->image, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		
		if (tracking_on) {
		    cv::Rect border = cv::boundingRect(threshold->image);
		    cv::rectangle(cv_ptr->image, border, cv::Scalar(0,255,0));
	
		    ros::Rate loop_rate(50);
		    while (tracking_on)
		    {
				resetMotors();			
				//std::string coordinates = "Coordinates: (" + std::to_string(int(location_x)) + ", " + std::to_string(int(location_y)) + ")";
				//cv::putText(cv_ptr->image, coordinates, cv::Point(15, 30), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,255,0), 1, cv::LINE_8);
		
				//Processing Image Feed
				double dist = round(2096.7 * pow(border.width, -0.884)); // Power Function: 5456.7x^(-1.09) or 2096.7x^(-0.884) the slope of the pix/distance relationship
				std::string distance = "Distance: " + std::to_string(int(dist)) + "cm";
				cv::putText(cv_ptr->image, distance, cv::Point(350, 470), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,255,0), 1, cv::LINE_8);
		
				if(found && !arrived)
				{
					if (border.x < 100)
					{
						holonomicDrive(-0.8, 0.0, 0.0);
						std::string searching = "GOING LEFT...";
						cv::putText(cv_ptr->image, searching, cv::Point(175, 135), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,255,0), 2, cv::LINE_8);
						publishMotors();
					}
					else if (border.x > 420)
					{
						holonomicDrive(0.8, 0.0, 0.0);
						std::string searching = "GOING RIGHT...";
						cv::putText(cv_ptr->image, searching, cv::Point(175, 435), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,255,0), 2, cv::LINE_8);
						publishMotors();
					}
					else
					{
						holonomicDrive(0.0, 0.5, 0.0);
						std::string searching = "GOING FORWARD...";
						cv::putText(cv_ptr->image, searching, cv::Point(175, 235), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,255,0), 2, cv::LINE_8);
						publishMotors();
						
					}
					
					if (dist <= 35)
					{
						resetMotors();
						arrived = true;
						std_srvs::Trigger msg;
						stop_motors_client.call(msg);
					}
					image_pub_.publish(cv_ptr->toImageMsg());
				}
				else if(!found) //Searching state to rotate about z-axis to search for cube
				{
					holonomicDrive(0.0, 0.0, 0.3);
					std::string searching = "SEARCHING...";
					cv::putText(cv_ptr->image, searching, cv::Point(175, 235), cv::FONT_HERSHEY_COMPLEX, 1.0, cv::Scalar(0,255,0), 2, cv::LINE_8);
					publishMotors();

					if(border.area() > 20)
					{
						found = true;
					}
					image_pub_.publish(cv_ptr->toImageMsg());
				}
					
				// Output modified video stream
				image_pub_.publish(cv_ptr->toImageMsg());
				ros::spinOnce();
				loop_rate.sleep();
		    }
		}
		
		image_pub_.publish(cv_ptr->toImageMsg());
	}
};

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	dynamic_reconfigure::Server<vmxpi_ros_cam::OpenCVSettingsConfig> server;
	dynamic_reconfigure::Server<vmxpi_ros_cam::OpenCVSettingsConfig>::CallbackType f;
	f = boost::bind(&ImageConverter::cfgCb, &ic, _1, _2);
	server.setCallback(f);
	ros::spin();
	return 0;
}
