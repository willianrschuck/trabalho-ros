#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int64.h>


#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

std::vector<cv::Mat> imagens;
ros::Publisher *pub;
ros::Rate *rate;

cv::Mat calcularHistograma(cv::Mat mat);

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	try {
		
		cv::Mat image = cv_bridge::toCvShare(msg,"bgr8")->image;
		cv::Mat hist = calcularHistograma(image);

		rate->sleep();

		unsigned int i = 0;
		for (auto hist_base : imagens) {
			double result = cv::compareHist(hist_base, hist, 1);
			if (!result) {
				std_msgs::Int64 response;
				response.data = i;
				pub->publish(response);
				std::cout << i << std::endl;
				break;
			}
			i++;
		}

	} catch(cv_bridge::Exception& e) {
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.",msg->encoding.c_str());
	}
}

cv::Mat calcularHistograma(cv::Mat mat) {

	cv::Mat hsv_mat;
	cv::cvtColor(mat, hsv_mat, cv::COLOR_BGR2HSV);

    cv::Mat hsv_half_down = hsv_mat( cv::Range( hsv_mat.rows/2, hsv_mat.rows ), cv::Range( 0, hsv_mat.cols ) );

    int histSize[] = { 50, 60 };
    float h_ranges[] = { 0, 180 };
    float s_ranges[] = { 0, 256 };

    const float* ranges[] = { h_ranges, s_ranges };

    int channels[] = { 0, 1 };
	
    cv::Mat hist_base, hist_half_down;

    calcHist( &hsv_mat, 1, channels, cv::Mat(), hist_base, 2, histSize, ranges, true, false );
    normalize( hist_base, hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

	return hist_base;

}

int main(int argc, char **argv){
	ros::init(argc,argv, "image_listener");
	ros::NodeHandle nh;

	/* THIS IS BAD AND WE NEED TO CHANGE IT */
    imagens.push_back(calcularHistograma(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/vazia.png",    cv::IMREAD_COLOR)));
    imagens.push_back(calcularHistograma(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/frente.png",   cv::IMREAD_COLOR)));
    imagens.push_back(calcularHistograma(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/esquerda.png", cv::IMREAD_COLOR)));
    imagens.push_back(calcularHistograma(cv::imread("/home/willian/catkin_ws/src/trabalho/src/images/direita.png",  cv::IMREAD_COLOR)));

	
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

	ros::Publisher thePub = nh.advertise<std_msgs::Int64>("result", 1);
	pub = &thePub;
	ros::Rate time_rate(2);
	rate = &time_rate;
	
	ros::spin();

}