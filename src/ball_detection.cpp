#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/moment.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <kobuki_msgs/Sound.h>

static const std::string OPENCV_WINDOW = "Image window";

using namespace cv;
using namespace boost::accumulators;

#define NO_BALLS 0
#define BALLS 1



class ballDetector
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
  ballDetector();

  ~ballDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
private :
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  vector<Vec3f> detectBalls(cv::Mat & image);

  //void state machine
  int newBallsDetected();
  int ballLost();
  int nop();

  //params hough circles
  int minDist_;
  int cannyThreshold_;
  int votesThreshold_;
  int minRadius_;
  int maxRadius_;
  //params   
  int square_grad_sum_;
  

  //state machine
  int current_state_;
  typedef int (ballDetector::*ScriptFunction)(void); // function pointer type
  //<current state, input>-><new state, function pointer>
  std::map<std::pair<int, int>, std::pair<int,ScriptFunction> > state_machine_;
  //

  //publishers
  ros::Publisher sound_publisher_;
};

ballDetector::ballDetector(): it_(nh_)
{
    // Subscrive to input video feed and publish output video feed

	image_sub_ = it_.subscribe("camera2/rgb/image_color", 1, &ballDetector::imageCb, this);

	//params
	minDist_ = 50;
	cannyThreshold_ = 40;
	votesThreshold_ = 25;
	minRadius_ = 20;
	maxRadius_ = 50;

	square_grad_sum_ = 1500;

   // cv::namedWindow(OPENCV_WINDOW);

    //state machine
    current_state_ = NO_BALLS;

    state_machine_[std::make_pair<int, int>(NO_BALLS,1)]
                  = std::make_pair<int, ScriptFunction>(BALLS, &ballDetector::newBallsDetected);
    state_machine_[std::make_pair<int, int>(BALLS,0)]
                  = std::make_pair<int, ScriptFunction>(NO_BALLS, &ballDetector::ballLost);
    state_machine_[std::make_pair<int, int>(BALLS,1)]
                  = std::make_pair<int, ScriptFunction>(BALLS, &ballDetector::nop);
    state_machine_[std::make_pair<int, int>(NO_BALLS,0)]
                  = std::make_pair<int, ScriptFunction>(NO_BALLS, &ballDetector::nop);

    // publishers
    sound_publisher_ = nh_.advertise<kobuki_msgs::Sound> ("/mobile_base/commands/sound", 1);
}

int ballDetector::newBallsDetected()
{
	ROS_INFO_STREAM("new balls detected");
	kobuki_msgs::Sound msg;
	msg.value = 2;
	sound_publisher_.publish(msg);
	return 0;
}

int ballDetector::ballLost()
{
	ROS_INFO_STREAM("balls lost");
	return 0;
}

int ballDetector::nop()
{
	return 0;
}

void ballDetector::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  	cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window

  vector<Vec3f> detected_balls = detectBalls(cv_ptr->image);

  // state machine
  int machine_event;
  if(detected_balls.size()>0)
	  machine_event = 1;
  else
	  machine_event = 0;

  //
  std::pair<int,ScriptFunction> next_action;
  next_action = state_machine_[std::pair<int, int>(current_state_, machine_event)];

  current_state_ = next_action.first;
  (this->*next_action.second)();

  // Output modified video stream
  //image_pub_.publish(cv_ptr->toImageMsg());
}

vector<Vec3f> ballDetector::detectBalls(cv::Mat & image)
{
	  vector<Vec3f> ret_circles;
	  
	  int width, height;
	  width = image.size[1];
	  height = image.size[0];
	  
	  Mat converted_img;
	  medianBlur(image, image, 5);
	  cv::cvtColor(image, converted_img, COLOR_BGR2GRAY);
	  
	  vector<Vec3f> circles;

	  
	  HoughCircles(converted_img, circles, CV_HOUGH_GRADIENT, 1,
			  minDist_, cannyThreshold_, votesThreshold_, minRadius_, maxRadius_);
	                                  
	                   
	  
	  //ROS_INFO_STREAM("nb of potential balls detected :"<<circles.size());
	  //test the inner sum of gradient
	  for( size_t i = 0; i < circles.size(); i++ )
	  {
		  Vec3i c = circles[i];
		  float radius = c[2];
		  Point center = Point(c[0], c[1]);
		  
		  //ROS_INFO_STREAM("x="<<center.x<<" y="<<center.y<<" radius="<<radius);
		  
		  if((center.x-radius>=0) && (center.x+radius<width) && (center.y-radius>=0) && (center.y+radius<height))
		  {
			  Mat sub_img_gray = converted_img(cv::Rect(center.x-radius,center.y-radius,2*radius,2*radius));
			  Mat sub_img_gray_float;
			  
			  
			  sub_img_gray.convertTo(sub_img_gray_float, CV_32F);
			  		  
			  
			  Mat grad_x_img, grad_y_img, grad_xy_img;
			  
			  Sobel(sub_img_gray_float, grad_x_img, sub_img_gray_float.depth(), 1, 0, 3);
			  Sobel(sub_img_gray_float, grad_y_img, sub_img_gray_float.depth(), 0, 1, 3);
			  
			  
			  grad_xy_img = abs(grad_x_img) + abs(grad_y_img);
			  

			  
			  float* grad_ptr = (float*)grad_xy_img.data;
			  char* img_ter = (char*)sub_img_gray.data;
			  
			  int cn = grad_xy_img.channels();
			  
			  float sum = 0;
			  int nb = 0;
			  
			  accumulator_set<float, stats<tag::variance, tag::mean> > acc_variance;
			  
			  //
			  float small_radius = 0.8*radius;
			  //

			  for (int x=-small_radius; x<small_radius; x++)
			  {
				  int y_max = sqrt(small_radius*small_radius - x*x);
				  int y_min = -sqrt(small_radius*small_radius - x*x);
				  
				  for (int y=y_min; y<y_max; y++)
				  { 
					 int X = radius + x;
					 int Y = radius + y;
					 
					 float grad_f = grad_ptr[Y*grad_xy_img.cols*cn + X*cn];
					 sum += grad_f * grad_f;
					 
					 char gray_val = img_ter[Y*grad_xy_img.cols*cn + X*cn];
					 acc_variance(gray_val);
					 
					 nb++;
				  } 

			  }
			  
			  circle( grad_xy_img, Point(radius, radius), radius, Scalar(255,255,255), 1, CV_AA);

			  //cv::imshow(OPENCV_WINDOW, grad_xy_img/255);


			  float variance_f = variance(acc_variance);
			  float mean_f = boost::accumulators::mean(acc_variance);
			  //ROS_INFO_STREAM("std dev :"<<sqrt(variance_f));
			  //ROS_INFO_STREAM("mean :"<<mean_f);
			  ROS_INFO_STREAM("i:"<<i<<" grad sum :"<<sum/nb);
			  
			  if(sum/nb < square_grad_sum_)
			  {
				  ret_circles.push_back(circles[i]);  
			  }
		  } 
	  }
	  
	  //
	  for( size_t i = 0; i < ret_circles.size(); i++ )
	  {
	  	       Vec3i c = circles[i];
	  	       circle( image, Point(c[0], c[1]), c[2], Scalar(0,0,255), 1, CV_AA);
	  	       circle( image, Point(c[0], c[1]), 2, Scalar(0,255,0), 1, CV_AA);
	  }
	  //visu
	//  cv::imshow(OPENCV_WINDOW, image);
	//  cv::waitKey(3);
	      
	  return ret_circles;

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_detector");
  ballDetector bd;
  ros::spin();
  return 0;
}
