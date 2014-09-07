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
#include "tf2_msgs/TFMessage.h"
#include <sensor_msgs/PointCloud.h>


static const std::string OPENCV_WINDOW = "Image window";

using namespace std;
using namespace cv;
using namespace boost::accumulators;

#define NO_BALLS_STATE 0
#define BALLS_STATE 1
#define REF_RES_X 640


class ballDetector
{
  ros::NodeHandle nh_;

  
public:
  ballDetector();

  ~ballDetector()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
  
private :
  //ball detection
  static bool areThoseTwoObservationsTheSameBall(Vec3f a, Vec3f b);
  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  std::list<Vec3f> detectBalls(cv::Mat & image);
  void filterAndOrganiseBalls(list<Vec3f>);
  void updateStateMachine();
  void drawBalls(Mat &, vector<Vec3f>);

  //map filling
  void poseMessageReceived(tf2_msgs::TFMessage);


  int width_;
  int height_;

  //
  bool use_ui_;

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
  int filtering_states_average_number_;
  list<int> micro_states_;
  list<std::list<Vec3f> > ball_positions_trough_time_;
  vector <Vec3f> ballPositions2D_;

  int current_state_;
  typedef int (ballDetector::*ScriptFunction)(void); // function pointer type
  //<current state, input>-><new state, function pointer>
  std::map<std::pair<int, int>, std::pair<int,ScriptFunction> > state_machine_;
  //

  //position
  Vec3f robot_position_;
  //pcl::PointCloud<pcl::PointXYZRGB> ball_pcl_;

  //publishers/Subscriber
  ros::Publisher sound_publisher_;

  ros::Subscriber robot_position_subscriber_;
  ros::Publisher ball_pcl_publisher_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  // image_transport::Publisher image_pub_;
};

ballDetector::ballDetector(): it_(nh_)
{
	ros::param::get("~ui", use_ui_);

	ROS_INFO_STREAM("use_ui:" << use_ui_);

    // Subscrive to input video feed and publish output video feed
	image_sub_ = it_.subscribe("camera2/rgb/image_color", 1, &ballDetector::imageCb, this);

	//params
	minDist_ = 50;
	cannyThreshold_ = 40;
	votesThreshold_ = 30;
	minRadius_ = 20;
	maxRadius_ = 50;

	square_grad_sum_ = 10000;

	filtering_states_average_number_ = 10;
	///

    if(use_ui_)
    	cv::namedWindow(OPENCV_WINDOW);

    //state machine
    current_state_ = NO_BALLS_STATE;

    state_machine_[std::make_pair<int, int>(NO_BALLS_STATE,1)]
                  = std::make_pair<int, ScriptFunction>(BALLS_STATE, &ballDetector::newBallsDetected);
    state_machine_[std::make_pair<int, int>(BALLS_STATE,0)]
                  = std::make_pair<int, ScriptFunction>(NO_BALLS_STATE, &ballDetector::ballLost);
    state_machine_[std::make_pair<int, int>(BALLS_STATE,1)]
                  = std::make_pair<int, ScriptFunction>(BALLS_STATE, &ballDetector::nop);
    state_machine_[std::make_pair<int, int>(NO_BALLS_STATE,0)]
                  = std::make_pair<int, ScriptFunction>(NO_BALLS_STATE, &ballDetector::nop);



    // publishers/subscribers
    robot_position_subscriber_ = nh_.subscribe("tf", 100, &ballDetector::poseMessageReceived, this);
    sound_publisher_  = nh_.advertise<kobuki_msgs::Sound> ("/mobile_base/commands/sound", 10);
    //map__subscriber_ = ;
    ball_pcl_publisher_= nh_.advertise<sensor_msgs::PointCloud>("/webcam_ball", 50);
   // ball_pcl_publisher_= nh_.advertise<sensor_msgs::PointCloud>("mobile_base/sensors/bumper_pointcloud", 50);
}

//map filling
void ballDetector::poseMessageReceived(tf2_msgs::TFMessage msg)
{

	std::string frame_id = msg.transforms[0].header.frame_id;
	std::string child_frame_id = msg.transforms[0].child_frame_id;

	if(frame_id=="odom")
	{

		float x = msg.transforms[0].transform.translation.x;
		float y = msg.transforms[0].transform.translation.y;

		float rz = msg.transforms[0].transform.rotation.z;

		robot_position_[0] = msg.transforms[0].transform.translation.x;
		robot_position_[1] = msg.transforms[0].transform.translation.y;
		robot_position_[2] = msg.transforms[0].transform.rotation.z;

		//ROS_INFO_STREAM("msg.header.frame_id="<<);
		ROS_INFO_STREAM(frame_id<<"/"<<child_frame_id);
		ROS_INFO_STREAM("position="<<x<<" "<<y<<" angle="<<rz*180/3.1415);
	}


}

//ball detection

int ballDetector::newBallsDetected()
{
	ROS_INFO_STREAM("new balls detected");
	kobuki_msgs::Sound msg;
	msg.value = 2;
	sound_publisher_.publish(msg);
	////
	unsigned int num_points = 100;
	sensor_msgs::PointCloud cloud;
	cloud.header.stamp = ros::Time::now();
	cloud.header.frame_id = "odom";

	cloud.points.resize(num_points);

	//we'll also add an intensity channel to the cloud
	cloud.channels.resize(1);
	cloud.channels[0].name = "intensities";
	cloud.channels[0].values.resize(num_points);

	//generate some fake data for our point cloud


	int n_points = 0;
	for(unsigned int i = 0; i < 10; ++i)
	{
		for(unsigned int j = 0; j < 10; ++j)
		{
			cloud.points[n_points].x = robot_position_[0] + i/100.0 + 20.0/100;
			cloud.points[n_points].y = robot_position_[1] + j/100.0 - 5.0/100;
			cloud.points[n_points].z = 0.02;
			cloud.channels[0].values[n_points] = 100;

			n_points++;
		}
	}
	ROS_INFO_STREAM("new balls detected, publish cloud");
	ball_pcl_publisher_.publish(cloud);
	////
	return 0;
}

int ballDetector::ballLost()
{
	ROS_INFO_STREAM("balls lost");
	kobuki_msgs::Sound msg;
	msg.value = 3;
	sound_publisher_.publish(msg);
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

  // detect balls
  list<Vec3f> detected_balls = detectBalls(cv_ptr->image);

  filterAndOrganiseBalls(detected_balls);

  ROS_INFO_STREAM("number of balls detected:"<<ballPositions2D_.size());

  updateStateMachine();

  if(use_ui_)
	  drawBalls(cv_ptr->image, this->ballPositions2D_);

}

void ballDetector::drawBalls(Mat & image, vector<Vec3f> balls)
{

	//Canny(filtered_image, display_mat, cannyThreshold_/2, cannyThreshold_);

	for( size_t i = 0; i < balls.size(); i++ )
	{
		Vec3i c = balls[i];
		circle( image, Point(c[0], c[1]), c[2], Scalar(255,255,255), 1, CV_AA);
		circle( image, Point(c[0], c[1]), 2, Scalar(255,255,255), 1, CV_AA);

		//CvFont font;
		//addText( image, std::string("ee"), Point(c[0], c[1]), font);
	}

	cv::imshow(OPENCV_WINDOW, image);
	cv::waitKey(3);
}

bool ballDetector::areThoseTwoObservationsTheSameBall(Vec3f a, Vec3f b)
{
	int max_dist_x=20;
	int max_dist_y=20;
	int max_r_diff=10;

	if(abs(a[0] - b[0]) > max_dist_x
		||
		abs(a[1] - b[1]) > max_dist_y
		||
		abs(a[2] - b[2]) > max_r_diff
	)
		return false;
	else
		return true;
}

void ballDetector::filterAndOrganiseBalls(list<Vec3f> detected_balls)
{
	ballPositions2D_.clear();

	ball_positions_trough_time_.push_front(detected_balls);

	if(ball_positions_trough_time_.size()>filtering_states_average_number_)
		ball_positions_trough_time_.pop_back();

	vector<Vec3f> balls_observations;
	vector<vector<Vec3f> > classified_balls_observations;

	list<list<Vec3f> >::iterator it_l;

	//concatenate the observation list
	for(it_l=ball_positions_trough_time_.begin(); it_l!=ball_positions_trough_time_.end(); it_l++)
		copy((*it_l).begin(), (*it_l).end(), back_inserter(balls_observations));

	//partition the observations
	vector<int> labels;

	partition(balls_observations, labels, &ballDetector::areThoseTwoObservationsTheSameBall);


	vector<int>::iterator it, max_it;
	max_it = max_element(labels.begin(), labels.end());

	if(labels.size()>0)
	{
		classified_balls_observations = vector<vector<Vec3f> >(*max_it + 1);

		for(int i=0; i<classified_balls_observations.size(); i++)
		{
			classified_balls_observations[labels[i]].push_back(balls_observations[i]);
		}
	}
	//ROS_INFO_STREAM("classified_balls_observations:"<<classified_balls_observations.size());

	//compute the average position of of each ball
	vector<vector<Vec3f> >::iterator ball_it;
	vector<Vec3f>::iterator ball_observation_it;


	for(ball_it=classified_balls_observations.begin(); ball_it!=classified_balls_observations.end(); ball_it++)
	{
		if((*ball_it).size()>0)
		{
			Vec3f average_ball;

			for(ball_observation_it=(*ball_it).begin(); ball_observation_it!=(*ball_it).end(); ball_observation_it++)
			{
				average_ball += *ball_observation_it;
			}

			average_ball *= 1.0 / ((*ball_it).size());

		ballPositions2D_.push_back(average_ball);
		}
	}


	//filtering
	if(detected_balls.size()>0)
	{
		micro_states_.push_front(1);
	}
	else
	{
		micro_states_.push_front(0);
	}

	if(micro_states_.size()>filtering_states_average_number_)
	{
		micro_states_.pop_back();
	}

}

void ballDetector::updateStateMachine()
{
	 int machine_event;

	  std::list<int>::iterator l_it;
	  float average = 0;
	  for(l_it = micro_states_.begin(); l_it!=micro_states_.end(); l_it++)
	  {
		  average +=*l_it;
		  //ROS_INFO_STREAM("micro_states_:"<<*l_it);
	  }
	  average/=micro_states_.size();

	  if(average>0.5)
		  machine_event = 1;
	  else
		  machine_event = 0;

	  // state machine
	  std::pair<int,ScriptFunction> next_action;
	  next_action = state_machine_[std::pair<int, int>(current_state_, machine_event)];

	  current_state_ = next_action.first;
	  (this->*next_action.second)();

}

list<Vec3f> ballDetector::detectBalls(cv::Mat & image)
{
	  list<Vec3f> ret_circles;
	  
	  
	  width_ = image.size[1];
	  height_ = image.size[0];

	  Mat converted_img, filtered_image;

	  cv::cvtColor(image, converted_img, COLOR_BGR2GRAY);
	  medianBlur(converted_img, filtered_image, 5);
	  //bilateralFilter(converted_img, filtered_image, 4, 80, 80 );
	  
	  vector<Vec3f> circles;

	  
	  float ratio = width_/REF_RES_X;

	  HoughCircles(filtered_image, circles, CV_HOUGH_GRADIENT, 1,
			  minDist_* ratio, cannyThreshold_, votesThreshold_ * ratio, minRadius_* ratio, maxRadius_* ratio);
	                                  
	                   
	  
	  //ROS_INFO_STREAM("nb of potential balls detected :"<<circles.size());
	  //test the inner sum of gradient
	  for( size_t i = 0; i < circles.size(); i++ )
	  {
		  Vec3i c = circles[i];
		  float radius = c[2];
		  Point center = Point(c[0], c[1]);
		  
		  //ROS_INFO_STREAM("x="<<center.x<<" y="<<center.y<<" radius="<<radius);
		  
		  if((center.x-radius>=0) && (center.x+radius<width_) && (center.y-radius>=0) && (center.y+radius<height_))
		  {
			  Mat sub_img_gray = filtered_image(cv::Rect(center.x-radius,center.y-radius,2*radius,2*radius));
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



			  float variance_f = variance(acc_variance);
			  float mean_f = boost::accumulators::mean(acc_variance);
			  //ROS_INFO_STREAM("std dev :"<<sqrt(variance_f));
			  //ROS_INFO_STREAM("mean :"<<mean_f);
			  //ROS_INFO_STREAM("i:"<<i<<" grad sum :"<<sum/nb);
			  
			  if(sum/nb < square_grad_sum_)
			  {
				  ret_circles.push_back(circles[i]);  
			  }
		  } 
	  }

	  return ret_circles;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_detector");
  ballDetector bd;
  ros::spin();
  return 0;
}
