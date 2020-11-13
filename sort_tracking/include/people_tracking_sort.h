
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <people_msgs/Person.h>
#include <people_msgs/People.h>
#include <image_transport/image_transport.h>

#include <image_transport/subscriber_filter.h>
#include "image_geometry/stereo_camera_model.h"

#include "cv_bridge/cv_bridge.h"
#include <visualization_msgs/Marker.h>

#include<unordered_map>
#include <opencv2/dnn.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>
#include<KalmanTracker3d.h>

#include<Sort.h>


using namespace std;
using namespace cv;
using namespace cv::dnn;

namespace SortTracking
{


	class SortTracking
	{
		public:
			ros::NodeHandle nh_;
			string image_topic_name;
			string depth_topic_name;
			string camera_name;
			string camera_info;
			string camera_rgb_info_topic;
			string camera_depth_info_topic;
			string rgb_ns;
			string depth_ns;
			string image_topic;
			string depth_topic;
			image_geometry::StereoCameraModel cam_model_; 
			image_transport::ImageTransport it_;
			image_transport::SubscriberFilter image_sub_;
			image_transport::SubscriberFilter depth_image_sub_;

			double current_time;
			// ros::Time prev_time;
			

			// ros::Time curr_disp;
			// ros::Time prev_disp;
			

			ros::Publisher people_tracker_measurements_pub;

			ros::Publisher marker_pub;
			image_transport::Publisher image_test_pub;

			string model_txt_;
			string model_bin_;
			ros::Rate rate;

			Net net;

			unordered_map<string,KalmanTracker3d> trackers3d;
			// unordered_map<string,int> trackers3d;
			float delta_t;

			message_filters::Subscriber<sensor_msgs::CameraInfo> c1_info_sub_;
			message_filters::Subscriber<sensor_msgs::CameraInfo> c2_info_sub_;
			typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateDepthPolicy; /**< Sync policy for approx time with depth. */
			typedef message_filters::Synchronizer<ApproximateDepthPolicy> ApproximateDepthSync;
			boost::shared_ptr<ApproximateDepthSync> approximate_depth_sync;

			shared_ptr<Sort> sort_tracker;

			int queue_size = 100;

			SortTracking(string name, float delta_t);

			void connect_callback();

			void image_depth_callback(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info);

			void track_people(const cv::Mat image, double confidence_threshold, const cv::Mat depth_image, image_geometry::StereoCameraModel *cam_model, std_msgs::Header header);
	};
};