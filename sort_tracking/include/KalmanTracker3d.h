#ifndef KALMAN_3DH
#define KALMAN_3DH 2
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <people_msgs/Person.h>
#include <people_msgs/PositionMeasurementArray.h>

using namespace std;
using namespace cv;

#define StateType3d people_msgs::Person

class KalmanTracker3d
{
	public:
		KalmanTracker3d(StateType3d stateMat, float d_t, double time_of_update);
		~KalmanTracker3d();
		// {
		// 	init_kf3D(StateType());
		// 	time_sice_update = 0;
		// }

		float d_t;

		
		static int kf3d_count;
		
		int time_since_update;
		int hits;
		int hit_streak;
		int age;
		int id;
		double time_of_update;

		StateType3d prev_pos;

		cv::KalmanFilter kf3d;
		cv::Mat measurement;

		std::vector<StateType3d> history;

		StateType3d predict();
		StateType3d update(StateType3d stateMat);

		void set_tou(double time);

		double get_time_of_update();
};

#endif
