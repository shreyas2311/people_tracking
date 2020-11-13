#include"KalmanTracker3d.h"
#include"ros/ros.h"

int KalmanTracker3d::kf3d_count = 0;


KalmanTracker3d::KalmanTracker3d(StateType3d stateMat, float d_t , double time_of_update):d_t(d_t),time_of_update(time_of_update)
{

	int stateNum = 6;
	int measureNum = 6;

	measurement = Mat::zeros(measureNum, 1, CV_32F);

	kf3d = KalmanFilter(stateNum, measureNum, 0);

	kf3d.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 
		1, d_t, 0, 0, 0, 0,
		0, 1, 0, 0, 0, 0,
		0, 0, 1, d_t, 0, 0,
		0, 0, 0, 1, 0, 0,
		0, 0, 0, 0, 1, d_t,
		0, 0, 0, 0, 0, 1);

	prev_pos = stateMat;

	kf3d.statePost.at<float>(0, 0) = stateMat.position.x;
	kf3d.statePost.at<float>(1, 0) = 0;
	kf3d.statePost.at<float>(2, 0) = stateMat.position.y;
	kf3d.statePost.at<float>(3, 0) = 0;
	kf3d.statePost.at<float>(4, 0) = stateMat.position.z;
	kf3d.statePost.at<float>(5, 0) = 0;

	
	setIdentity(kf3d.measurementMatrix);
	setIdentity(kf3d.processNoiseCov, Scalar::all(1e-2));
	setIdentity(kf3d.measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kf3d.errorCovPost, Scalar::all(1));
}


StateType3d KalmanTracker3d::predict()
{
	// predict
	Mat p = kf3d.predict();
	age += 1;

	if (time_since_update > 0)
		hit_streak = 0;
	time_since_update += 1;

	// StateType predictBox = get_rect_xysr(p.at<float>(0, 0), p.at<float>(1, 0), p.at<float>(2, 0), p.at<float>(3, 0));

	people_msgs::Person predicted_person;
	predicted_person.position.x = p.at<float>(0, 0);
	predicted_person.velocity.x = p.at<float>(1, 0);
	predicted_person.position.y = p.at<float>(2, 0);
	predicted_person.velocity.y = p.at<float>(3, 0);
	predicted_person.position.z = p.at<float>(4, 0);
	predicted_person.velocity.z = p.at<float>(5, 0);

	history.push_back(predicted_person);
	return history.back();
}

StateType3d KalmanTracker3d::update(StateType3d stateMat)
{
	time_since_update = 0;
	history.clear();
	hits += 1;
	hit_streak += 1;



	measurement.at<float>(0, 0) =  stateMat.position.x;
	measurement.at<float>(1, 0) =  (stateMat.position.x -  prev_pos.position.x )/ d_t;
	measurement.at<float>(2, 0) =  stateMat.position.y;
	measurement.at<float>(3, 0) =  (stateMat.position.y -  prev_pos.position.y )/ d_t;
	measurement.at<float>(4, 0) =  stateMat.position.z;
	measurement.at<float>(5, 0) =  (stateMat.position.z -  prev_pos.position.z )/ d_t;

	Mat return_mat = kf3d.correct(measurement);

	StateType3d return_state;
	
	return_state.name = stateMat.name;
	return_state.position.x = return_mat.at<float>(0,0);
	return_state.velocity.x = return_mat.at<float>(1,0);
	return_state.position.y = return_mat.at<float>(2,0);
	return_state.velocity.y = return_mat.at<float>(3,0);
	return_state.position.z = return_mat.at<float>(4,0);
	return_state.velocity.z = return_mat.at<float>(5,0);

	// ROS_INFO("position z is %s", return_state.name.c_str());
	prev_pos = return_state;

	return return_state;
}

double KalmanTracker3d::get_time_of_update()
{
	return time_of_update;
}
void KalmanTracker3d::set_tou(double time)
{
	time_of_update = time;
}
KalmanTracker3d::~KalmanTracker3d()
{
	
}
