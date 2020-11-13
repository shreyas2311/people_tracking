#ifndef SORT_H
#define SORT_H
#include <iostream>
#include <vector>
#include<set>
#include<KalmanTracker.h>
#include<Hungarian.h>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utils/trace.hpp>
#include <memory>

using namespace std;
using namespace cv;

typedef struct TrackingBox
{
	int frame;
	int id;
	Rect_<float> box;
}TrackingBox;

class Sort
{
	int max_age;
	int min_hits;
	vector<KalmanTracker> trackers;
	vector< Rect_<float> > predictedBoxes;
	vector< vector<double> > iouMatrix;
	vector<int> assignment;

	set<int> unmatchedDetections;
	set<int> unmatchedTrajectories;
	set<int> allItems;
	set<int> matchedItems;
	vector<cv::Point> matchedPairs;
	vector<TrackingBox> frameTrackingResult;
	unsigned int trkNum;
	unsigned int detNum;
	double iouThreshold;

	public:

		// Sort();

		Sort(int max_age, int min_hits);

		// static Sort * getSortPointer(int max_age, int min_hits);
		// // {
  // 			Sort* sort_ptr = new Sort(1,3);
  // 			return sort_ptr;
		// }

		vector<TrackingBox> update(vector<Rect> detections);

		double GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt);
		// {
		// float in = (bb_test & bb_gt).area();
		// float un = bb_test.area() + bb_gt.area() - in;

		// if (un < DBL_EPSILON)
		// 	return 0;

		// return (double)(in / un);
		// }

};

#endif