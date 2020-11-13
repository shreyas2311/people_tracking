#include<Sort.h>

// Sort::Sort()
// {
// 	cout << "please use set_param function to set max_age and min_hits method";
// }

// Sort::Sort* getSortPointer(int max_age, int min_hits)
// {
// 	Sort* sort_ptr = new Sort(max_age,min_hits);
//   	return sort_ptr;
// }

Sort::Sort(int max_age, int min_hits):max_age(max_age), min_hits(min_hits)
{
	trkNum = 0;
	detNum = 0;
	iouThreshold = 0.3;
}

double Sort::GetIOU(Rect_<float> bb_test, Rect_<float> bb_gt)
{
	float in = (bb_test & bb_gt).area();
	float un = bb_test.area() + bb_gt.area() - in;

	if (un < DBL_EPSILON)
		return 0;

	return (double)(in / un);
}

vector<TrackingBox> Sort::update(vector<Rect> detections)
{

	//Getting predictions for existing trackers
	predictedBoxes.clear();
	
	for(auto it = trackers.begin(); it != trackers.end();)
	{
		Rect_<float> pBox = (*it).predict();
		if (pBox.x >= 0 && pBox.y >= 0)
		{
			predictedBoxes.push_back(pBox);
			it++;
		}
		else
		{
			it = trackers.erase(it);
			//cerr << "Box invalid at frame: " << frame_count << endl;	
		}
	}



	//associate detections to tracked object (both represented as bounding boxes)

	trkNum = predictedBoxes.size();
	detNum = detections.size();
	iouMatrix.clear();
	iouMatrix.resize(trkNum, vector<double>(detNum, 0));


	for (unsigned int i = 0; i < trkNum; i++) // compute iou matrix as a distance matrix
	{
		for (unsigned int j = 0; j < detNum; j++)
		{
			// use 1-iou because the hungarian algorithm computes a minimum-cost assignment.
			iouMatrix[i][j] = 1 - GetIOU(predictedBoxes[i], detections[j]);
			// cout << iouMatrix[i][j] ;
		}
	}


	HungarianAlgorithm HungAlgo;
	assignment.clear();
	if( iouMatrix.size() > 0)
			HungAlgo.Solve(iouMatrix, assignment);

	unmatchedTrajectories.clear();
	unmatchedDetections.clear();
	allItems.clear();
	matchedItems.clear();

	// std::cout << "done\n";

	if (detNum > trkNum)
	{
		// cout << "hi";
		for (unsigned int n = 0; n < detNum; n++)
				allItems.insert(n);

		for (unsigned int i = 0; i < trkNum; ++i)
				matchedItems.insert(assignment[i]);

		set_difference(allItems.begin(), allItems.end(),
				matchedItems.begin(), matchedItems.end(),
				insert_iterator< set<int> >(unmatchedDetections, unmatchedDetections.begin()));
	}
	else if (detNum < trkNum) // there are unmatched trajectory/predictions
	{
		for (unsigned int i = 0; i < trkNum; ++i)
			if (assignment[i] == -1) // unassigned label will be set as -1 in the assignment algorithm
				unmatchedTrajectories.insert(i);
	}

	// cout << matchedItems.size();
	matchedPairs.clear();

	for (unsigned int i = 0; i < trkNum; ++i)
	{
		if (assignment[i] == -1) // pass over invalid values
			continue;
		if (1 - iouMatrix[i][assignment[i]] < iouThreshold)
		{
			unmatchedTrajectories.insert(i);
			unmatchedDetections.insert(assignment[i]);
		}
		else
		{	
			// cout << "assignment "<< i << "\n";
			// cout << assignment[i] << "\n\n";
			matchedPairs.push_back(cv::Point(i, assignment[i]));
		}
	}

	// cout << "total size is\n";
	// cout << matchedPairs.size();

	int detIdx, trkIdx;

	for (unsigned int i = 0; i < matchedPairs.size(); i++)
	{
		trkIdx = matchedPairs[i].x;
		detIdx = matchedPairs[i].y;
		trackers[trkIdx].update(detections[detIdx]);
	}

	// cout << "here";

	for (auto umd : unmatchedDetections)
	{
		KalmanTracker tracker = KalmanTracker(detections[umd]);
		trackers.push_back(tracker);
	}

	frameTrackingResult.clear();

	// cout << "trackers";
	// cout << trackers.size();

	for (auto it = trackers.begin(); it != trackers.end();)
	{

		// cout << (*it).m_hit_streak ;
		if ((*it).m_time_since_update < 1)
		{
			// cout << "here2344";
			TrackingBox res;
			res.box = (*it).get_state();
			res.id = (*it).m_id + 1;
			// res.frame = frame_count;
			frameTrackingResult.push_back(res);
			it++;
		}
		else
			it++;

		// remove dead tracklet
		if (it != trackers.end() && (*it).m_time_since_update > max_age)
			it = trackers.erase(it);
	}

	return frameTrackingResult;

	// cout << "returning\n\n\n";

	// cout << frameTrackingResult.size();
}