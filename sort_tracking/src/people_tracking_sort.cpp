#include<people_tracking_sort.h>


namespace SortTracking
{
    SortTracking::SortTracking(string name, float delta_t):it_(nh_),delta_t(delta_t),rate(10)
    {
        camera_name = nh_.resolveName("camera");
        image_topic_name = nh_.resolveName("image_topic");
        depth_topic_name = nh_.resolveName("depth_topic");

        rgb_ns = nh_.resolveName("rgb_ns");
        depth_ns = nh_.resolveName("depth_ns");
        camera_info = nh_.resolveName("camera_info");
        camera_rgb_info_topic = ros::names::clean(camera_name + "/" + rgb_ns + "/" + camera_info);
        camera_depth_info_topic = ros::names::clean(camera_name + "/" + depth_ns + "/" + camera_info);
        
        marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 20);

        ros::SubscriberStatusCallback position_publisher_connect_callback = boost::bind(&SortTracking::connect_callback, this);
        people_tracker_measurements_pub = nh_.advertise<people_msgs::People>("people_tracker/person_tracker_measurement", 10, position_publisher_connect_callback, position_publisher_connect_callback);
        

        image_topic = ros::names::clean(camera_name + "/" + rgb_ns + "/" + image_topic_name);
        depth_topic = ros::names::clean(camera_name + "/" + depth_ns + "/" + depth_topic_name);

        image_test_pub = it_.advertise("out_image_base_topic", 1);

        approximate_depth_sync.reset(new ApproximateDepthSync(ApproximateDepthSync(queue_size), image_sub_, depth_image_sub_, c1_info_sub_, c2_info_sub_));
        approximate_depth_sync->registerCallback(boost::bind(&SortTracking::image_depth_callback, this, _1, _2, _3, _4));
              
        sort_tracker = make_shared<Sort>(1,3);

        ros::NodeHandle local_nh("~");
        local_nh.param("model_txt", model_txt_, std::string(""));
        local_nh.param("model_bin", model_bin_, std::string(""));
        String model_txt = model_txt_;
        String model_bin = model_bin_;
        net = readNetFromCaffe(model_txt, model_bin);

        ROS_INFO("Please subscribe to people_tracker_measurements.");
      
        ros::spin();

    }

    void SortTracking::connect_callback()
    {
        if(people_tracker_measurements_pub.getNumSubscribers() == 0)
        {
            ROS_INFO("Please subscribe to people detectors outbound topics");
            image_sub_.unsubscribe();
            depth_image_sub_.unsubscribe();
            c1_info_sub_.unsubscribe();
            c2_info_sub_.unsubscribe(); 
        }
        else
        {
            image_sub_.subscribe(it_, image_topic, 3);
            depth_image_sub_.subscribe(it_, depth_topic, 3);
            c1_info_sub_.subscribe(nh_, camera_rgb_info_topic, 3);
            c2_info_sub_.subscribe(nh_, camera_depth_info_topic, 3);
        }
    }

    void SortTracking::image_depth_callback(const sensor_msgs::Image::ConstPtr &image, const sensor_msgs::Image::ConstPtr& depth_image, const sensor_msgs::CameraInfo::ConstPtr& c1_info, const sensor_msgs::CameraInfo::ConstPtr& c2_info )
    {
        // sensor_msgs::CameraInfo info:: = c2_info; 
        cv_bridge::CvImageConstPtr cv_image_ptr = cv_bridge::toCvShare(image, "bgr8");
        cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image);
        cv::Mat depth_32fc1 = cv_depth_ptr->image;
        if (depth_image->encoding != "32FC1")
        {
            cv_depth_ptr->image.convertTo(depth_32fc1, CV_32FC1, 0.001);
        }

        bool true_ = cam_model_.fromCameraInfo(c1_info, c2_info);
        // ROS_INFO("changed %d", true_);

        track_people(cv_image_ptr->image, 0.2, depth_32fc1, &cam_model_, image->header);
    }

    void SortTracking::track_people(const cv::Mat image, double confidence_threshold, const cv::Mat depth_image, image_geometry::StereoCameraModel *cam_model, std_msgs::Header header)
    {
        Mat resize_image;
        Mat image_copy = image.clone();
        resize(image_copy, resize_image,  Size(300,300));
        Mat inputBlob = blobFromImage(resize_image, 0.007843, Size(300,300), Scalar(127.5, 127.5, 127.5), false);
        net.setInput(inputBlob, "data");
        Mat detection = net.forward("detection_out");
        Mat detectionMat(detection.size[2], detection.size[3], CV_32F, detection.ptr<float>());
        vector<Rect> detections;
        for (int i = 0; i < detectionMat.rows; i++)
        {
            float confidence = detectionMat.at<float>(i, 2);
            if (confidence > confidence_threshold)
            {
                int idx = static_cast<int>(detectionMat.at<float>(i, 1));
                if(idx == 15)
                {
                    
                    int xLeftBottom = static_cast<int>(detectionMat.at<float>(i, 3) * image.cols);
                    int yLeftBottom = static_cast<int>(detectionMat.at<float>(i, 4) * image.rows);
                    int xRightTop = static_cast<int>(detectionMat.at<float>(i, 5) * image.cols);
                    int yRightTop = static_cast<int>(detectionMat.at<float>(i, 6) * image.rows);

                    Rect object((int)xLeftBottom, (int)yLeftBottom,
                            (int)(xRightTop - xLeftBottom),
                            (int)(yRightTop - yLeftBottom));
                    detections.push_back(object);
                }
            }
        }

        people_msgs::PositionMeasurement pos;
        people_msgs::PositionMeasurementArray pos_meas_array;
        people_msgs::Person person;
        people_msgs::People people;
        people_msgs::People people_pub;
        
        for(auto each : sort_tracker->update(detections))
        {   
            ros::Time current_time = ros::Time::now();
            pos.header.stamp    =  current_time;
            pos.header.frame_id =  header.frame_id;

            Mat depth_roi(  depth_image, Rect( floor(each.box.x + 0.25*each.box.width),
                                        floor(each.box.y + 0.25*each.box.height),
                                        floor(each.box.x + 0.75*each.box.width) - floor(each.box.x + 0.25*each.box.width) + 1,
                                        floor(each.box.y + 0.75*each.box.height) - floor(each.box.y + 0.25*each.box.height) + 1 )  );
            std::vector<float> depths;

            for (int i =0; i<depth_roi.rows; i++)
            {
                float *dptr = depth_roi.ptr<float>(i);
                for(int j = 0; j<depth_roi.cols; j++)
                {
                    if(dptr[j] == dptr[j]) //To eliminate nans
                    {
                        depths.push_back(dptr[j]);  
                    }
                }
            }

            cv::Point2d center2d = Point2d(each.box.x + (each.box.width/2.0),
                                           each.box.y + (each.box.height/2.0)  );

            std::vector<float>::iterator dbegin = depths.begin();
            std::vector<float>::iterator dend = depths.end();
            if (depths.size() > 0)
            {
                std::sort(dbegin, dend);
                double avg_d = depths[floor(depths.size() / 2.0)];
                cv::Point3d center3d;
                // each_person.width3d = fabs((cam_model->left()).getDeltaX(each_person.bounding_box.width, avg_d));                
                center3d =  (cam_model->left()).projectPixelTo3dRay(center2d);
                center3d = (avg_d / center3d.z) *center3d;
                person.position.x =  center3d.x;
                person.position.y =  center3d.y;       
                person.position.z =  center3d.z;
                // person.object_id = to_string(each.id);
                person.name = to_string(each.id);
                people.people.push_back(person);

            }
        }



        for(people_msgs::Person each_person : people.people)
        {
            auto it = trackers3d.find(each_person.name);
            if(it == trackers3d.end())
            {
                
                trackers3d.insert({each_person.name.c_str(),KalmanTracker3d(each_person, delta_t, ros::Time::now().toSec())});
                people_pub.people.push_back(each_person);
            }
            else
            { 
                it->second.predict();
                people_pub.people.push_back(it->second.update(each_person));
                it->second.set_tou(ros::Time::now().toSec());
            }  
        }

        auto it = trackers3d.begin();

        current_time = ros::Time::now().toSec();        

        while(it != trackers3d.end())
        {
            if(current_time - it->second.get_time_of_update() > 10.0)
            {
                it = trackers3d.erase(it);
            }   
            else
            {
                it++;
            }
        }
        people_tracker_measurements_pub.publish(people_pub); 

       
        // publishing mrkers

        int count = 0;

        for(people_msgs::Person each_person : people_pub.people)
        {
            visualization_msgs::Marker m;
            m.header.stamp =  ros::Time::now();
            m.header.frame_id = header.frame_id;
            m.ns = "Person";
            m.id = count;
            m.type = m.SPHERE;
            m.pose.position.x = each_person.position.x;
            m.pose.position.y = each_person.position.y;
            m.pose.position.z = each_person.position.z;
            m.scale.x = .20;
            m.scale.y = .20;
            m.scale.z = .20;

            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0;

            marker_pub.publish(m);

            // ROS_INFO("Publishong here...");


            // count = count + 1;
        }
        // if(pub_marker)
        // {
        //     visualization_msgs::Marker m;
        //     m.header.stamp =  current_time;
        //     m.header.frame_id = header.frame_id;
        //     m.ns = "Person";
        //     m.id = i;
        // }   

        rate.sleep();

    }
};


int main(int argc, char **argv)
{
    //rate chosen is 0.1. Need to add param as a roslaunch file. Also need to change ros rate accordingly.
    ros::init(argc, argv, "sort_tracking");

    SortTracking::SortTracking SortTracking(ros::this_node::getName(), 10);

    ROS_INFO("Hi");

    return 0;
}
