/*
 *
 * Copyright (C) 2016 Ewerton Lopes
 *
 * LICENSE: This is a free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License (LGPL v3) as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version. This code is distributed in the hope
 * that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License (LGPL v3): http://www.gnu.org/licenses/.
 */

#ifndef TRACKER_H
#define TRACKER_H

#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <darknet_ros_py/RecognizedObjectArray.h>
#include <darknet_ros_py/RecognizedObject.h>

//  TODO remove unused includes
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#define MIN_DISTANCE 0.3
#define MAX_DISTANCE 4.2
#define MIN_BLOB_RADIUS 15
#define BUFFER_DIST_SIZE 5

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;


class PlayerTracker{

private:

    ros::Publisher pub;

    image_transport::Publisher pub_result_image;

    std::string depth_frame_name;

    float width_fov;
    float height_fov;
    bool synchronize_frames;

    tf::TransformListener* tfListener;  // Global tf listener pointer

public:


    ros::NodeHandle& nh;

    sensor_msgs::Image last_depth;

    std::string topic_depth_image, topic_roi;


    PlayerTracker(ros::NodeHandle& nh);

    void loop();

    void depthCallback(const sensor_msgs::ImageConstPtr &depth);
//    void roiCallback(const sensor_msgs::RegionOfInterestConstPtr &roi);
    void roiCallback(const darknet_ros_py::RecognizedObjectArrayConstPtr &rec);

//    // Connection callback that unsubscribes from the tracker if no one is subscribed.
//    void connectCallback(image_transport::SubscriberFilter &sub_col,
//                         image_transport::SubscriberFilter &sub_dep,
//                         image_transport::ImageTransport &it);

};

#endif
