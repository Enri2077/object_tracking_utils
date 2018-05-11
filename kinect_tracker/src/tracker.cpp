/* main.cpp -- This file is part of the robogame_kinectfeatures_extractor ROS node created for
 * the purpose of extracting relevant motion features from images.
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

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <iomanip>
#include <time.h>
#include <signal.h>


#include "tracker.h"

#define QUEUE_SIZE 5
#define RATE 30


void PlayerTracker::depthCallback(const sensor_msgs::ImageConstPtr &depth){
//    ROS_INFO("Received depth");
	last_depth = *depth;
}


//void PlayerTracker::roiCallback(const sensor_msgs::RegionOfInterestConstPtr &roi){
void PlayerTracker::roiCallback(const darknet_ros_py::RecognizedObjectArrayConstPtr &rec){

	//////////// TEMPORARY variables ////////////
//    float bb_x_min_norm = 0.45;
//	float bb_x_max_norm = 0.55;
//	float bb_y_min_norm = 0.65;
//	float bb_y_max_norm = 0.75;
    /////////////////////////////////////////////

    ROS_INFO("Received objects");
	for(darknet_ros_py::RecognizedObject object : rec->objects){

		sensor_msgs::RegionOfInterest roi = object.bounding_box;

		ROS_INFO_STREAM("object: " << object);

		cv_bridge::CvImagePtr cv_depth_ptr;

		try{
			// Get depth image as matrix
			cv_depth_ptr = cv_bridge::toCvCopy(last_depth);
		}catch (cv_bridge::Exception& e){
			ROS_ERROR("%s",e.what());
			return;
		}

		cv::Mat depmat = cv_depth_ptr->image;
		int depth_frame_width = depmat.size().width;
		int depth_frame_height = depmat.size().height;


		float bb_x_min_norm = ((double)roi.x_offset) / (double)depth_frame_width;
		float bb_x_max_norm = ((double)roi.x_offset + (double)roi.width) / (double)depth_frame_width;
		float bb_y_max_norm = ((double)roi.y_offset + (double)roi.height) / (double)depth_frame_height;

		// normalized coordinate of the bottom center point of the bounding box in the image
		float bb_bottom_x = (bb_x_min_norm + bb_x_max_norm)/2;
		float bb_bottom_y = bb_y_max_norm; // Notice the origin of the y coordinate in the image correspond to the top

	//    int bb_x_min = bb_x_min_norm*depth_frame_width;
	//    int bb_x_max = bb_x_max_norm*depth_frame_width;
	//    int bb_y_min = bb_y_min_norm*depth_frame_height;
	//    int bb_y_max = bb_y_max_norm*depth_frame_height;
	//    int bb_width = bb_x_max - bb_x_min;
	//    int bb_height = bb_y_max - bb_y_min;
		int bb_x_min = roi.x_offset;
	//    int bb_x_max = bb_x_max_norm*depth_frame_width;
		int bb_y_min = roi.y_offset;
	//    int bb_y_max = bb_y_max_norm*depth_frame_height;
		int bb_width = roi.width;
		int bb_height = roi.height;

		ROS_INFO("Received bb_bottom_x: %f bb_bottom_y: %f", bb_bottom_x, bb_bottom_y);

		cv::Rect bb_rect(bb_x_min, bb_y_min, bb_width, bb_height);
		cv::Mat roi_mat;//, roiArea;

		try{
			// obtain the image ROI:
			depmat.convertTo(depmat, CV_32F, 1.0);
			roi_mat = cv::Mat(depmat, bb_rect);

//			std::cout << roi_mat << std::endl;

		}catch (cv::Exception ex){
			ROS_ERROR("%s",ex.what());
			return;
		}

		cv:Scalar distance = cv::mean(roi_mat, roi_mat == roi_mat);        // compute mean value of the region of interest.
												   // RECALL: the pixels correspond to distance in mm.
//		ROS_INFO_STREAM("distance:" << distance[0]);

		// Defining rho, phi and theta
		float rho = distance[0];
		float phi = (0.5 - bb_bottom_x) * width_fov;
		float theta = M_PI / 2 - (0.5 - bb_bottom_y) * height_fov;

		float x = rho * sin(theta) * cos(phi);
		float y = rho * sin(theta) * sin(phi);
		float z = rho * cos(theta);

		ROS_INFO_STREAM("rho:\t"<<rho<<"\tphi:\t"<<phi*180/M_PI<<" deg\ttheta:\t"<<theta*180/M_PI<<" deg");
		ROS_INFO_STREAM("x:\t"<< x << "\ty:\t"<<  y << "\tz:\t"<< z);

//		ROS_INFO("rho:\t%.2f\tphi:\t%.2fdeg\ttheta:\t%.2fdeg", rho, phi*180/M_PI, theta*180/M_PI);
//		ROS_INFO("x:\t%.2f\ty:\t%.2f\tz:\t%.2f", x, y, z);

	//	// TF-Broadcaster
	//	static tf::TransformBroadcaster br;
	//	tf::StampedTransform playerTransform;
	//
	//	tf::Transform framePlayerTransform;
	//	framePlayerTransform.setOrigin( tf::Vector3(x, y, z) );
	//	tf::Quaternion q;
	//	q.setRPY(0, 0, 0);
	//	framePlayerTransform.setRotation(q);
	//	ros::Time now = ros::Time::now();
	//	br.sendTransform(tf::StampedTransform(framePlayerTransform, now, "/camera", "/object"));
	}


}

void PlayerTracker::loop(){

    ros::Rate rate(RATE);

    while(ros::ok()){
    	// Process callbacks at rate's frequency
        rate.sleep();
	    ros::spinOnce();
    }
}


PlayerTracker::PlayerTracker(ros::NodeHandle& nh) : nh(nh)  {

    nh.getParam("depth_topic", topic_depth_image);
    nh.getParam("roi_topic", topic_roi);
    nh.getParam("width_fov", width_fov);
    nh.getParam("height_fov", height_fov);
    nh.getParam("synchronize_frames", synchronize_frames);

//    ros::Subscriber sub = nh.subscribe(topic_depth_image, 1000, &PlayerTracker::depthCallback, this);


//    ROS_INFO_STREAM("Color depth topic: " << sub.getNumPublishers());
    
    ROS_INFO_STREAM("Color depth topic: " << topic_depth_image);

  	tfListener = new tf::TransformListener();

}


int main(int argc, char** argv){
    ros::init(argc, argv, "kinect_tracker");
    ros::NodeHandle nh;

    PlayerTracker player_tracker(nh);

    ros::Subscriber sub_depth = nh.subscribe(player_tracker.topic_depth_image, 1000, &PlayerTracker::depthCallback, &player_tracker);
    ros::Subscriber sub_roi = nh.subscribe(player_tracker.topic_roi, 1000, &PlayerTracker::roiCallback, &player_tracker);
    
    player_tracker.loop();

    // Create a subscriber.
    // Set queue size to 1 because generating a queue here will only pile up images and delay the output by the amount of queued images
//    image_transport::SubscriberFilter subscriber_depth;
//    image_transport::SubscriberFilter subscriber_image;

//    subscriber_depth.subscribe(it, player_tracker.topic_depth_image.c_str(),1);
//    subscriber_image.subscribe(it, player_tracker.topic_color_image.c_str(),1);
	
//    //The real queue size for synchronisation is set here.
//    sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy(QUEUE_SIZE);
//    MySyncPolicy.setAgePenalty(1000); //set high age penalty to publish older data faster even if it might not be correctly synchronized.
//
//    // Create synchronization policy. Here: async because time stamps will never match exactly
//    const sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> MyConstSyncPolicy = MySyncPolicy;
//    Synchronizer< sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> > sync(MyConstSyncPolicy,
//                                                                                       subscriber_depth,
//                                                                                       subscriber_image);
//    // Register one callback for all topics
//    sync.registerCallback(boost::bind(&PlayerTracker::callback, &player_tracker, _1, _2));
    



    return 0;
}
