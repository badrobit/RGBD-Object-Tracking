/*
 * bounding_box_tracking.h
 *
 *  Created on: Jan 23, 2013
 *      Author: badrobit
 */

#include <math.h>

/***************** ROS *********************/
#include <ros/ros.h>
#include <ros/console.h>
#include <visualization_msgs/Marker.h>

/***************** PCL *********************/
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>	

#ifndef BOUNDING_BOX_TRACKING_H_
#define BOUNDING_BOX_TRACKING_H_

class bounding_box_tracking 
{
public:
	bounding_box_tracking();
	virtual ~bounding_box_tracking();

	bool track_centroid( pcl::PointCloud< pcl::PointXYZ> &candidate, visualization_msgs::Marker &marker_output );

private: 
	visualization_msgs::Marker mark_cluster( pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, int id, float r, float g, float b );


	bool				m_first_run;

	bool				m_object_exceeds_tolerance; 
	Eigen::Vector4f 	m_initial_position; 
};

#endif /* BOUNDING_BOX_TRACKING_H_ */