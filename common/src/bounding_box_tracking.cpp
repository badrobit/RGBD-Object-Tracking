/*
 * bounding_box_tracking.cpp
 *
 *  Created on: Jan 23, 2013
 *      Author: badrobit
 */

#include "bounding_box_tracking.h"

const double m_movement_tolerance = 0.03; 

bounding_box_tracking::bounding_box_tracking()
{
	m_first_run = true; 
	m_object_exceeds_tolerance = false; 

	ROS_INFO( "Bounding Box Tracking Library Initilized" ); 
}

bounding_box_tracking::~bounding_box_tracking() 
{
	// TODO Auto-generated destructor stub
}

bool 
bounding_box_tracking::track_centroid( pcl::PointCloud< pcl::PointXYZ> &candidate, visualization_msgs::Marker &marker_output )
{
	ROS_INFO( "Bounding Box Tracking: starting tracking" ); 
	ros::Time start, finish;
    start = ros::Time::now();

	if( candidate.size() == 0 )
    {
      ROS_WARN( "Bounding Box Tracking: Empty Point Cloud Passed In" ); 
      return false; 
    }

	Eigen::Vector4f candidate_centroid;
    pcl::compute3DCentroid( candidate, candidate_centroid );

    if( m_first_run )
    {
    	m_initial_position = candidate_centroid; 
    	m_first_run = false; 
    }
    else
    {
    	ROS_DEBUG( "Bounding Box Tracking: Reference Position( %f, %f, %f )", m_initial_position[0], m_initial_position[1], m_initial_position[2] ); 
    	ROS_DEBUG( "Bounding Box Tracking: Current Position  ( %f, %f, %f )", candidate_centroid[0], candidate_centroid[1], candidate_centroid	[2] ); 

    	double distance = sqrt( pow( m_initial_position[0] - candidate_centroid[0], 2 ) + 
    							pow( m_initial_position[1] - candidate_centroid[1], 2 ) ); 

    	ROS_INFO( "Bounding Box Tracking: Distance=%f", distance ); 

    	if( distance >= m_movement_tolerance )
    	{
    		m_object_exceeds_tolerance = true;
    	}
    	else
    	{
    		m_object_exceeds_tolerance = false; 
    	}
    }

    // We want to track the time here because anything after this is just drawing which we track seperately.
    finish = ros::Time::now();
    ROS_INFO("tracking_time=%lf", (finish.toSec() - start.toSec() ));

    if( !m_object_exceeds_tolerance )
    {
    	marker_output = mark_cluster( candidate, 100, 0, 1, 0 ); 
    }
    else
    {
    	marker_output = mark_cluster( candidate, 100, 1, 0, 0 ); 
    }

	return false; 	
}

visualization_msgs::Marker 
bounding_box_tracking::mark_cluster( pcl::PointCloud<pcl::PointXYZ> &cloud_cluster, int id, float r, float g, float b ) 
{ 
	ros::Time start, finish;
    start = ros::Time::now();

	Eigen::Vector4f centroid; 
	Eigen::Vector4f min; 
	Eigen::Vector4f max; 

	pcl::compute3DCentroid( cloud_cluster, centroid); 
	pcl::getMinMax3D( cloud_cluster, min, max); 

	//Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    //trans.translation().matrix() = Eigen::Vector3f (candidate_centroid[0], candidate_centroid[1], candidate_centroid[2]);

	uint32_t shape = visualization_msgs::Marker::CUBE; 
	visualization_msgs::Marker marker; 
	marker.header.frame_id = cloud_cluster.header.frame_id; 
	marker.header.stamp = ros::Time::now(); 

	marker.id = id; 
	marker.type = shape; 
	marker.action = visualization_msgs::Marker::ADD; 

	marker.pose.position.x = centroid[0]; 
	marker.pose.position.y = centroid[1]; 
	marker.pose.position.z = centroid[2]; 
	marker.pose.orientation.x = 0.25; 
	marker.pose.orientation.y = 0.0; 
	marker.pose.orientation.z = 0.0; 
	marker.pose.orientation.w = 1.0; 

	marker.scale.x = (max[0]-min[0]); 
	marker.scale.y = (max[1]-min[1]); 
	marker.scale.z = (max[2]-min[2]); 

	if (marker.scale.x ==0) 
	  marker.scale.x=0.1; 

	if (marker.scale.y ==0) 
	marker.scale.y=0.1; 

	if (marker.scale.z ==0) 
	marker.scale.z=0.1; 

	marker.color.r = r; 
	marker.color.g = g; 
	marker.color.b = b; 
	marker.color.a = 0.5; 

	marker.lifetime = ros::Duration(0.5); 

	ROS_INFO( "Markers Output" ); 
	finish = ros::Time::now();
    ROS_INFO("drawing_tracking_time=%lf", (finish.toSec() - start.toSec() ));
	return marker; 
}