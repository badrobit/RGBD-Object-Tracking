/*
 * platformextraction.cpp
 *
 *  Created on: Dec 19, 2012
 *      Author: badrobit
 */

#include "platform_extraction.h"

platform_extraction::platform_extraction() 
{
	ROS_INFO( "Platform Extraction Library Ready." ); 
}

platform_extraction::~platform_extraction() 
{
	// TODO Auto-generated destructor stub
}


void platform_extraction::extract_platform( pcl::PointCloud<pcl::PointXYZ> &input_point_cloud, 
										 	pcl::PointCloud<pcl::PointXYZ> &extracted_platform_cloud )
{
	ROS_DEBUG( "Platform Extraction: Input Point Cloud Size: %d", (int) input_point_cloud.size() );
	ros::Time start, finish;
	start = ros::Time::now();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f ( new pcl::PointCloud<pcl::PointXYZ> ); 
	pcl::ModelCoefficients::Ptr coefficients ( new pcl::ModelCoefficients ); 
	pcl::PointIndices::Ptr inliers ( new pcl::PointIndices ); 

	pcl::SACSegmentation<pcl::PointXYZ> segmentation; 
	segmentation.setOptimizeCoefficients( true ); 
	segmentation.setModelType( pcl::SACMODEL_PLANE ); 
	segmentation.setMethodType( pcl::SAC_RANSAC ); 
	segmentation.setDistanceThreshold( 0.01 ); 
	segmentation.setInputCloud( input_point_cloud.makeShared() ); 
	segmentation.segment( *inliers, *coefficients ); 

	if( inliers->indices.size() == 0 )
	{
		ROS_WARN( "Platform Extraction: Could not find a plannar model in the data" ); 
	}

	pcl::ExtractIndices< pcl::PointXYZ > extract;
    extract.setInputCloud( input_point_cloud.makeShared() );
    extract.setIndices( inliers );
    extract.setNegative( false );
    extract.filter( extracted_platform_cloud );

    extract.setNegative( true );
    extract.filter( *cloud_f );
    input_point_cloud = *cloud_f;

    finish = ros::Time::now();
    ROS_INFO("platform_extraction_time=%lf", (finish.toSec() - start.toSec() ));
    ROS_INFO( "Platform Extraction Complete." ); 
}