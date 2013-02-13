/*
 * object_candidate_tracking.h
 *
 *  Created on: Jan 22, 2013
 *      Author: badrobit
 */
// ROS
#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>

#include <pcl/common/centroid.h>	

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

using namespace pcl::tracking;

#ifndef OBJECT_CANDIDATE_TRACKING_H_
#define OBJECT_CANDIDATE_TRACKING_H_

class object_candidate_tracking 
{
public:
	object_candidate_tracking();
	virtual ~object_candidate_tracking();

	bool track_object_candidate( const pcl::PointCloud<pcl::PointXYZ> &input_point_cloud, 
								 pcl::PointCloud<pcl::PointXYZ> &candidate,
								 pcl::PointCloud<pcl::PointXYZRGB> &output );

private:
	bool create_particles( pcl::PointCloud<pcl::PointXYZRGB> &output ); 

	boost::shared_ptr< ParticleFilterTracker<pcl::PointXYZ, ParticleXYZRPY> > 	m_tracker;
};

#endif /* OBJECT_CANDIDATE_TRACKING_H_ */
