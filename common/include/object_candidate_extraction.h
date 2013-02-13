/*
 * object_candidate_extraction.h
 *
 *  Created on: Dec 19, 2012
 *      Author: badrobit
 */

// ROS
#include <ros/ros.h>
#include <ros/console.h>
 
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#ifndef OBJECT_CANDIDATE_EXTRACTION_H_
#define OBJECT_CANDIDATE_EXTRACTION_H_

class object_candidate_extraction
{
public:
	object_candidate_extraction();
	virtual ~object_candidate_extraction();
	bool extract_object_candidates( pcl::PointCloud<pcl::PointXYZ> &input_point_cloud, 
									pcl::PointCloud<pcl::PointXYZ> &object_candidate );
};

#endif /* OBJECT_CANDIDATE_EXTRACTION_H_ */
