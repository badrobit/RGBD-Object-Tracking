/*
 * platformextraction.h
 *
 *  Created on: Dec 19, 2012
 *      Author: badrobit
 */

#ifndef PLATFORM_EXTRACTION_H_
#define PLATFORM_EXTRACTION_H_

#include <ros/ros.h>

/*
 * Includes for the Point Cloud library which is required in order to process the point clouds that
 * we are being provided with. More information can be found at: http://www.pointclouds.org.
 */
 #include <pcl/io/pcd_io.h>
 #include <pcl/point_types.h>
 #include <pcl/ModelCoefficients.h>
 #include <pcl/filters/extract_indices.h>
 #include <pcl/filters/extract_indices.h>
 #include <pcl/sample_consensus/model_types.h>
 #include <pcl/sample_consensus/method_types.h>
 #include <pcl/segmentation/sac_segmentation.h>

 #include "pcl/filters/project_inliers.h"

/*
 * This class is responsible for finding the platform of the KUKA youBot robot and segmenting it
 * from all of the points in the provided point cloud. This is a part of the safe object
 * transportation pipeline and is one of the first steps.
 */
class platform_extraction
{
public:
	platform_extraction();
	virtual ~platform_extraction();

	void extract_platform( pcl::PointCloud<pcl::PointXYZ> &input_point_cloud, 
				 		   pcl::PointCloud<pcl::PointXYZ> &extracted_platform_cloud );

};

#endif /* PLATFORMEXTRACTION_H_ */