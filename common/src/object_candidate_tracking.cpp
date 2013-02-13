/*
 * objectcandidatetracking.cpp
 *
 *  Created on: Jan 22, 2013
 *      Author: badrobit
 */

#include "object_candidate_tracking.h"

object_candidate_tracking::object_candidate_tracking()
{
	boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZ, ParticleXYZRPY> > tracker (new KLDAdaptiveParticleFilterOMPTracker< pcl::PointXYZ, ParticleXYZRPY > ());
    tracker->setMaximumParticleNum (500);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    ParticleXYZRPY bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;
    tracker->setBinSize (bin_size);
    m_tracker = tracker;

	PCL_INFO( "Object Candidate Tracking Library Ready"); 
}

object_candidate_tracking::~object_candidate_tracking() 
{
}

bool object_candidate_tracking::track_object_candidate( const pcl::PointCloud<pcl::PointXYZ> &input_point_cloud, 
														pcl::PointCloud<pcl::PointXYZ> &candidate,
														pcl::PointCloud<pcl::PointXYZRGB> &output )
{
    ROS_INFO( "TRACKING: Input Point Cloud Size: %d", (int) candidate.size() );

	std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

	m_tracker->setTrans (Eigen::Affine3f::Identity ());
    m_tracker->setStepNoiseCovariance (default_step_covariance);
    m_tracker->setInitialNoiseCovariance (initial_noise_covariance);
    m_tracker->setInitialNoiseMean (default_initial_mean);
    m_tracker->setIterationNum (1);
    
    m_tracker->setParticleNum (400);
    m_tracker->setResampleLikelihoodThr( 0.1 );
    m_tracker->setUseNormal( false );

    ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr coherence = ApproxNearestPairPointCloudCoherence<pcl::PointXYZ>::Ptr (new ApproxNearestPairPointCloudCoherence<pcl::PointXYZ> ());

    boost::shared_ptr<DistanceCoherence<pcl::PointXYZ> > distance_coherence = boost::shared_ptr<DistanceCoherence<pcl::PointXYZ> > (new DistanceCoherence<pcl::PointXYZ> ());
    coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<pcl::search::Octree<pcl::PointXYZ> > search (new pcl::search::Octree<pcl::PointXYZ> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.001);
    m_tracker->setCloudCoherence (coherence);

    Eigen::Vector4f candidate_centroid;
    pcl::compute3DCentroid( candidate, candidate_centroid );
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.translation().matrix() = Eigen::Vector3f (candidate_centroid[0], candidate_centroid[1], candidate_centroid[2]);

    m_tracker->setInputCloud( boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >( input_point_cloud ) );
    m_tracker->setReferenceCloud( boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >( candidate ) ); 
    m_tracker->setTrans(trans);
    m_tracker->compute();

    output.header = input_point_cloud.header; 
    create_particles( output ); 

	return false; 
}

bool object_candidate_tracking::create_particles( pcl::PointCloud<pcl::PointXYZRGB> &output )
{
	ParticleFilterTracker<pcl::PointXYZ, ParticleXYZRPY>::PointCloudStatePtr particles = m_tracker->getParticles ();
    if( particles )
    {
        for (size_t i = 0; i < particles->points.size (); i++)
        {
            pcl::PointXYZRGB point;

            point.x = particles->points[i].x;
            point.y = particles->points[i].y;
            point.z = particles->points[i].z;
            uint8_t r = 0;    
            uint8_t g = 255;
            uint8_t b = 0;
            int32_t rgb = (r << 16) | (g << 8) | b;
            point.rgb = *(float *)(&rgb);  
            output.points.push_back( point ); 
        }
    	return true;
    }
    else
    {
      PCL_WARN("no particles\n");
      return false;
    }

	return false; 
}