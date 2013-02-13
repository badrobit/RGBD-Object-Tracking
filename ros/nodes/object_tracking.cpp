#include <ros/ros.h> 
#include <ros/console.h>

#include <visualization_msgs/Marker.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include "platform_extraction.h"
#include "object_candidate_extraction.h"
#include "object_candidate_tracking.h"
#include "bounding_box_tracking.h"

#include <pcl/io/pcd_io.h>  
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>

class object_tracking_node
{
public: 

    object_tracking_node()
    { 
        m_platform_extracter = platform_extraction(); 
        m_object_canadidate_extracter = object_candidate_extraction(); 
        m_object_candidate_tracking = object_candidate_tracking();
        m_bounding_box_tracker = bounding_box_tracking(); 


        m_point_cloud_subscriber =  m_node_handler.subscribe<sensor_msgs::PointCloud2>( "safe_object_transportation_input", 
                                                                                        1, 
                                                                                        &object_tracking_node::get_point_cloud_data, 
                                                                                        this );

        m_platform_publisher = m_node_handler.advertise<sensor_msgs::PointCloud2>( "safe_object_transportation/segmented_platform", 1 ); 

        m_preprocessing_publisher = m_node_handler.advertise<sensor_msgs::PointCloud2>( "safe_object_transportation/preprocessed_input", 1 ); 

        m_platform_marker_publisher = m_node_handler.advertise<visualization_msgs::Marker>( "safe_object_transportation/platform_markers", 0 );

        m_object_canadidate_marker_publisher = m_node_handler.advertise<visualization_msgs::Marker>( "safe_object_transportation/object_markers", 0 );

        m_object_canadidate_publisher = m_node_handler.advertise<sensor_msgs::PointCloud2>( "safe_object_transportation/object_candidates", 1 ); 

        m_object_canadidate_tracking_publisher = m_node_handler.advertise<sensor_msgs::PointCloud2>( "safe_object_transportation/tracking_particles", 1 ); 

        ROS_INFO( "Started Safe Object Transporation Node." ); 
    }

private: 

    void get_point_cloud_data( const sensor_msgs::PointCloud2::ConstPtr& input_point_cloud )
    {
        ROS_INFO( "tracking_starting" ); 
        ros::Time overall_start, overall_finish;
        overall_start = ros::Time::now();

        pcl::PointCloud<pcl::PointXYZRGB> tracking_points; 
        visualization_msgs::Marker object_bounding_box; 

        pcl::PointCloud<pcl::PointXYZ> point_cloud;         
        pcl::PointCloud<pcl::PointXYZ> platform_point_cloud; 
        while( !prepare_point_cloud( input_point_cloud, point_cloud ) );
        pcl::PointCloud<pcl::PointXYZ> object_candidate;

        publish_tracking_particles( point_cloud, m_preprocessing_publisher ); 

        m_platform_extracter.extract_platform( point_cloud, platform_point_cloud );  

        publish_point_cloud( platform_point_cloud, m_platform_publisher, m_platform_marker_publisher );   

        if( m_object_canadidate_extracter.extract_object_candidates( point_cloud, object_candidate ) )
        {
          publish_tracking_particles( object_candidate, m_object_canadidate_publisher ); 

          m_bounding_box_tracker.track_centroid( object_candidate, object_bounding_box ); 

          m_object_canadidate_marker_publisher.publish( object_bounding_box );
        }

        overall_finish = ros::Time::now();
        ROS_INFO("overall_time=%lf", (overall_finish.toSec() - overall_start.toSec() ));
        
        //publish_object_candidates( object_candidates, m_object_canadidate_publisher, m_object_canadidate_marker_publisher );  

        //m_object_candidate_tracking.track_object_candidate( point_cloud, platform_point_cloud, tracking_points );  

        //publish_tracking_particles( tracking_points, m_object_canadidate_tracking_publisher );
    }

    bool prepare_point_cloud( const sensor_msgs::PointCloud2::ConstPtr& input, pcl::PointCloud<pcl::PointXYZ> &output )
    {
        sensor_msgs::PointCloud2::Ptr cloud_filtered (new sensor_msgs::PointCloud2 ());
        pcl::PointCloud<pcl::PointXYZ> temp_cloud; 

        if( !input )
        {
            ROS_ERROR( "Error No Point Cloud Passed In!" );
            return false;  
        }

        // -------------- Subsampling -------------------------------------- //
        ros::Time subsampling_start, subsampling_finish;
        subsampling_start = ros::Time::now();

        pcl::VoxelGrid<sensor_msgs::PointCloud2> sor;
        sor.setInputCloud( input );
        sor.setLeafSize (0.001f, 0.001f, 0.001f);
        sor.filter( *cloud_filtered );

        pcl::fromROSMsg( *cloud_filtered, temp_cloud ); 

        subsampling_finish = ros::Time::now();
        ROS_INFO("subsampling_time=%lf", (subsampling_finish.toSec() - subsampling_start.toSec() ));

        // -------------- Limit distance Kinect can see -------------------- //
        ros::Time roi_start, roi_finish;
        roi_start = ros::Time::now();

        pcl::PassThrough<pcl::PointXYZ> passthrough_filter;
        passthrough_filter.setInputCloud( temp_cloud.makeShared() );
        passthrough_filter.setFilterFieldName( "x" );
        passthrough_filter.setFilterLimits( -0.15, 0.15 );
        passthrough_filter.filter( output ); 

        passthrough_filter.setInputCloud( output.makeShared() );
        passthrough_filter.setFilterFieldName( "y" );
        passthrough_filter.setFilterLimits( -0.07, 0.07 );
        passthrough_filter.filter( output ); 

        passthrough_filter.setInputCloud( output.makeShared() );
        passthrough_filter.setFilterFieldName( "z" );
        passthrough_filter.setFilterLimits( 0.40, 0.58 );
        passthrough_filter.filter( output ); 

        roi_finish = ros::Time::now();
        ROS_INFO("roi_time=%lf", (roi_finish.toSec() - roi_start.toSec() ));

        ROS_INFO( "Point Cloud Preparation Complete" ); 
        return true; 
    }

    void publish_point_cloud( pcl::PointCloud<pcl::PointXYZ> &point_cloud, ros::Publisher &publisher, ros::Publisher &marker_publisher )
    {
        sensor_msgs::PointCloud2 output_cloud; 
        pcl::toROSMsg( point_cloud, output_cloud ); 
        publisher.publish( output_cloud ); 

        marker_publisher.publish( mark_cluster( point_cloud.makeShared(), "test" , 1, 1, 0, 0 ) );

        ROS_INFO( "Published Point Cloud" ); 
    }

    void publish_tracking_particles( pcl::PointCloud<pcl::PointXYZ> &point_cloud, ros::Publisher &publisher )
    {
        sensor_msgs::PointCloud2 output_cloud; 
        pcl::toROSMsg( point_cloud, output_cloud ); 
        publisher.publish( output_cloud ); 

        ROS_INFO( "Published Point Cloud" ); 
    }

    void publish_object_candidates( std::vector<pcl::PointCloud<pcl::PointXYZ> > &object_candidates, ros::Publisher &object_publisher, ros::Publisher &marker_publisher )
    {
        for( std::size_t i = 0; i != object_candidates.size(); i++ )
        {
            sensor_msgs::PointCloud2 output_cloud; 
            ROS_INFO_STREAM( "Point cloud size:" << object_candidates[0].size() );
            pcl::toROSMsg( object_candidates[0], output_cloud ); 

            object_publisher.publish( output_cloud ); 
            marker_publisher.publish( mark_cluster( object_candidates[0].makeShared(), "test" , i, 1, 0, 1 ) );
        }

        ROS_INFO( "Object Candidate Point Clouds Published" ); 
    }

    visualization_msgs::Marker mark_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, std::string ns ,int id, float r, float g, float b) 
    { 
      Eigen::Vector4f centroid; 
      Eigen::Vector4f min; 
      Eigen::Vector4f max; 
      
      pcl::compute3DCentroid (*cloud_cluster, centroid); 
      pcl::getMinMax3D (*cloud_cluster, min, max); 
      
      uint32_t shape = visualization_msgs::Marker::CUBE; 
      visualization_msgs::Marker marker; 
      marker.header.frame_id = cloud_cluster->header.frame_id; 
      marker.header.stamp = ros::Time::now(); 
      
      marker.ns = ns; 
      marker.id = id; 
      marker.type = shape; 
      marker.action = visualization_msgs::Marker::ADD; 
      
      marker.pose.position.x = centroid[0]; 
      marker.pose.position.y = centroid[1]; 
      marker.pose.position.z = centroid[2]; 
      marker.pose.orientation.x = 0.0; 
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

      //marker.lifetime = ros::Duration(); 
      marker.lifetime = ros::Duration(0.5); 

      ROS_INFO( "Markers Output" ); 
      return marker; 
    }

    /** Variables **/ 
    ros::NodeHandle             m_node_handler;
    ros::Subscriber             m_point_cloud_subscriber; 

    ros::Publisher              m_platform_publisher;
    ros::Publisher              m_preprocessing_publisher; 
    ros::Publisher              m_platform_marker_publisher; 
    ros::Publisher              m_object_canadidate_publisher; 
    ros::Publisher              m_object_canadidate_marker_publisher;
    ros::Publisher              m_object_canadidate_tracking_publisher;

    platform_extraction         m_platform_extracter;
    object_candidate_extraction m_object_canadidate_extracter; 
    object_candidate_tracking   m_object_candidate_tracking; 
    bounding_box_tracking       m_bounding_box_tracker; 
};


int main(int argc, char **argv)
{
    ros::init( argc, argv, "safe_object_transportation" ); 

    object_tracking_node ros_node; 

    ros::spin(); 
	return 0;
}
