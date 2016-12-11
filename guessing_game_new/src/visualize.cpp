#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h> // enable pcl publishing
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

// Visualization
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>

//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;


//! Callback for processing the Point Cloud data
void showCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr plane, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cluster)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_two_clouds (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_two_clouds->setBackgroundColor(0,0,0);

     // cloud: green / cloud2: red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color1 (plane, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color2 (cluster, 255, 0, 0);

    //add both
    viewer_two_clouds->addPointCloud<pcl::PointXYZ> (plane, single_color1, "plane_cloud");
    viewer_two_clouds->addPointCloud<pcl::PointXYZ> (cluster, single_color2, "cluster_cloud");

    // set coordinateSystem and init camera
    viewer_two_clouds->addCoordinateSystem(0.01);
    viewer_two_clouds->initCameraParameters();

    while(!viewer_two_clouds->wasStopped())
    {
      viewer_two_clouds->spinOnce();
      boost::this_thread::sleep (boost::posix_time::microseconds(100000));
    }
    viewer_two_clouds->close();
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_visualization");
    ros::NodeHandle nh;
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > cloud_sub(nh,"/segmentation/plane_points", 1);
    message_filters::Subscriber<pcl::PointCloud<pcl::PointXYZ> > clust_sub(nh,"/segmentation/clusters_points", 1);
    message_filters::TimeSynchronizer<pcl::PointCloud<pcl::PointXYZ>,pcl::PointCloud<pcl::PointXYZ> > sync(cloud_sub, clust_sub, 10);
    sync.registerCallback(boost::bind(&showCloud, _1, _2));
    ros::spin();
    return 0;
}


