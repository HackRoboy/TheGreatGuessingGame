#include <ros/ros.h>
#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Char.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <image_geometry/pinhole_camera_model.h>


// Visualization
//#include <visualization_msgs/MarkerArray.h>
//#include <visualization_msgs/Marker.h>


//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;
using namespace cv;

//! Plane segmentation class
//! computes and split the big planes from the rest of the point cloud clusters
class Clustering
{

private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;


    //! Subscribers to the PointCloud data
    // Optional: MESSAGE FILTERS COULD BE A GOOD IDEA FOR GRABBING MULTIPLE TOPICS SYNCRONIZED, NOT NEEDED THOUGH
    // message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub(nh_,"/kinect2/qhd/points", 1);
    // message_filters::Subscriber<sensor_msgs::Image> img_sub(nh_,"/kinect2/qhd/image_color_rect", 1);
    // TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::Image> sync(cloud_sub, img_sub, 10);
    // sync.registerCallback(boost::bind(&processCloud, _1, _2));

    ros::Subscriber cloud_sub;
    pcl::PCDWriter writer;

    //! Parameters
    float tolerance; 
    int min_size;
    int max_size; 


    //------------------ Callbacks -------------------

    //! Callback for service calls
	

    //! Callback for subscribers
    //! Complete processing of new point cloud
    void processCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr&); // for multiple data topics (const sensor_msgs::TypeConstPtr &var, const sensor_msgs::TypeConstPtr &var, ...)

public:
    //! Subscribes to and advertises topics
    Clustering(ros::NodeHandle nh) : nh_(nh), priv_nh_("~") //,
        //sub(nh, "topic", 5) // constructor initialization form for the subscriber if needed
    {
        tolerance = 0.05;
        min_size = 100;
        max_size = 25000;

        cloud_sub = nh_.subscribe("/segmentation/clusters_points", 1, &Clustering::processCluster, this);

        // Callback function register

        //initialize params

    }

    ~Clustering() {}
};

//! Callback for processing the Point Cloud data
void Clustering::processCluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_filtered)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (tolerance); 
    ec.setMinClusterSize (min_size);
    ec.setMaxClusterSize (max_size);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
        j++;

    }
    
    return;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "clustering");
    ros::NodeHandle nh;
    Clustering node(nh);
    ros::spin();
    return 0;
}


