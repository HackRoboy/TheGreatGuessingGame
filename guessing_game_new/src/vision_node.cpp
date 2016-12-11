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
#include <sensor_msgs/Image.h>
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
class PlaneSegmentation
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

    //! Publisher for pointclouds
    ros::Publisher pub_plane_pc_;
    ros::Publisher pub_clusters_pc_;
    ros::Publisher pub_image_;


    //! Parameters
    float threshold; 
    float percent;  


    //! Internal data
    pcl::PointCloud<pcl::PointXYZ> curr_table_pc;
    pcl::PointCloud<pcl::PointXYZ> curr_clusters_pc;



    //------------------ Callbacks -------------------

    //! Callback for service calls
  

    //! Callback for subscribers
    //! Complete processing of new point cloud
    void processCloud(const sensor_msgs::PointCloud2ConstPtr& var); // for multiple data topics (const sensor_msgs::TypeConstPtr &var, const sensor_msgs::TypeConstPtr &var, ...)

public:
    //! Subscribes to and advertises topics
    PlaneSegmentation(ros::NodeHandle nh) : nh_(nh), priv_nh_("~") //,
        //sub(nh, "topic", 5) // constructor initialization form for the subscriber if needed
    {

        threshold = 0.05;
        percent = 0.6;

        pub_plane_pc_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ> >("/segmentation/plane_points", 10);
        pub_clusters_pc_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ> >("/segmentation/clusters_points", 10);
        pub_image_ = nh_.advertise<sensor_msgs::Image>("/segmentation/image_mask", 10);

        cloud_sub = nh_.subscribe("/kinect2/qhd/points", 1, &PlaneSegmentation::processCloud, this);



        // Callback function register

        //initialize params



    }

    ~PlaneSegmentation() {}
};

//! Callback for processing the Point Cloud data
void PlaneSegmentation::processCloud(const sensor_msgs::PointCloud2ConstPtr& var)
{

    pcl::PointCloud< pcl::PointXYZ > pc; // internal data
   
  // Convert the data to the internal var (pc) using pcl function: fromROSMsg
  // TODO
    pcl::fromROSMsg(*var, pc);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pc.makeShared(); // cloud to operate
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f( new pcl::PointCloud<pcl::PointXYZ> ); // cloud to store the filter the data

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p( new pcl::PointCloud<pcl::PointXYZ> ); // cloud to store the main plane cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane( new pcl::PointCloud<pcl::PointXYZ> ); // cloud to store the main plane cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_np( new pcl::PointCloud<pcl::PointXYZ> ); // cloud to store the non-main plane cloud

    Mat img(pc.width,pc.height,CV_8UC1);;

    std::cout << "PointCloud before filtering has: " << pc.points.size() << " data points." << std::endl; //*
    std::cout << "width: " << pc.width << "height: " << pc.height << std::endl;


    // Down sample the pointcloud using VoxelGrid
  // TODO
    pcl::VoxelGrid<pcl::PointXYZ> vog;
    vog.setInputCloud (cloud);
    vog.setLeafSize (0.01f, 0.01f, 0.01f); // 1 cm
    vog.filter (*cloud_f);
    //----

    // // Create the filtering object
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud (cloud_f);
    // sor.setMeanK (50);
    // sor.setStddevMulThresh (1.0);
    // sor.filter (*cloud_f);

    std::cout << "PointCloud after filtering has: " << cloud_f->points.size()  << " data points." << std::endl;

    // Create the segmentation object for the plane model and set all the parameters using pcl::SACSegmentation<pcl::PointXYZ>
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices );
    pcl::ModelCoefficients::Ptr coefficients( new pcl::ModelCoefficients );

    // set parameters of the SACS segmentation
  // TODO 
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (threshold);
    // seg.setInputCloud (cloud_f);
    //----

    // Segment the planes using pcl::SACSegmentation::segment() function and pcl::ExtractIndices::filter() function
    // TODO
    // If you want to extract more than one plane you have to do a while
    int nr_points = cloud_f->points.size();
    while (cloud_f->points.size () > percent * nr_points)
    {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_f);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_f);
    extract.setIndices (inliers);
    extract.setNegative (false);
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);

    for (int i=0; i < inliers.indices.size(); i++)
    {
      img.at<double>(inliers.indices[i])=255;
    }

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_np);
    *cloud_f = *cloud_np;
    *cloud_p += *cloud_plane; 
    }

    imshow("Mask",img);

    // Publish biggest plane
  // TODO
    pub_plane_pc_.publish(cloud_p->makeShared());

  // Tips: 
  // - you can copy the pointcloud using pcl::copyPointCloud()
  // - Set the header frame_id to the pc2 header frame_id
  // - you can use pcl_conversions::toPCL() to convert the stamp from pc2 header to pointcloud header stamp
  // - to publish -> pub_plane_pc_.publish( pointcloud_variable.makeShared() )
    //----

    // Publish other clusters
    // TODO similar to the previous publish
    pub_clusters_pc_.publish(cloud_np->makeShared());
    
    return;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_segmentation");
    ros::NodeHandle nh;
    PlaneSegmentation node(nh);
    ros::spin();
    return 0;
}


