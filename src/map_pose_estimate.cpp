#include <ros/ros.h>
// PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/filters/passthrough.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>

#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

ros::Publisher pub, pub_small_map, pub_corrected_pose;

bool map_saved;

pcl::PointCloud<pcl::PointXYZ>::Ptr saved_map (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PassThrough<pcl::PointXYZ> pass;
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

void map_cb(const nav_msgs::OccupancyGridConstPtr& map_input)
{
    if(map_saved==false)
    {
        double width = map_input->info.width;
        double height = map_input->info.height;
        double resolution = map_input->info.resolution;
        double map_size = width*height;
        int k=0;
        for (int i=0; i<width; i++)
        {
            for (int j=0; j<height; j++)
            {
                k++;

                if (map_input->data[k]==1)
                {
                    pcl::PointXYZ map_point;
                    map_point.x = (height - j)*resolution;
                    map_point.y = (width -i)*resolution;
                    map_point.z = 0.0;
                    saved_map->push_back(map_point);

                }
            }

        }
        if (k>=map_size)
        {
            map_saved = true;
        }
    }

}

void clicked_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& init_pose)
{
    double x_pose = init_pose->pose.pose.position.x;
    double y_pose = init_pose->pose.pose.position.y;
    tf::Transform clicked_pose, icp_tf, final_tf;
    clicked_pose.setOrigin(tf::Vector3(x_pose, y_pose, 0.0));
    tf::Quaternion heading;
    tf::quaternionMsgToTF(init_pose->pose.pose.orientation, heading);
    clicked_pose.setRotation(heading);
    pcl_ros::transformPointCloud (*cloud_in, *cloud_in, clicked_pose);

    pcl::PointCloud<pcl::PointXYZ>::Ptr smaller_region (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    pass.setInputCloud(saved_map);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-35.0+x_pose, 35.0+x_pose);
    pass.filter(*smaller_region);

    pass.setInputCloud(smaller_region);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-35.0+y_pose, 35.0+y_pose);
    pass.filter(*smaller_region);

    sensor_msgs::PointCloud2 output, output2;
    pcl::toROSMsg(*smaller_region, output2);
    pub_small_map.publish(output2);

    Eigen::Matrix4f cm;
    icp.setInputSource (cloud_in);
    icp.setInputTarget (smaller_region);
    icp.align(*cloud_out);
    double score = icp.getFitnessScore();
    cm = icp.getFinalTransformation();

    tf::Matrix3x3 tf3d;
    tf3d.setValue(double(cm(0,0)),double(cm(0,1)),double(cm(0,2)),double(cm(1,0)),double(cm(1,1)),double(cm(1,2)),double(cm(2,0)),double(cm(2,1)),double(cm(2,2)));
    tf::Quaternion tfqt;
    tf3d.getRotation(tfqt);

    icp_tf.setOrigin(tf::Vector3(cm(0,3),cm(1,3),cm(2,3)));
    icp_tf.setRotation(tfqt);

    final_tf = clicked_pose * icp_tf;


    pcl::toROSMsg (*cloud_out, output);
    pub.publish(output);
}

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{


  pcl::fromROSMsg (*input, *cloud_in);

  //pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  map_saved = false;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("input", 1, cloud_cb);
  ros::Subscriber sub_map = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, map_cb);
  ros::Subscriber sub_pose = nh.subscribe <geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, clicked_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  pub_small_map = nh.advertise<sensor_msgs::PointCloud2> ("/selected_map", 1);
  pub_corrected_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);

  // Spin
  ros::spin ();
}

