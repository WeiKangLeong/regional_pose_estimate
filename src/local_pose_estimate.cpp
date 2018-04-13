/* Author: WeiKang */

#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <laser_geometry/laser_geometry.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

class LocalAmclEstimate
{
    public:
        LocalAmclEstimate();
        //~LocalAmclEstimate();

    private:
        void fake_movement(const geometry_msgs::PoseWithCovarianceStampedConstPtr& initial);
        void compress_laser_scan(const sensor_msgs::LaserScan::ConstPtr& scan_in);

        tf::TransformBroadcaster* tfb_;
        tf::TransformListener* listener_;
        tf::Transform tf_;
        tf::Quaternion q_;

        laser_geometry::LaserProjection projector_;

        ros::NodeHandle nh_;
        ros::Subscriber pose_sub_;
        ros::Subscriber laser_sub_;
        ros::Publisher laser_pcl_pub_;
        ros::ServiceClient nomotion_serv_;

        std::string parent_frame_id_, child_frame_id_;

};

LocalAmclEstimate::LocalAmclEstimate()
: parent_frame_id_("amcl_pose"), child_frame_id_("base_link")
{
    pose_sub_ = nh_.subscribe("/initialpose", 100, &LocalAmclEstimate::fake_movement, this);
    laser_sub_ = nh_.subscribe("/scooter/front_top_scan_filtered", 100, &LocalAmclEstimate::compress_laser_scan, this);
    nomotion_serv_ = nh_.serviceClient<std_srvs::Empty>("request_no_motion_update");
    laser_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("/top_scan_topdown", 1);

    listener_ = new tf::TransformListener;
    tf_.setOrigin(tf::Vector3(0.0, 0.0, 1.89));
    q_.setRPY(0.0, 0.262, 0.0);
    tf_.setRotation(q_);
}

void LocalAmclEstimate::fake_movement(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initial)
{
    std_srvs::Empty nothing_one, nothing_two;
    nomotion_serv_.call(nothing_one);
    tf_.setOrigin(tf::Vector3(initial->pose.pose.position.x, initial->pose.pose.position.y, initial->pose.pose.position.z));
    //tf::quaternionMsgToTF(initial->pose.pose.orientation, tf_.setRotation());
    //tf_.setRotation(tf::Quaternion(initial->pose.pose.orientation.x, initial->pose.pose.orientation.y, initial->pose.pose.orientation.z, initial->pose.pose.orientation.w));
    //tf::StampedTransform odom_transform_stamped(tf_, initial->header.stamp, "/scooter/odom", "/scooter/map");
    //tfb_->sendTransform(odom_transform_stamped);
}

void LocalAmclEstimate::compress_laser_scan(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(!listener_->waitForTransform(
            "/scooter/front_top_lidar",
            "/scooter/base_link",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
            ros::Duration(1.0))){
         return;
      }

    sensor_msgs::PointCloud2 laser_cloud, laser_cloud_2;
    //projector_.transformLaserScanToPointCloud("/scooter/base_link",*scan_in, laser_cloud, *listener_, 0.0, 1);
    projector_.projectLaser (*scan_in, laser_cloud);


    //projector_.projectLaser(*scan_in, laser_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pcl_2 (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (laser_cloud, *laser_pcl);
    pcl_ros::transformPointCloud(*laser_pcl, *laser_pcl_2, tf_);

    for (int i=0; i<laser_pcl_2->size(); i++)
    {
        laser_pcl_2->points[i].z = 0.0;
    }
    std::cout<<"laser size: "<<scan_in->ranges.size()<<" point cloud size: "<<laser_pcl_2->size()<<std::endl;
    pcl::toROSMsg(*laser_pcl_2, laser_cloud_2);
    laser_cloud_2.header = scan_in->header;
    laser_cloud_2.header.frame_id = "top_down_laser";
    laser_pcl_pub_.publish(laser_cloud_2);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_pose_estimate");
    LocalAmclEstimate localamclestimate;
    ros::spin();
    return 0;
}
