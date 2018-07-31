#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <laser_geometry/laser_geometry.h>
#include <tf/transform_listener.h>

#define PI 3.14159265359

ros::Publisher pub;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pcl_3 (new pcl::PointCloud<pcl::PointXYZ>);

sensor_msgs::LaserScan laser_one;
sensor_msgs::LaserScan laser_two;
sensor_msgs::LaserScan laser_three;

tf::TransformListener *tf_listener_;
tf::StampedTransform side1_transform, side2_transform, side3_transform;

laser_geometry::LaserProjection projector_;

/*void
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
    //pcl::fromROSMsg (*input, *cloud_in);

  //sensor_msgs::PointCloud2 output;

  // Do data processing here...
  //output = *input;
  pcl::toROSMsg (*cloud_in, output);

  // Publish the data.
  pub.publish (output);
}*/

sensor_msgs::LaserScan pointcloud_to_laser(sensor_msgs::PointCloud2 cloud_msg)
  {

    //build laserscan output
    sensor_msgs::LaserScan output;
    output.header.frame_id = "combined_laser";
    output.header.stamp = laser_one.header.stamp;


    output.angle_min = -PI;
    output.angle_max = PI;
    output.angle_increment = PI/180.0;
    output.time_increment = 1.0/30.0;
    output.scan_time = 1.0/50.0;
    output.range_min = 0.4;
    output.range_max = 50.0;

    //determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range

      output.ranges.assign(ranges_size, output.range_max + 1.0);


    sensor_msgs::PointCloud2 cloud_out;
    sensor_msgs::PointCloud2Ptr cloud;

    // Transform cloud if necessary

      cloud_out = cloud_msg;


    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float>
              iter_x(cloud_out, "x"), iter_y(cloud_out, "y"), iter_z(cloud_out, "z");
              iter_x != iter_x.end();
              ++iter_x, ++iter_y, ++iter_z)
    {
        double range = hypot(*iter_x, *iter_y);
        double angle = atan2(*iter_y, *iter_x);
      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;
      }

    }
    //pub_.publish(output);
    return output;
  }


void sidekick1_cb(sensor_msgs::LaserScanConstPtr scan)
{
    laser_one = *scan;
    //std::cout<<"laser 1: "<<laser_one.header.stamp.sec<<" "<<laser_one.header.stamp.nsec<<std::endl;
}

void sidekick2_cb(sensor_msgs::LaserScanConstPtr scan)
{
    //laser_two = *scan;
    //std::cout<<"laser 2: "<<laser_two.header.stamp.sec<<" "<<laser_two.header.stamp.nsec<<std::endl;
    sensor_msgs::PointCloud2 laser_cloud, laser_cloud_2;
    //projector_.transformLaserScanToPointCloud("/scooter/base_link",*scan_in, laser_cloud, *listener_, 0.0, 1);
    projector_.projectLaser (*scan, laser_cloud);


    //projector_.projectLaser(*scan_in, laser_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pcl (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_pcl_2 (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg (laser_cloud, *laser_pcl);

    tf::Transform tf_;
    tf::Quaternion q_;
    tf_.setOrigin(tf::Vector3(-0.08, 0.0, 1.71));
    q_.setRPY(0.0, 0.2618, 0.0);
    tf_.setRotation(q_);
    pcl_ros::transformPointCloud(*laser_pcl, *laser_pcl_2, tf_);

    //std::cout<<laser_pcl_2->points[0].z<<std::endl;

    for (int i=0; i<laser_pcl_2->size(); i++)
    {

        if (laser_pcl_2->points[i].z>=0.4 && laser_pcl_2->points[i].z<=1.0)
        {
            laser_pcl_2->points[i].z = 0.0;
            laser_pcl_3->push_back(laser_pcl_2->points[i]);
        }

    }
}

void sidekick3_cb(sensor_msgs::LaserScanConstPtr scan)
{
    laser_three = *scan;
    //std::cout<<"laser 3: "<<laser_three.header.stamp.sec<<" "<<laser_three.header.stamp.nsec<<std::endl;
}

int
main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "laser_combine");
    ros::NodeHandle nh;

	ros::Rate loop_rate(15.0);

    tf_listener_ = new tf::TransformListener();

    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
    ros::Subscriber sub_laser_one = nh.subscribe<sensor_msgs::LaserScan> ("/front_bottom_lidar", 1, sidekick1_cb);
    ros::Subscriber sub_laser_two = nh.subscribe<sensor_msgs::LaserScan> ("/top_scan", 1, sidekick2_cb);
    ros::Subscriber sub_laser_three = nh.subscribe<sensor_msgs::LaserScan> ("/tim_sidekick3", 1, sidekick3_cb);

    pub = nh.advertise<sensor_msgs::PointCloud2> ("laser_combined", 1);

    sensor_msgs::PointCloud2 laser_cloud;
    sensor_msgs::PointCloud2 output2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_assembly(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_two(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_three(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one_tf(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_two_tf(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_three_tf(new pcl::PointCloud<pcl::PointXYZ>);

    /*try{
        tf_listener_->lookupTransform("/base_link", "/sidekick1",
                                      ros::Time::now(), side1_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("tim sidekick 1: Looking Transform Failed");
	std::cout<<ros::Time::now()<<std::endl;
        //return;
    }

    try{
        tf_listener_->lookupTransform("/sidekick2", "/base_link",
                                      ros::Time::now(), side2_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("tim sidekick 2: Looking Transform Failed");
        //return;
    }

    try{
        tf_listener_->lookupTransform("sidekick3", "base_link",
                                      ros::Time::now(), side3_transform);
    }catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("tim sidekick 3: Looking Transform Failed");
        //return;
    }*/

	tf::Transform side_one, side_two, side_three;
        tf::Quaternion quaternion_one, quaternion_two, quaternion_three;
        quaternion_one.setRPY(0.0,0.0,0.0);
        side_one.setOrigin(tf::Vector3(1.03, 0.0, 0.0));
	side_one.setRotation(quaternion_one);
        quaternion_two.setRPY(PI, 0.0, 1.5158);
	side_two.setOrigin(tf::Vector3(-1.0, 0.77, 0.76));
	side_two.setRotation(quaternion_two);
        quaternion_three.setRPY(PI, 0.0, -1.5968);
	side_three.setOrigin(tf::Vector3(-1.05, -0.72, 0.76));
        side_three.setRotation(quaternion_three);
        //tf::Quaternion quaternion_one(0.0,0.0,0.0);
        //side_one.setOrigin(tf::Vector3(0.0,0.0,0.0));
        //side_one.setRotation(quaternion_one);
        /*tf::Quaternion quaternion_two(0.0,0.0,0.0);
        side_two.setOrigin(tf::Vector3(0.0,0.0,0.0));
        side_two.setRotation(quaternion_two);
        tf::Quaternion quaternion_three(0.0,0.0,0.0);
        side_three.setOrigin(tf::Vector3(0.0,0.0,0.0));
        side_three.setRotation(quaternion_three);*/

    while (ros::ok())
    {
	ros::spinOnce();
        sensor_msgs::LaserScan scan_combined;
//        double time_one, time_two, time_three;
//        time_one=laser_one.header.stamp.toSec();
//        time_two=laser_two.header.stamp.toSec();
//        time_three = laser_three.header.stamp.toSec();

//      if (std::fabs(time_two-time_one)<0.05 && std::fabs(time_three-time_one)<0.05)
//      {
          
          projector_.projectLaser(laser_one, laser_cloud);
          pcl::fromROSMsg(laser_cloud, *cloud_one);
          pcl_ros::transformPointCloud (*cloud_one, *cloud_one_tf, side_one);
          *laser_assembly+=*cloud_one_tf;

          *laser_assembly+=*laser_pcl_3;
          laser_pcl_3->clear();

          //laser_input->clear();
//          projector_.projectLaser(laser_two, laser_cloud);
//          pcl::fromROSMsg(laser_cloud, *cloud_two);
//          pcl_ros::transformPointCloud (*cloud_two, *cloud_two_tf, side_two);
//          *laser_assembly+=*cloud_two_tf;
//          //laser_input->clear();
//          projector_.projectLaser(laser_three, laser_cloud);
//          pcl::fromROSMsg(laser_cloud, *cloud_three);
//          pcl_ros::transformPointCloud (*cloud_three, *cloud_three_tf, side_three);
//          *laser_assembly+=*cloud_three_tf;
//          //laser_input->clear();
          pcl::toROSMsg(*laser_assembly, output2);
//          sensor_msgs::PointCloud2ConstPtr output3 = *output2;
//          scan_combined = pointcloud_to_laser(output2);
          output2.header.stamp = laser_one.header.stamp;
          output2.header.frame_id = "base_link";
          pub.publish(output2);
        //std::cout<<"cloud 1: "<<cloud_one->size()<<" cloud 2: "<<cloud_two->size()<<" cloud 3: "<<cloud_three->size()<<" and total: "<<laser_assembly->size()<<std::endl;
        //std::cout<<time_one<<" "<<time_two<<" "<<time_three<<std::endl;
          //std::cout<<"time diff: "<<time_two-time_one<<" and time diff: "<<time_three-time_one<<std::endl;
          laser_assembly->clear();
//      }
//      else
//      {
//          std::cout<<"time diff: "<<time_two-time_one<<" and time diff: "<<time_three-time_one<<std::endl;
//      }
	loop_rate.sleep();

    }


    // Spin
    //ros::spin ();
}

