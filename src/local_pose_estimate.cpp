/* Author: WeiKang */

#include "ros/ros.h"

#include "tf/transform_listener.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_srvs/Empty.h"

class LocalAmclEstimate
{
    public:
        LocalAmclEstimate();
        //~LocalAmclEstimate();

    private:
        void fake_movement(const geometry_msgs::PoseWithCovarianceStampedConstPtr& initial);

        ros::NodeHandle nh_;
        ros::Subscriber pose_sub_;
        ros::ServiceClient nomotion_serv_;

        std::string parent_frame_id_, child_frame_id_;

};

LocalAmclEstimate::LocalAmclEstimate()
: parent_frame_id_("amcl_pose"), child_frame_id_("base_link")
{
    pose_sub_ = nh_.subscribe("/initialpose", 100, &LocalAmclEstimate::fake_movement, this);
    nomotion_serv_ = nh_.serviceClient<std_srvs::Empty>("request_no_motion_update");
}

void LocalAmclEstimate::fake_movement(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initial)
{
    std_srvs::Empty nothing_one, nothing_two;
    nomotion_serv_.call(nothing_one);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_pose_estimate");
    LocalAmclEstimate localamclestimate;
    ros::spin();
    return 0;
}
