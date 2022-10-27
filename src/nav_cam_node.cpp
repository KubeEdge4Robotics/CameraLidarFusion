#include <ros/ros.h>
#include <nav_follow/BoundingBoxes.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>
#define Hydrant_Height 0.71
#define CX 320
#define FX 530.47
#define CY 240
#define FY 530.47
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;

class TargetFollow{
protected:
    ros::NodeHandle nh_;
    ros::Subscriber detect_sub_;
    std::shared_ptr<MoveBaseClient> ac_;
public:
    TargetFollow(ros::NodeHandle& nodehandle):nh_(nodehandle)
    {
        ac_ = make_shared<MoveBaseClient>("move_base", true);
        detect_sub_ = nh_.subscribe<nav_follow::BoundingBoxes>("/detected_objects_in_image", 10, &TargetFollow::detect_callback,this);
    }
    void detect_callback(const nav_follow::BoundingBoxes::ConstPtr &msg){
        ROS_INFO("detect_callback!\n");
        double reference = 2;

        if(msg->bounding_boxes.size()>0){
            double px = (msg->bounding_boxes[0].xmin+msg->bounding_boxes[0].xmax) / 2.0;
            double h = (msg->bounding_boxes[0].ymax-msg->bounding_boxes[0].ymin);
            double thera = -atan2((px-CX),FX);
            double distance = Hydrant_Height*FY/h;
            ROS_INFO("thera=%f, distance=%f\n", thera, distance);
            move_base_msgs::MoveBaseGoal goal;

            if(distance > 1.2*reference){
                goal.target_pose.header.frame_id = "base_link";
                goal.target_pose.header.stamp = msg->header.stamp;
                goal.target_pose.pose.position.x = (distance-reference) * cos(thera);
                goal.target_pose.pose.position.y = (distance-reference) * sin(thera);
                tf2::Quaternion q; q.setRPY(0,0,thera);
                goal.target_pose.pose.orientation.x = q.getX();
                goal.target_pose.pose.orientation.y = q.getY();
                goal.target_pose.pose.orientation.z = q.getZ();
                goal.target_pose.pose.orientation.w = q.getW();
                ROS_INFO("Sending goal");
                ac_->sendGoal(goal);
                ac_->waitForResult();
                if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    ROS_INFO("Sending goal");   
                }
                else{
                    ROS_INFO("Failed");
                }
            }
        }
    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "nav");
    ROS_INFO("Follow begin!\n");
    ros::NodeHandle nh("");
    TargetFollow ttf(nh);
    move_base_msgs::MoveBaseGoal goal;
    ros::spin();
    return 0;
}
