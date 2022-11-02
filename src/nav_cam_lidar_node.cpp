#include <thread>
#include <string>
#include <cmath>
#include <limits>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
//引入消息头文件
#include <nav_follow/BoundingBoxes.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf2/impl/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/opencv.hpp>

#define MATCH_THRESHOLD 10
#define Hydrant_Height 0.71
#define CX 320
#define FX 530.47
#define CY 240
#define FY 530.47
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
using namespace std;
// using namespace pcl;
using namespace cv;

string camera_frame_id("camera_link");
string base_frame_id("base_link");
string odom_frame_id("odom");
string lidar_frame_id("lidar_link");
string world_frame_id("world_link");
typedef pcl::PointXYZ PointType;
class TargetFollow{
protected:
    ros::NodeHandle nh_;
    ros::Subscriber detect_sub_;
    ros::Publisher point_pub_;
    std::shared_ptr<MoveBaseClient> ac_;
    std::shared_ptr<std::thread> main_thread_;
    shared_ptr<tf2_ros::Buffer> buffer_;
    shared_ptr<tf2_ros::TransformListener> tf_listener_;
    shared_ptr<sensor_msgs::PointCloud> point_cloud_;
    shared_ptr<pcl::PointCloud<PointType>> point_cloud_world_;
    shared_ptr<sensor_msgs::PointCloud> target_point_cloud_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::LaserScan>> scan_sub_;
    shared_ptr<tf2_ros::MessageFilter<sensor_msgs::LaserScan>> scan_filter_;
    double yaw_camera_;
    double yaw_scan_;
    double camera2scan_yaw_;
    double dist_camera_;
    double dist_scan_;
    ros::Time last_update_;
    double reference_;//目的地距目标参考距离
    cv::Mat image;


public:
    TargetFollow(ros::NodeHandle& nodehandle, double frequency):nh_(nodehandle), yaw_camera_(0), yaw_scan_(0),
    camera2scan_yaw_(0), dist_camera_(0), dist_scan_(0), last_update_(ros::Time::now()),reference_(1.5){
        ac_ = make_shared<MoveBaseClient>("move_base", true);
        detect_sub_ = nh_.subscribe<nav_follow::BoundingBoxes>("/detected_objects_in_image", 2, &TargetFollow::detectCallback, this); 
        point_pub_ = nh_.advertise<sensor_msgs::PointCloud>("/tracks", 10);
        buffer_ = make_shared<tf2_ros::Buffer>(ros::Duration(15));
        scan_sub_ = make_shared<message_filters::Subscriber<sensor_msgs::LaserScan>>(nh_, "/scan", 2);
        scan_filter_ = make_shared<tf2_ros::MessageFilter<sensor_msgs::LaserScan>>(*scan_sub_, *buffer_, odom_frame_id, 2, nh_);
        scan_filter_->registerCallback(boost::bind(&TargetFollow::scanCallback, this, _1));//tf过滤器注册回调函数
        while(!buffer_->canTransform(lidar_frame_id, camera_frame_id, ros::Time(0))){
            ROS_INFO("Wait for transform from camera_link to base_scan... \n");
            ros::Duration(0.5).sleep();
        }
        geometry_msgs::TransformStamped camera2scan = buffer_->lookupTransform(camera_frame_id, lidar_frame_id, ros::Time(0));
        tf2::Quaternion quat_tf;
        tf2::fromMsg(camera2scan.transform.rotation, quat_tf);
        camera2scan_yaw_ = tf2::impl::getYaw(quat_tf);
        ROS_INFO("Yaw for scan to camera is %s\n", camera2scan_yaw_);
        main_thread_ = make_shared<std::thread>(std::bind(&TargetFollow::mainProcessLoop, this, frequency));
    }
    ~TargetFollow(){
        if(main_thread_.use_count() > 0) main_thread_->join();
    }

    void scanCallback(const sensor_msgs::PointCloud::ConstPtr &msg){  //三维点云处理 PointCloud2是点云数据格式，需要转成pcl格式更方便处理
        // shared_ptr<sensor_msgs::PointCloud> temp_cloud = make_shared<sensor_msgs::PointCloud>();
        

        pcl::PointCloud<PointType> pointcloud;//模板点云类，模板参数是点的格式
        pcl::fromROSMsg(*msg, pointcloud);//把ROS消息转化成pcl点云格式

        Eigen::Matrix3d camInRefer;
        camInRefer << FX, 0, CX,
                      0, FY, CY,
                      0,  0,  1;
        Eigen::Matrix3d pixel;
        Eigen::Matrix3d lidar_coord;
        Eigen::Matrix3d world_coord;

        for (uint i = 0; i < pointcloud.points.size(); i++){//遍历点云的每一个点，uint表示无符号的ints
            // pointcloud.points[i];//每一个点
            geometry_msgs::TransformStamped transform = buffer_->lookupTransform(world_frame_id, lidar_frame_id,ros::Time().fromSec(pointcloud.points[i].x),ros::Duration(0.01)); //通过slam计算得出的雷达在世界坐标系下的坐标，得到转换关系T矩阵
            Eigen::Isometry3d eigen_transform = tf2::transformToEigen(transform); //通过Eigen将T矩阵转换成单位向量矩阵
            lidar_coord << pointcloud.points[i].x, pointcloud.points[i].y, pointcloud.points[i].z;//雷达坐标系点的坐标
            world_coord = eigen_transform * lidar_coord; //计算每一个点云在世界坐标系下的坐标
            
            //3DTo2D
            pixel = camInRefer * eigen_transform * world_coord; //每一个点云的世界坐标转换为像素坐标 
            double u = pixel[0];
            double v = pixel[1];

            int radiusCircle = 30;
            cv::Point centerCircle(u, v);
            cv::Scalar colorCircle(0, 255, 0);
            cv::circle(image, centerCircle, radiusCircle, colorCircle, cv::FILLED); 

            //2DTo3D
            

        }
        // point_cloud_world_ = temp_cloud;



    }

    //2DTo3D
    void detectCallback(const nav_follow::BoundingBoxes::ConstPtr &msg){   //检测回调函数，定义输入取msg的位置
        if(msg->bounding_boxes.size()>0){        //bounding_box数>0
            vector<tf2::Vector3> rays;           //定义像素射线容器
            geometry_msgs::TransformStamped cam2odom;
            if(!buffer_->canTransform(odom_frame_id, camera_frame_id, msg->header.stamp, ros::Duration(0))) return;
            try{
                cam2odom = buffer_->lookupTransform(odom_frame_id, camera_frame_id, msg->header.stamp, ros::Duration(0)); //查询odomid和图像id
            }
            catch(tf2::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }

            //bbox的中心坐标和高
            for(size_t i=0; i < msg->bounding_boxes.size(); i++){
                double px = (msg->bounding_boxes[0].xmin + msg->bounding_boxes[0].xmax) / 2.0; 
                double py = (msg->bounding_boxes[0].ymin + msg->bounding_boxes[0].ymax) / 2.0;
                double h = (msg->bounding_boxes[0].ymax - msg->bounding_boxes[0].ymin);

                geometry_msgs::Vector3Stamped ray_stamped;
                ray_stamped.header.stamp = msg->header.stamp;
                ray_stamped.header.frame_id = camera_frame_id;
                ray_stamped.vector.x = 1;
                ray_stamped.vector.y = -(px - CX) / FX;
                ray_stamped.vector.z = -(py - CY) / FY;
                ray_stamped = buffer_->transform(ray_stamped, odom_frame_id, ros::Duration(0));
                tf2::Vector3 vec(ray_stamped.vector.x, ray_stamped.vector.y, ray_stamped.vector.z);

                vec.normalize();
                rays.push_back(vec);
                ROS_INFO("vec3.x=%f, vec3.y=%f, vec3.z=%f", vec.x(), vec.y(), vec.z());
            }
            vector<shared_ptr<tf2::Vector3>> point_match(rays.size());
            vector<int> point_match_id(rays.size(), -1);
            vector<double> detec_min_dist2(rays.size(), std::numeric_limits<double>::max());
            weak_ptr<sensor_msgs::PointCloud> cloud_weak_ptr(point_cloud_);
            auto point_cloud = cloud_weak_ptr.lock();

            
            for(size_t i=0; i < point_cloud->points.size(); i++){ //遍历点云
                double point_min_dist2 = std::numeric_limits<double>::max();
                int detec_match_id = -1;
                shared_ptr<tf2::Vector3> point_vector = make_shared<tf2::Vector3>(point_cloud->points[i].x-cam2odom.transform.translation.x, point_cloud->points[i].y-cam2odom.transform.translation.y, point_cloud->points[i].z-cam2odom.transform.translation.z);
                for(size_t j=0; j<rays.size(); j++){  //遍历像素射线容器
                    double inner_prod = rays[j].dot(*point_vector);
                    if(inner_prod < 0) continue;
                    double vertical_dist2 = point_vector->length2() - pow(inner_prod, 2);
                    if(vertical_dist2 < point_min_dist2){
                        point_min_dist2 = vertical_dist2;
                        detec_match_id = j;
                    }
                }
                if( !(detec_match_id < 0)){
                    if (point_min_dist2 < detec_min_dist2[detec_match_id] && point_min_dist2 < MATCH_THRESHOLD){
                        point_match[detec_match_id] = point_vector;
                        point_match_id[detec_match_id] = i;
                        detec_min_dist2[detec_match_id] = point_min_dist2;
                    }
                }
            }
            shared_ptr<sensor_msgs::PointCloud> target_cloud = make_shared<sensor_msgs::PointCloud>();
            for(size_t j = 0; j < rays.size(); j++){
                if(point_match_id[j] > 0 && detec_min_dist2[j] < MATCH_THRESHOLD){
                    target_cloud->header = msg->header;
                    target_cloud->header.frame_id = odom_frame_id;
                    geometry_msgs::Point32 point;
                    point.x = point_cloud->points[point_match_id[j]].x;
                    point.y = point_cloud->points[point_match_id[j]].y;
                    point.z = point_cloud->points[point_match_id[j]].z;
                    target_cloud->points.push_back(point);
                }
                else ROS_INFO("Taeget isn't in scan filed %f\n", detec_min_dist2[j]);
            }
            if(target_cloud->points.size()>0){
                point_pub_.publish(*target_cloud);
                last_update_ = ros::Time::now();
            }
            target_point_cloud_ = target_cloud;
        }
    }
    void mainProcessLoop(double frequency){//执行跟踪线程
        static bool flag = false;//导航成功的标志量
        while(!ac_->waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ros::Rate loop_rate(frequency);
        while(ros::ok()){
            ROS_INFO("mainProcessLoop!\n");
            weak_ptr<sensor_msgs::PointCloud> cloud_weak_ptr(target_point_cloud_);
            auto point_cloud = cloud_weak_ptr.lock();
            
            geometry_msgs::TransformStamped base2odom;
            try{
                base2odom = buffer_->lookupTransform(odom_frame_id, base_frame_id, ros::Time::now(), ros::Duration(0));
            }
            catch(tf2::TransformException ex){
                ROS_ERROR("%s", ex.what());
            }
            double dist = 0;
            double yaw = 0;
            if( point_cloud->points.size()>0 ){
                double dx = point_cloud->points[0].x - base2odom.transform.translation.x;
                double dy = point_cloud->points[0].y - base2odom.transform.translation.y;
                 
                dist = sqrt( dx * dx + dy * dy);
                yaw = atan2(dy, dx);
            }
            move_base_msgs::MoveBaseGoal goal;
            if(dist > 1.1*reference_ || ! flag){
                goal.target_pose.header.frame_id = odom_frame_id;
                goal.target_pose.header.stamp = point_cloud->header.stamp;
                goal.target_pose.pose.position.x = (dist - reference_) * cos(yaw);
                goal.target_pose.pose.position.y = (dist - reference_) * sin(yaw);
                tf2::Quaternion q; q.setRPY(0,0,yaw);
                goal.target_pose.pose.orientation.x = q.getX();
                goal.target_pose.pose.orientation.y = q.getY();
                goal.target_pose.pose.orientation.z = q.getZ();
                goal.target_pose.pose.orientation.w = q.getW();
                ac_->sendGoal(goal);
            }
            else{
                ac_->waitForResult();
                if(ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    flag = true;
                    ROS_INFO("Reached goal!\n");
                }
                else {
                    ROS_INFO("Failed!\n");
                }
            }
            // ac_->cancelAllGoals();//cancel all goals
            loop_rate.sleep();
        }
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, "nav");
    ROS_INFO("Follow begin!\n");
    ros::NodeHandle nh("");
    TargetFollow ttf(nh, 10);

    ros::spin();
    return 0;
}