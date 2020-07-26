#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class ClosestObstacle 
{
    public:
        ClosestObstacle();
        void callback(const sensor_msgs::LaserScan::ConstPtr& scan);
        bool changing_path = false;
        int min_distance = 0;
        int speed;
    private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};


nh.getParam("/global_name", global_name)

void ClosestObstacle::callback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}