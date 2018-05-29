#include "ros/ros.h"
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/WrenchStamped.h>


using namespace std;

class pointCloudPosition{
    public:
        pointCloudPosition();
        ~pointCloudPosition();
        void run();
    
    private:
        double force;
        double x;
        double y;
        double z;

        ros::Subscriber lwa4pBlueCurrentPositionSub;
        ros::Subscriber lwa4pBlueForceSub;
        ros::Publisher lwa4pBluePointCloudPub;
        
        void lwa4pBlueCurrentPositionCallBack(const std_msgs::Float64MultiArray &msg);
        void lwa4pBlueForceCallBack(const std_msgs::Float64 &msg);
};
