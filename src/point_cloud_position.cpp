#include <schunk_lwa4p_force/point_cloud_position.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

pointCloudPosition::pointCloudPosition(){
    std::string configFile;
    ros::NodeHandle n;
    ros::NodeHandle private_node_handle_("~");

    private_node_handle_.param("config", configFile, string("new_node"));
    
    force = 0.0;
    x = 0.0;
    y = 0.0;
    z = 0.0;
    lwa4pBlueCurrentPositionSub = n.subscribe("/lwa4p_blue/current_position", 100, &pointCloudPosition::lwa4pBlueCurrentPositionCallBack, this);
    lwa4pBlueForceSub = n.subscribe("/lwa4p_blue/applied_force", 100, &pointCloudPosition::lwa4pBlueForceCallBack, this);
    lwa4pBluePointCloudPub = n.advertise<PointCloud>("/lwa4p_blue/points", 100);
}

pointCloudPosition::~pointCloudPosition() {

}

void pointCloudPosition::lwa4pBlueCurrentPositionCallBack(const  std_msgs::Float64MultiArray &msg) {
    x = msg.data[6];
    y = msg.data[7];
    z = msg.data[8];
}

void pointCloudPosition::lwa4pBlueForceCallBack(const std_msgs::Float64 &msg) {
    force = msg.data;
}

void pointCloudPosition::run() {
    ros::Rate r(100);
    bool temp = true;

    PointCloud::Ptr msg(new PointCloud);
    msg->header.frame_id = "map";
    //msg->height = msg->width = 1;

    while (ros::ok()) 
    {
        if(force > 1.0) {
            pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
            msg->points.push_back(pcl::PointXYZ(x, y, z));
            lwa4pBluePointCloudPub.publish(msg);
        }
        
        ros::spinOnce();
        r.sleep();
    }
    
}


