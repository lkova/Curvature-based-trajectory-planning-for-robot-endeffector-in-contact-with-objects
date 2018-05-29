#include <schunk_lwa4p_force/point_cloud_position.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pointCloudPositionNode");

    pointCloudPosition pcp;
    pcp.run();
    return 0;
}