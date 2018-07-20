#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <gazebo_msgs/LinkStates.h>
#include <sstream>
#include <math.h> 


namespace gazebo
{
  class ModelPush : public ModelPlugin
  { 
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;

      k = 100;

      lwa4p_blue_position_pub = node.advertise<geometry_msgs::Vector3>("/lwa4p_blue/applied_force", 10);

      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    public: void OnUpdate()
    {
      lwa4p_blue_position_sub = node.subscribe("/gazebo/link_states", 10, &ModelPush::subCallback, this);
      lwa4p_blue_position_pub.publish(pub_msg);
    }

    void subCallback(const gazebo_msgs::LinkStates &msg)
    {
      // pose[8] is currently arm_6, sphere is pose[1]
      // 10 tool link
      double arm_x = msg.pose[10].position.x;
      double arm_y = msg.pose[10].position.y;
      double arm_z = msg.pose[10].position.z;
      
      double sphere_x = msg.pose[1].position.x;
      double sphere_y = msg.pose[1].position.y;
      double sphere_z = msg.pose[1].position.z;
   
      double r = 0.5;

      double force;
      double dx = fabs(arm_x - sphere_x);
      double dy = fabs(arm_y - sphere_y);
      double dz = fabs(arm_z - sphere_z);
      double distance = (pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
      double distance_s = sqrt(distance);
      double delta_r = fabs(r - sqrt(distance));
      
      if(distance < pow(r, 2))
      {
        force = k * delta_r;
        xf = force * dx / distance_s;
        yf = force * dy / distance_s;
        zf = force * dz / distance_s;
      }
      else {
        force = 0.0;
      }
      pub_msg.x = xf;
      pub_msg.y = yf;
      pub_msg.z = zf;
    } 

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: ros::NodeHandle node;
    private: ros::Publisher  lwa4p_blue_position_pub;
    private: ros::Subscriber lwa4p_blue_position_sub;
    private: ros::Publisher lwa4p_blue_sphere;
    
    private: geometry_msgs::Vector3 pub_msg;
    double k, xf, yf, zf;
    
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}