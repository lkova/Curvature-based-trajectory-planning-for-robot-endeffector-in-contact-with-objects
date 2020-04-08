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
#include<time.h>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  { 
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      this->model = _parent;

      k = 100;
      time_check = true;
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
      arm_x = msg.pose[10].position.x;
      arm_y = msg.pose[10].position.y;
      arm_z = msg.pose[10].position.z;
      
      sphere_x = msg.pose[1].position.x;
      sphere_y = msg.pose[1].position.y;
      sphere_z = msg.pose[1].position.z;  

      calculate_force(); 
    } 

    void calculate_force() {
        double r = 0.3;

        double force;
        double dx = fabs(arm_x - sphere_x);
        double dy = fabs(arm_y - sphere_y);
        double dz = fabs(arm_z - sphere_z);
        double distance = (pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        double distance_s = sqrt(distance);
        delta_r = fabs(r - sqrt(distance));

        if(time_check) {
          clock_gettime(CLOCK_REALTIME, &previous_time);
          delta_r_prev = delta_r;
          time_check = false;
        }
        clock_gettime(CLOCK_REALTIME, &current_time);
        td = (current_time.tv_sec - previous_time.tv_sec) + (current_time.tv_nsec - previous_time.tv_nsec) / 1000000000.0;
        double pokemon;
        
        if(distance < pow(r, 2))
        {
          
          double f0 = pow(delta_r, 3) * 1000;
          force = f0 + k * delta_r;
          /*if(force > 1.35 || force < 0.75) {
            printf("force = %lf\n", force);
          }*/

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
        previous_time = current_time;
        delta_r_prev = delta_r;
    }

    private: physics::ModelPtr model;
    private: event::ConnectionPtr updateConnection;
    private: ros::NodeHandle node;
    private: ros::Publisher  lwa4p_blue_position_pub;
    private: ros::Subscriber lwa4p_blue_position_sub;
    private: ros::Publisher lwa4p_blue_sphere;
    
    private: geometry_msgs::Vector3 pub_msg;
    double k, xf, yf, zf, td;
    double arm_x, arm_y, arm_z;
    double sphere_x, sphere_y, sphere_z;
    timespec current_time, previous_time;
    double delta_r, delta_r_prev;
    bool time_check;
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
