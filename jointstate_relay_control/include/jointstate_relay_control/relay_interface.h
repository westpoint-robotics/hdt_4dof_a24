#ifndef relay_interface_h
#define relay_interface_h

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <map>
#include <mutex>
#include <queue>

#define MAX_DRIVES  127

class RelayInterface : public hardware_interface::RobotHW
{
public:
  RelayInterface(std::string robot_name, bool fake_execution=false);
  ~RelayInterface();
  
  bool init(ros::NodeHandle& nh);
  bool read();
  bool write();
  bool reset();
  
private:
  std::string robot_name_;
  ros::Time last_receipt, last_read;
  // std::string relay_sub_topic, relay_pub_topic;
  
  void msgCallback(const sensor_msgs::JointState& msg);
  
  bool processMsg(const sensor_msgs::JointState *msg);

  std::map<std::string, ros::Subscriber> msg_sub_;
  std::map<std::string, ros::Publisher> msg_pub_;
  
  hardware_interface::JointStateInterface state_interface_;
  hardware_interface::PositionJointInterface pos_interface_;
  hardware_interface::VelocityJointInterface vel_interface_;
  
  std::map<int, bool> addr_map_;
  std::map<int, std::string> namspace_map_;
  std::vector<std::string> namespace_list_;
  
  std::map<int, std::string> addr_to_name_map;
  std::map<std::string,int> name_to_add_map;
  
  double cmd_pos_[MAX_DRIVES];
  double cmd_vel_[MAX_DRIVES];
  double pos_[MAX_DRIVES];
  double vel_[MAX_DRIVES];
  double eff_[MAX_DRIVES];
  
  int drive_type_[MAX_DRIVES];
  
  enum joint_modes
  {
    NONE_MODE,
    JOINT_POSITION_MODE,
    JOINT_VELOCITY_MODE
  };
  joint_modes joint_mode_current[MAX_DRIVES];
  
  bool fake_execution_;
  bool control_enabled_;
  
  std::mutex mutex_;
  std::queue<sensor_msgs::JointState> msg_queue_;
  
  // variables for services 
  // bool hdt_service_enable_;
  
  // bool read_drive_mode(int addr);
  // bool set_drive_mode(int addr, joint_modes new_mode);

};

#endif // relay_interface_h
