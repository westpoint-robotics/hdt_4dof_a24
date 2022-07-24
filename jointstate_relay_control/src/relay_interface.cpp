#define _USE_MATH_DEFINES

#include <jointstate_relay_control/relay_interface.h>
#include <sensor_msgs/JointState.h>

#include <math.h>
#include <sstream>

static const std::string SUB_TOPIC = "/arm/joints";
static const std::string PUB_TOPIC = "/arm/joints/goal";

/*----------------------------------------------------------------------------
 * constructor
 *----------------------------------------------------------------------------*/
RelayInterface::RelayInterface(std::string robot_name, bool fake_execution): robot_name_(robot_name), fake_execution_(fake_execution), control_enabled_(false)
{
  ros::Time ts = ros::Time::now();
  last_receipt = ts;
  last_read = ts;
  
  // initialize data
  for (int addr = 0; addr < MAX_DRIVES; addr++)
  {
    cmd_pos_[addr] = 0.0;
    cmd_vel_[addr] = std::nanf(""); // assume joint starts in position mode
    pos_[addr] = 0.0;
    vel_[addr] = 0.0;
    eff_[addr] = 0.0;
    
    // assume joint starts in position mode
    joint_mode_current[addr] = NONE_MODE;
  }
}

/*----------------------------------------------------------------------------
 * destructor
 *----------------------------------------------------------------------------*/
RelayInterface::~RelayInterface()
{
}

/*----------------------------------------------------------------------------
 * msg callback
 *----------------------------------------------------------------------------*/
void RelayInterface::msgCallback(const sensor_msgs::JointState& msg)
{
  //ROS_INFO("RelayInterface:msgCallback");
  
  // lock mutex
  std::lock_guard<std::mutex> lock(mutex_);
  
  last_receipt = ros::Time::now();

  // push to msg queue
  int count = msg.position.size();
  for (int i = 0; i < count; i++)
  {
    pos_[i+1] = msg.position[i] ;
  }

  count = msg.effort.size();
  for (int i = 0; i < count; i++)
  {
    eff_[i+1] = msg.effort[i];
  }
  
  count = msg.velocity.size();
  for (int i = 0; i < count; i++)
  {
    vel_[i+1] = msg.velocity[i];
  }
}

/*----------------------------------------------------------------------------
 * init
 *----------------------------------------------------------------------------*/
bool RelayInterface::init(ros::NodeHandle& nh)
{ 
  //ROS_INFO("RelayInterface:init robot name = %s", robot_name_.c_str());
  
  ros::NodeHandle robot_nh(nh, robot_name_);
  // std::string relay_pub_topic = "/arm/joints/goal";
  // std::string relay_sub_topic = "/arm/joints";
  
  // get joints for given robot
  std::vector<std::string> joints;
  if (!robot_nh.getParam("hardware_interface/joints", joints))
  {
    ROS_ERROR("RelayInterface:init could not get joints param");
    return false;
  }
  
  //ROS_INFO("RelayInterface:init found %d joints", (int)joints.size());

  // iterate through joints
  for (std::vector<std::string>::const_iterator joint = joints.begin(); joint != joints.end(); ++joint)
  {
    // look for id param associated with joint
    std::stringstream addr_param;
    addr_param << "hardware_interface/" << *joint << "/addr";
    
    // check for valid addr
    int addr;
    if (robot_nh.getParam(addr_param.str(), addr))
    {
     if ((addr > 0) && (addr < MAX_DRIVES))
     {
        //ROS_INFO("RelayInterface:init registering %s with id %d", joint->c_str(), id);
        // set enabled to false initially
        addr_map_[addr] = false;
        
        // connect and register state interface
        hardware_interface::JointStateHandle state_handle(*joint, &pos_[addr], &vel_[addr], &eff_[addr]);
        state_interface_.registerHandle(state_handle);

        // connect and register position interface
        hardware_interface::JointHandle pos_handle(state_interface_.getHandle(*joint), &cmd_pos_[addr]);
        pos_interface_.registerHandle(pos_handle);
        
        // connect and register velocity interface
        hardware_interface::JointHandle vel_handle(state_interface_.getHandle(*joint), &cmd_vel_[addr]);
        vel_interface_.registerHandle(vel_handle);
        
        /*
        // tyring to read joint modes - does not get populated by position or velocity controllers
        hardware_interface::JointCommandModes test_mode;
        hardware_interface::JointModeHandle mode_handle(*joint,&test_mode);
        mode_handle_[addr] = mode_handle;
        ROS_INFO("RelayInterface:init mode handle getName %s",mode_handle_[addr].getName().c_str());
        ROS_INFO("RelayInterface:init mode handle getModeName %s",mode_handle_[addr].getModeName(mode_handle_[addr].getMode()).c_str());
        */
        
        // build map of joint names and CAN ID's
        addr_to_name_map.insert(std::pair<int, std::string>(addr,*joint));
        name_to_add_map.insert(std::pair<std::string,int>(*joint,addr));
      }
      // report invalid id
      else
      {
        ROS_ERROR("RelayInterface:init invalid addr = %d", addr);
        return false;
      }
    }
    else
    {
      ROS_ERROR("RelayInterface:init could not find param %s", addr_param.str().c_str());
      return false;
    }

    // look for namespace param associated with joint
    std::stringstream namespace_param;
    namespace_param << "hardware_interface/" << *joint << "/namespace";

    // check for namespace
    std::string temp_namespace;
    if (robot_nh.getParam(namespace_param.str(), temp_namespace))
    {
      //ROS_INFO("RelayInterface:init found param %s is %s", namespace_param.str().c_str(),temp_namespace.c_str());
      namspace_map_[addr] = temp_namespace;
      if(std::find(namespace_list_.begin(), namespace_list_.end(), temp_namespace) != namespace_list_.end()){
        // namespace is already in the list?
      }else{
        namespace_list_.push_back(temp_namespace);
      }
    }
    else
    {
      // namespace is optional for now, if not specfied in hardware_interface.yaml file assume there is no namespace
      // use empty string as place holder for now
      namspace_map_[addr] = "";
      if(std::find(namespace_list_.begin(), namespace_list_.end(), "") != namespace_list_.end()){
        // namespace is already in the list?
      }else{
        namespace_list_.push_back("");
      }
      //ROS_ERROR("RelayInterface:init could not find param %s", namespace_param.str().c_str());
      //return false;
    }
  }
  
  // register interfaces
  registerInterface(&state_interface_);
  registerInterface(&pos_interface_);
  registerInterface(&vel_interface_);
  
  // link to hardware topics, or warn about fake execution
  if (fake_execution_)
  {
    ROS_WARN("RelayInterface:init fake execution enabled");
  }    
  else
  {
    ROS_INFO("\tsubscriber %s",SUB_TOPIC.c_str());
    msg_sub_[SUB_TOPIC] = nh.subscribe(SUB_TOPIC.c_str(), 10, &RelayInterface::msgCallback, this);

    // publishers
    ROS_INFO("\tpublisher %s",PUB_TOPIC.c_str());
    msg_pub_[PUB_TOPIC] = nh.advertise<sensor_msgs::JointState>(PUB_TOPIC.c_str(), 1);
  }
  
  return true;
}

/*----------------------------------------------------------------------------
 * read
 *----------------------------------------------------------------------------*/
bool RelayInterface::read()
{
  // update "telem" for fake execution
  if (fake_execution_)
  {
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      pos_[addr] = cmd_pos_[addr];
      vel_[addr] = 0.0;
      eff_[addr] = 0.0;
    }
    
    return true;
  }

  control_enabled_ = last_receipt > last_read;
  last_read = ros::Time::now();
  
  return control_enabled_;
}
 
 /*----------------------------------------------------------------------------
 * write
 *----------------------------------------------------------------------------*/
bool RelayInterface::write()
{ 
  if ((!fake_execution_) && (control_enabled_))
  {
    sensor_msgs::JointState goal;
    goal.header.stamp = ros::Time::now();

    int count = addr_map_.size();
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      goal.position.push_back(cmd_pos_[addr]);
    }

    msg_pub_[PUB_TOPIC].publish(goal);
    // ROS_INFO("publishing = %f, %f, %f, %f", cmd_pos_[1], cmd_pos_[2], cmd_pos_[3], cmd_pos_[4]);
    
    return true;
  }
  
  return false;
}

 /*----------------------------------------------------------------------------
 * reset - set commanded position back to telem position
 *----------------------------------------------------------------------------*/
bool RelayInterface::reset()
{ 
  if ((!fake_execution_) && (control_enabled_))
  {
    // iterate through address list
    for(std::map<int, bool>::iterator it = addr_map_.begin(); it != addr_map_.end(); ++it)
    {
      int addr = it->first;
      
      //ROS_INFO("RelayInterface::reset drive %d cmd %f telem %f", addr,cmd_pos_[addr],pos_[addr]);
      cmd_pos_[addr] = pos_[addr];
      cmd_vel_[addr] = std::nanf("");
      // assume joint starts in position mode
      joint_mode_current[addr] = NONE_MODE;
    }
    
    return true;
  }
  
  return false;
}
