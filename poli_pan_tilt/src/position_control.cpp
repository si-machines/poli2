/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#include "poli_pan_tilt/position_control.h"
#include "sensor_msgs/JointState.h"

using namespace position_control;

PositionControl::PositionControl()
    :node_handle_(""),
     node_handle_priv_("~"),
     profile_velocity_(0),
     profile_acceleration_(0)
{
  if (loadDynamixel())
  {
    checkLoadDynamixel();
  }
  else
  {
    ROS_ERROR("Cant' Load Dynamixel, Please check Parameter");
  }

  if (!multi_driver_->initSyncWrite())
    ROS_INFO("Pan-init SyncWrite Failed!");

  writeValue_ = new WriteValue;

  setTorque(true);

  if (multi_driver_->getProtocolVersion() == 2.0 &&
      !(multi_driver_->multi_dynamixel_[0]->model_name_.find("PRO") != std::string::npos))
  {
    setProfileValue(profile_velocity_, profile_acceleration_);
  }

  initDynamixelStatePublisher();
  initDynamixelInfoServer();
}

PositionControl::~PositionControl()
{
  setTorque(false);

  ros::shutdown();
}

bool PositionControl::loadDynamixel()
{
  bool ret = false;

  dynamixel_driver::DynamixelInfo *pan_info = new dynamixel_driver::DynamixelInfo;

  pan_info->lode_info.device_name      = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  pan_info->lode_info.baud_rate        = node_handle_.param<int>("baud_rate", 57600);
  pan_info->lode_info.protocol_version = node_handle_.param<float>("protocol_version", 2.0);

  pan_info->model_id                   = node_handle_.param<int>("pan_id", 1);

  dynamixel_info_.push_back(pan_info);

  node_handle_priv_.getParam("profile_velocity", profile_velocity_);
  node_handle_priv_.getParam("profile_acceleration", profile_acceleration_);

  multi_driver_ = new dynamixel_multi_driver::DynamixelMultiDriver(dynamixel_info_[MOTOR]->lode_info.device_name,
                                                                   dynamixel_info_[MOTOR]->lode_info.baud_rate,
                                                                   dynamixel_info_[MOTOR]->lode_info.protocol_version);

 ret =  multi_driver_->loadDynamixel(dynamixel_info_);

 return ret;
}

bool PositionControl::setTorque(bool onoff)
{
  writeValue_->torque.clear();
  writeValue_->torque.push_back(onoff);
  writeValue_->torque.push_back(onoff);

  if (!multi_driver_->syncWriteTorque(writeValue_->torque))
  {
    ROS_ERROR("SyncWrite Torque Failed!");
    return false;
  }

  return true;
}

bool PositionControl::setProfileValue(uint32_t prof_vel, uint32_t prof_acc)
{
  writeValue_->prof_vel.clear();
  writeValue_->prof_acc.clear();

  writeValue_->prof_vel.push_back(prof_vel);
  writeValue_->prof_vel.push_back(prof_vel);

  writeValue_->prof_acc.push_back(prof_acc);
  writeValue_->prof_acc.push_back(prof_acc);

  if (!multi_driver_->syncWriteProfileVelocity(writeValue_->prof_vel))
  {
    ROS_ERROR("SyncWrite Profile Velocity Failed!");
    return false;
  }

  if (!multi_driver_->syncWriteProfileAcceleration(writeValue_->prof_acc))
  {
    ROS_ERROR("SyncWrite Profile Acceleration Failed!");
    return false;
  }

  return true;
}

bool PositionControl::setPosition(uint32_t pan_pos)
{
  writeValue_->pos.clear();
  writeValue_->pos.push_back(pan_pos);

  if (!multi_driver_->syncWritePosition(writeValue_->pos))
  {
    ROS_ERROR("SyncWrite Position Failed!");
    return false;
  }

  return true;
}

bool PositionControl::checkLoadDynamixel()
{
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("  dynamixel_workbench controller; position control example(Pan & Tilt) ");
  ROS_INFO("-----------------------------------------------------------------------");
  ROS_INFO("PAN MOTOR INFO");
  ROS_INFO("ID    : %d", dynamixel_info_[0]->model_id);
  ROS_INFO("MODEL : %s", dynamixel_info_[0]->model_name.c_str());
  ROS_INFO(" ");
  if (multi_driver_->getProtocolVersion() == 2.0 &&
      !(multi_driver_->multi_dynamixel_[0]->model_name_.find("PRO") != std::string::npos))
  {
    ROS_INFO(" ");
    ROS_INFO("Profile Velocity     : %d", profile_velocity_);
    ROS_INFO("Profile Acceleration : %d", profile_acceleration_);
  }
  ROS_INFO("-----------------------------------------------------------------------");
}

bool PositionControl::initDynamixelStatePublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<sensor_msgs::JointState>("/pan_motor/joint_state", 10);
}

bool PositionControl::initDynamixelInfoServer()
{
  joint_command_sub = node_handle_.subscribe("/pan_motor/position_controller/command",1, &PositionControl::jointCommandSubCallback, this);
}

bool PositionControl::readDynamixelState()
{
  multi_driver_->readMultiRegister("torque_enable");

  multi_driver_->readMultiRegister("present_position");

  multi_driver_->readMultiRegister("goal_position");
  multi_driver_->readMultiRegister("moving");

  if (multi_driver_->getProtocolVersion() == 2.0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->model_name_.find("XM") != std::string::npos)
    {
      multi_driver_->readMultiRegister("goal_current");

      multi_driver_->readMultiRegister("present_current");
    }
    multi_driver_->readMultiRegister("goal_velocity");
    multi_driver_->readMultiRegister("present_velocity");
  }
  else
  {
    multi_driver_->readMultiRegister("moving_speed");
    multi_driver_->readMultiRegister("present_speed");
  }
}

bool PositionControl::dynamixelStatePublish()
{
  readDynamixelState();

  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[multi_driver_->multi_dynamixel_.size()];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;
  
  sensor_msgs::JointState js;
  js.header.frame_id = "pan_joint";
  js.name.push_back("pan_joint");

  for (std::vector<dynamixel_tool::DynamixelTool *>::size_type num = 0; num < multi_driver_->multi_dynamixel_.size(); ++num)
  {
    dynamixel_state[num].model_name          = multi_driver_->multi_dynamixel_[num]->model_name_;
    dynamixel_state[num].id                  = multi_driver_->multi_dynamixel_[num]->id_;
    dynamixel_state[num].torque_enable       = multi_driver_->read_value_["torque_enable"]      ->at(num);
    dynamixel_state[num].present_position    = multi_driver_->read_value_["present_position"]   ->at(num);
    dynamixel_state[num].goal_position       = multi_driver_->read_value_["goal_position"]      ->at(num);
    dynamixel_state[num].moving              = multi_driver_->read_value_["moving"]             ->at(num);

    js.position.push_back(convertValue2Radian(dynamixel_state[num].present_position));
    js.velocity.push_back(dynamixel_state[num].moving);
    js.effort.push_back(dynamixel_state[num].present_current);

    if (multi_driver_->getProtocolVersion() == 2.0)
    {
      if (multi_driver_->multi_dynamixel_[MOTOR]->model_name_.find("XM") != std::string::npos)
      {
        dynamixel_state[num].goal_current    = multi_driver_->read_value_["goal_current"]   ->at(num);
        dynamixel_state[num].present_current = multi_driver_->read_value_["present_current"]->at(num);
      }
      dynamixel_state[num].goal_velocity    = multi_driver_->read_value_["goal_velocity"]->at(num);
      dynamixel_state[num].present_velocity = multi_driver_->read_value_["present_velocity"]->at(num);
    }
    else
    {
      dynamixel_state[num].goal_velocity    = multi_driver_->read_value_["moving_speed"]->at(num);
      dynamixel_state[num].present_velocity = multi_driver_->read_value_["present_speed"]->at(num);
    }

    dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[num]);
  }
  js.header.stamp = ros::Time::now();
  dynamixel_state_list_pub_.publish(js);

}

uint32_t PositionControl::convertRadian2Value(float radian)
{
  uint32_t value = 0;

  if (radian > 0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ <= multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
      return multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_;

    value = (radian * (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) / multi_driver_->multi_dynamixel_[MOTOR]->max_radian_)
                + multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;
  }
  else if (radian < 0)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ >= multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
      return multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_;

    value = (radian * (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) / multi_driver_->multi_dynamixel_[MOTOR]->min_radian_)
                + multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;
  }
  else
    value = multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_;

  return value;
}

float PositionControl::convertValue2Radian(int32_t value)
{
  float radian = 0.0;

 
  if (value > multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->max_radian_ <= 0)
      return multi_driver_->multi_dynamixel_[MOTOR]->max_radian_;

    radian = (float) (value - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) * multi_driver_->multi_dynamixel_[MOTOR]->max_radian_
               / (float) (multi_driver_->multi_dynamixel_[MOTOR]->value_of_max_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_);
  }
  else if (value < multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_)
  {
    if (multi_driver_->multi_dynamixel_[MOTOR]->min_radian_ >= 0)
      return multi_driver_->multi_dynamixel_[MOTOR]->min_radian_;

    radian = (float) (value - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_) * multi_driver_->multi_dynamixel_[MOTOR]->min_radian_
               / (float) (multi_driver_->multi_dynamixel_[MOTOR]->value_of_min_radian_position_ - multi_driver_->multi_dynamixel_[MOTOR]->value_of_0_radian_position_);
  }
 
  return radian;
}

bool PositionControl::controlLoop()
{
  dynamixelStatePublish();
}

void PositionControl::jointCommandSubCallback(const std_msgs::Float64::ConstPtr& msg){
  //Assume radian input. Convert to motor units
  setPosition(convertRadian2Value(msg->data));
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "pan_position_control");
  PositionControl pos_ctrl;

  ros::Rate loop_rate(20);

  while (ros::ok())
  {

    pos_ctrl.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
