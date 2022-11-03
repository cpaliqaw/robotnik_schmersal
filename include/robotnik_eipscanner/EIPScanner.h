#ifndef ROBOTNIK_EIPSCANNER_EIPSCANNER_H_
#define ROBOTNIK_EIPSCANNER_EIPSCANNER_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <mutex>
#include <bitset>

#include <ParameterObject.h>

#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/Register.h>
#include <robotnik_msgs/Registers.h>
#include <robotnik_msgs/set_digital_output.h>
#include <rcomponent/rcomponent.h>

#include "robotnik_eipscanner/Ps_C1_C100_Input.h"

namespace robotnik_eipscanner {
class EIPScanner : public rcomponent::RComponent
{
private:
  /* EIPScanner stuff */
  string ip_address_;

public:
  EIPScanner(ros::NodeHandle h);
  virtual ~EIPScanner();
protected:
  /* RComponent methods */

  //! Setups all the ROS' stuff
  virtual int rosSetup();

  /* RComponent methods !*/

  /* RComponent stuff */

  //! Public node handle, to receive data
  ros::NodeHandle nh_;
  //! Private node handle, to read params and publish data
  ros::NodeHandle pnh_;

  /* RComponent stuff */

  //! Actions performed on init state
  virtual void initState();
  //! Actions performed on standby state
  virtual void standbyState();
  //! Actions performed on ready state
  virtual void readyState();
  //! Actions performed on the emergency state
  virtual void emergencyState();
  //! Actions performed on Failure state
  virtual void failureState();
  //! Reads data a publish several info into different topics
  virtual void rosPublish();
  //! Reads params from params server
  virtual void rosReadParams();
  virtual int rosShutdown();

private:

  /* RComponent stuff !*/

  /* ROS Stuff */

  // Publishers


  //! Publish raw input to a topic
  ros::Publisher inputs_pub_;
  string inputs_pub_topic_name_;
  robotnik_msgs::inputs_outputs inputs_msg_;
  // Lock access to the input data while writing
  // and publishing a single time
  std::mutex message_mutex_;

  std::bitset<kPs_C1_C100_OutputSize> output_bits_;
  ros::ServiceServer set_digital_output_service_;
  string set_digital_output_service_name_;
  // std::mutex digital_output_mutex_; use one mutex to avoid connection refusals from the PLC?

  robotnik_msgs::Registers registers_;

  bool SetDigitalOutputServiceCallback(robotnik_msgs::set_digital_output::Request& req,
                                robotnik_msgs::set_digital_output::Response& res);
  

  /* EIPScanner stuff */
  Ps_C1_C100_Input ps_c1_c100_input_;
  void Discover();
  void GetIdentityObject();
  void PollState();
  void WriteState();
  void SendExplicitMessage(Instance instance, eipScanner::cip::ServiceCodes service_code);
};
} // namespace robotnik_eipscanner

#endif  // ROBOTNIK_EIPSCANNER_EIPSCANNER_H_
