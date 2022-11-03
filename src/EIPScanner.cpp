#include <fstream> 
#include <bitset>
#include <chrono>
#include <thread>
#include <mutex>

#include <DiscoveryManager.h>
#include <ParameterObject.h>
#include <FileObject.h>
#include <utils/Logger.h>
#include <ConnectionManager.h>
#include <cip/connectionManager/NetworkConnectionParams.h>
#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_eipscanner/EIPScanner.h>

#include "robotnik_eipscanner/Ps_C1_C100_Input.h"

using namespace std::chrono_literals;
using eipScanner::DiscoveryManager;

using eipScanner::IdentityObject;
using eipScanner::SessionInfo;
using std::bitset;

using eipScanner::MessageRouter;
using eipScanner::cip::ServiceCodes;
using eipScanner::cip::EPath;
using eipScanner::cip::CipUint;
using eipScanner::cip::CipInt;
using eipScanner::cip::CipByte;
using eipScanner::cip::CipDword;
using eipScanner::cip::GeneralStatusCodes;
using eipScanner::ParameterObject;
using eipScanner::utils::Buffer;
using eipScanner::FileObject;
using eipScanner::utils::Logger;
using eipScanner::utils::LogLevel;
using eipScanner::ConnectionManager;
using eipScanner::cip::connectionManager::ConnectionParameters;
using eipScanner::cip::connectionManager::NetworkConnectionParams;

namespace robotnik_eipscanner {

EIPScanner::EIPScanner(ros::NodeHandle h) : RComponent(h), nh_(h), pnh_("~"), ip_address_("192.168.0.210")
{
  component_name.assign(pnh_.getNamespace());

  readParam(pnh_, "ip_address", ip_address_, ip_address_);
  inputs_pub_topic_name_ = "inputs";

  set_digital_output_service_name_ = "digital_output";

  Logger::setLogLevel(LogLevel::DEBUG);

  rosReadParams();

  inputs_msg_.digital_inputs.assign(robotnik_eipscanner::kPs_C1_C100_InputSize, 1);
  inputs_msg_.digital_outputs.assign(robotnik_eipscanner::kPs_C1_C100_OutputSize, 1);

  Discover();
  GetIdentityObject();
  // std::this_thread::sleep_for(100ms); // without this, 80% of the time you'll get a "Connection refused" error
  // WriteState();
}

EIPScanner::~EIPScanner()
{
}

void EIPScanner::rosReadParams()
{
  bool required = true;
  bool not_required = false;

  readParam(pnh_, "desired_freq", desired_freq_, 1.0, required);
}

int EIPScanner::rosSetup()
{
  RComponent::rosSetup();


  bool required = true;
  bool not_required = false;

  // Publisher
  inputs_pub_ = pnh_.advertise<robotnik_msgs::inputs_outputs>(inputs_pub_topic_name_, 10);
  set_digital_output_service_ =
        pnh_.advertiseService(set_digital_output_service_name_, &EIPScanner::SetDigitalOutputServiceCallback, this);
  
}

int EIPScanner::rosShutdown()
{
  RComponent::rosShutdown();
}

void EIPScanner::rosPublish()
{
  RComponent::rosPublish();

  if (getState() == robotnik_msgs::State::READY_STATE)
  {
    std::lock_guard<std::mutex> inputs_lock_guard(message_mutex_);
    std::this_thread::sleep_for(100ms);
    PollState();
    inputs_pub_.publish(inputs_msg_);
  }
}

void EIPScanner::initState()
{
  RComponent::initState();
  switchToState(robotnik_msgs::State::STANDBY_STATE);
}

void  EIPScanner::standbyState()
{
    RCOMPONENT_INFO_STREAM("standbyState()");
    switchToState(robotnik_msgs::State::READY_STATE);
}

void  EIPScanner::readyState()
{

}

void EIPScanner::failureState()
{

}

void EIPScanner::emergencyState()
{

}

bool EIPScanner::SetDigitalOutputServiceCallback(robotnik_msgs::set_digital_output::Request & req,
                                robotnik_msgs::set_digital_output::Response & res)
{
  RCOMPONENT_WARN_STREAM("Received srv trigger petition.");
  if (state == robotnik_msgs::State::READY_STATE)
  {
    std::lock_guard<std::mutex> guard(message_mutex_);
    std::this_thread::sleep_for(100ms);
    output_bits_.reset();
    output_bits_[req.output] = req.value;
    RCOMPONENT_INFO("Preparing to write state");
    WriteState();
    RCOMPONENT_INFO("Finished writing state");
    output_bits_.reset();
    res.ret = true;
    return true;
  }
  else
  {
    res.ret = false;
    return true;
  }
  // No way to get here
  return false;
}

void EIPScanner::Discover()
{
  RCOMPONENT_INFO("Attempting to discover devices");

  auto receive_timeout = std::chrono::seconds(2);
  DiscoveryManager discoveryManager(ip_address_, kExplicitMessagingDefaultPort, receive_timeout);
  auto devices = discoveryManager.discover();

  if (devices.size() == 0) {
    RCOMPONENT_ERROR_STREAM("No devices found!");
    exit(0);
  }

  for (auto & device : devices) {
    RCOMPONENT_INFO_STREAM("Discovered device: " << device.identityObject.getProductName()
                          << " with address " << device.socketAddress.toString());
  }
}

void EIPScanner::GetIdentityObject()
{
  RCOMPONENT_INFO_STREAM("Attempting to create an identity object");
    try
    {
      RCOMPONENT_INFO_STREAM("Creating a session info object with ip address " << ip_address_ <<
        " and port " << std::hex << kExplicitMessagingDefaultPort);
      auto si = std::make_shared<SessionInfo>(ip_address_, kExplicitMessagingDefaultPort);
      IdentityObject identityObject(1, si);

      RCOMPONENT_INFO_STREAM("Vendor ID: " << identityObject.getVendorId() << " Device Type: "
              << identityObject.getDeviceType() << " Product Code: " << identityObject.getProductCode() 
              << " Revision: "  << identityObject.getRevision().toString() << " Status: "
              << identityObject.getStatus() << " Serial Number: " << identityObject.getSerialNumber()
              << " Product Name: " << identityObject.getProductName());
    }
    catch(const std::system_error & e)
    {
      RCOMPONENT_ERROR_STREAM(e.what());
      exit(0);
    }
}

void EIPScanner::WriteState()
{
  RCOMPONENT_INFO_STREAM("Attempting to write data to the PLC");
  SendExplicitMessage(Instance::kWrite, ServiceCodes::SET_ATTRIBUTE_SINGLE);
}

void EIPScanner::PollState()
{
  RCOMPONENT_INFO_STREAM("Attempting to read data from the PLC");
  SendExplicitMessage(Instance::kRead, ServiceCodes::GET_ATTRIBUTE_SINGLE);
}

void EIPScanner::SendExplicitMessage(Instance instance, ServiceCodes service_code)
{
  if ((service_code != ServiceCodes::GET_ATTRIBUTE_SINGLE) && (service_code != ServiceCodes::SET_ATTRIBUTE_SINGLE)) {
    RCOMPONENT_ERROR_STREAM("Unexpected EtherNet/IP service code: " << std::hex << service_code);
    return;
  }
  try
  {
    RCOMPONENT_INFO_STREAM("Creating a session info object with ip address " << ip_address_ <<
      " and port " << std::hex << kExplicitMessagingDefaultPort);
    auto si = std::make_shared<SessionInfo>(ip_address_, kExplicitMessagingDefaultPort);

    // Read the number of the parameters
    MessageRouter messageRouter;
    // According to DOC_MAN_FIE_installationshandbuch-psc1-feldbusse_SEN_AIN_V7.pdf, p. 93:
    // There are two different Assembly Objects (class ID: 04h) available:
    // Instance 64h(100d) / (PLC -> PSC1)
    // 4 Byte Functional Inputs and 64 Byte SD Bus request
    // Instance 65h(101d) / (PLC <- PSC1)
    // 128 Byte Functional Outputs and 64 Byte SD Bus response

    // the data (attribute 03h ) and the data length (attribute 04h ) can be read or written.
    // From https://eipscanner.readthedocs.io/en/latest/explicit_messaging.html
    // The most typical operations in the explicit communication are reading and writing CIP attributes.
    // The example that we used above is suitable, but we should keep in mind 2 things:
    // 1. Use cip::Epath with Attribute ID which you’re going to read or write an attribute.
    // For an example EPath(1,2,3), where ClassId=1, InstanceId=2, AttributeId=3
    // Use cip::ServiceCodes enum with the common service codes

    const EPath epath = EPath(kPsc1C100_ClassId, (CipUint)instance, (CipUint)Attribute::kData);
    RCOMPONENT_INFO_STREAM("Creating an epath with ClassID=" <<  kPsc1C100_ClassId << " InstanceID=" << (CipUint)instance << " AttributeId=" << (CipUint)Attribute::kData);

    Buffer send_buffer;
    // From a screenshot that appears on
    // DOC_MAN_FIE_installationshandbuch-psc1-feldbusse_SEN_AIN_V7.pdf, p. 93
  
    for (auto i = 0; i < 1; ++i) {
      RCOMPONENT_INFO_STREAM("Pushing byte " << i);
      // bitset<8> data_bits(arg);
      RCOMPONENT_INFO_STREAM("Binary: " << output_bits_ << '\n');
      CipDword dword = output_bits_.to_ulong();
      send_buffer << dword;
    }
    
    // also from p 93:
    // "Reading/Writing of fieldbus data is also possible via Explicit Messaging objects."
    // "With the services Get_Attribute_Single (0Eh/14d) Set_Attribute_Single (10h/16d)""
    RCOMPONENT_INFO_STREAM("Sending a request with service_code=" << service_code);
    auto response = messageRouter.sendRequest(
      si,
      service_code,
      epath,
      send_buffer.data());
    
    if (response.getGeneralStatusCode() != GeneralStatusCodes::SUCCESS) {
      RCOMPONENT_ERROR_STREAM("Failed to read the data");
      logGeneralAndAdditionalStatus(response);
      return;
    }
    RCOMPONENT_INFO_STREAM("The reply service is " << response.getServiceCode());
    logGeneralAndAdditionalStatus(response);

    RCOMPONENT_INFO_STREAM("The size of the data is " << response.getData().size() << " bytes");
    Buffer receive_buffer(response.getData());
    RCOMPONENT_INFO_STREAM("The size of the buffer is " << receive_buffer.size() << " bytes");

    if (service_code == ServiceCodes::SET_ATTRIBUTE_SINGLE) {
      if (!receive_buffer.empty()) {
        RCOMPONENT_ERROR_STREAM("Error: unexpected non-empty data buffer after a write operation");
      }
      return;
    }

    int byte_number = 0;
    while (!receive_buffer.empty()) {
      RCOMPONENT_DEBUG_STREAM("Byte # " << byte_number);
      CipByte byte;
      receive_buffer >> byte;
      RCOMPONENT_DEBUG_STREAM("Decimal: " << (int)byte);
      bitset<kBitsPerByte> data_bits(byte);
      RCOMPONENT_DEBUG_STREAM("Binary: " << data_bits << '\n');

      int bit_number = 0;
      for (auto i = 0; i < kBitsPerByte; ++i) {
        inputs_msg_.digital_inputs[byte_number*kBitsPerByte + bit_number] = data_bits[i];
        bit_number += 1;
      }

      if (byte != 0b0000'0000) {
        RCOMPONENT_DEBUG_STREAM("NOT ZERO");
      }
      byte_number += 1;
    }
  }
  catch(const std::system_error & e)
  {
    RCOMPONENT_ERROR_STREAM(e.what());
    exit(0);
  }
}


} // namespace robotnik_eipscanner