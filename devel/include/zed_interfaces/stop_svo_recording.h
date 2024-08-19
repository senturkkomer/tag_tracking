// Generated by gencpp from file zed_interfaces/stop_svo_recording.msg
// DO NOT EDIT!


#ifndef ZED_INTERFACES_MESSAGE_STOP_SVO_RECORDING_H
#define ZED_INTERFACES_MESSAGE_STOP_SVO_RECORDING_H

#include <ros/service_traits.h>


#include <zed_interfaces/stop_svo_recordingRequest.h>
#include <zed_interfaces/stop_svo_recordingResponse.h>


namespace zed_interfaces
{

struct stop_svo_recording
{

typedef stop_svo_recordingRequest Request;
typedef stop_svo_recordingResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct stop_svo_recording
} // namespace zed_interfaces


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::zed_interfaces::stop_svo_recording > {
  static const char* value()
  {
    return "784b6c45ec0bd93cee43c7cd15145736";
  }

  static const char* value(const ::zed_interfaces::stop_svo_recording&) { return value(); }
};

template<>
struct DataType< ::zed_interfaces::stop_svo_recording > {
  static const char* value()
  {
    return "zed_interfaces/stop_svo_recording";
  }

  static const char* value(const ::zed_interfaces::stop_svo_recording&) { return value(); }
};


// service_traits::MD5Sum< ::zed_interfaces::stop_svo_recordingRequest> should match
// service_traits::MD5Sum< ::zed_interfaces::stop_svo_recording >
template<>
struct MD5Sum< ::zed_interfaces::stop_svo_recordingRequest>
{
  static const char* value()
  {
    return MD5Sum< ::zed_interfaces::stop_svo_recording >::value();
  }
  static const char* value(const ::zed_interfaces::stop_svo_recordingRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::zed_interfaces::stop_svo_recordingRequest> should match
// service_traits::DataType< ::zed_interfaces::stop_svo_recording >
template<>
struct DataType< ::zed_interfaces::stop_svo_recordingRequest>
{
  static const char* value()
  {
    return DataType< ::zed_interfaces::stop_svo_recording >::value();
  }
  static const char* value(const ::zed_interfaces::stop_svo_recordingRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::zed_interfaces::stop_svo_recordingResponse> should match
// service_traits::MD5Sum< ::zed_interfaces::stop_svo_recording >
template<>
struct MD5Sum< ::zed_interfaces::stop_svo_recordingResponse>
{
  static const char* value()
  {
    return MD5Sum< ::zed_interfaces::stop_svo_recording >::value();
  }
  static const char* value(const ::zed_interfaces::stop_svo_recordingResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::zed_interfaces::stop_svo_recordingResponse> should match
// service_traits::DataType< ::zed_interfaces::stop_svo_recording >
template<>
struct DataType< ::zed_interfaces::stop_svo_recordingResponse>
{
  static const char* value()
  {
    return DataType< ::zed_interfaces::stop_svo_recording >::value();
  }
  static const char* value(const ::zed_interfaces::stop_svo_recordingResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ZED_INTERFACES_MESSAGE_STOP_SVO_RECORDING_H
