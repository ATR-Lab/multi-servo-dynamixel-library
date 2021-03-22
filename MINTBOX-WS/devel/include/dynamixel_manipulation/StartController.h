// Generated by gencpp from file dynamixel_manipulation/StartController.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_MANIPULATION_MESSAGE_STARTCONTROLLER_H
#define DYNAMIXEL_MANIPULATION_MESSAGE_STARTCONTROLLER_H

#include <ros/service_traits.h>


#include <dynamixel_manipulation/StartControllerRequest.h>
#include <dynamixel_manipulation/StartControllerResponse.h>


namespace dynamixel_manipulation
{

struct StartController
{

typedef StartControllerRequest Request;
typedef StartControllerResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct StartController
} // namespace dynamixel_manipulation


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::dynamixel_manipulation::StartController > {
  static const char* value()
  {
    return "94c76c2df56346fcaa2611bdac54f434";
  }

  static const char* value(const ::dynamixel_manipulation::StartController&) { return value(); }
};

template<>
struct DataType< ::dynamixel_manipulation::StartController > {
  static const char* value()
  {
    return "dynamixel_manipulation/StartController";
  }

  static const char* value(const ::dynamixel_manipulation::StartController&) { return value(); }
};


// service_traits::MD5Sum< ::dynamixel_manipulation::StartControllerRequest> should match 
// service_traits::MD5Sum< ::dynamixel_manipulation::StartController > 
template<>
struct MD5Sum< ::dynamixel_manipulation::StartControllerRequest>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_manipulation::StartController >::value();
  }
  static const char* value(const ::dynamixel_manipulation::StartControllerRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_manipulation::StartControllerRequest> should match 
// service_traits::DataType< ::dynamixel_manipulation::StartController > 
template<>
struct DataType< ::dynamixel_manipulation::StartControllerRequest>
{
  static const char* value()
  {
    return DataType< ::dynamixel_manipulation::StartController >::value();
  }
  static const char* value(const ::dynamixel_manipulation::StartControllerRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::dynamixel_manipulation::StartControllerResponse> should match 
// service_traits::MD5Sum< ::dynamixel_manipulation::StartController > 
template<>
struct MD5Sum< ::dynamixel_manipulation::StartControllerResponse>
{
  static const char* value()
  {
    return MD5Sum< ::dynamixel_manipulation::StartController >::value();
  }
  static const char* value(const ::dynamixel_manipulation::StartControllerResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::dynamixel_manipulation::StartControllerResponse> should match 
// service_traits::DataType< ::dynamixel_manipulation::StartController > 
template<>
struct DataType< ::dynamixel_manipulation::StartControllerResponse>
{
  static const char* value()
  {
    return DataType< ::dynamixel_manipulation::StartController >::value();
  }
  static const char* value(const ::dynamixel_manipulation::StartControllerResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // DYNAMIXEL_MANIPULATION_MESSAGE_STARTCONTROLLER_H
