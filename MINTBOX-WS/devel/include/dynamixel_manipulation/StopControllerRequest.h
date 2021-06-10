// Generated by gencpp from file dynamixel_manipulation/StopControllerRequest.msg
// DO NOT EDIT!


#ifndef DYNAMIXEL_MANIPULATION_MESSAGE_STOPCONTROLLERREQUEST_H
#define DYNAMIXEL_MANIPULATION_MESSAGE_STOPCONTROLLERREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace dynamixel_manipulation
{
template <class ContainerAllocator>
struct StopControllerRequest_
{
  typedef StopControllerRequest_<ContainerAllocator> Type;

  StopControllerRequest_()
    : controller_name()  {
    }
  StopControllerRequest_(const ContainerAllocator& _alloc)
    : controller_name(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _controller_name_type;
  _controller_name_type controller_name;





  typedef boost::shared_ptr< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> const> ConstPtr;

}; // struct StopControllerRequest_

typedef ::dynamixel_manipulation::StopControllerRequest_<std::allocator<void> > StopControllerRequest;

typedef boost::shared_ptr< ::dynamixel_manipulation::StopControllerRequest > StopControllerRequestPtr;
typedef boost::shared_ptr< ::dynamixel_manipulation::StopControllerRequest const> StopControllerRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator1> & lhs, const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator2> & rhs)
{
  return lhs.controller_name == rhs.controller_name;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator1> & lhs, const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace dynamixel_manipulation

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "df2b10f2f876d82269ae3fc1e0538e11";
  }

  static const char* value(const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdf2b10f2f876d822ULL;
  static const uint64_t static_value2 = 0x69ae3fc1e0538e11ULL;
};

template<class ContainerAllocator>
struct DataType< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dynamixel_manipulation/StopControllerRequest";
  }

  static const char* value(const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string controller_name\n"
;
  }

  static const char* value(const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.controller_name);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct StopControllerRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::dynamixel_manipulation::StopControllerRequest_<ContainerAllocator>& v)
  {
    s << indent << "controller_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.controller_name);
  }
};

} // namespace message_operations
} // namespace ros

#endif // DYNAMIXEL_MANIPULATION_MESSAGE_STOPCONTROLLERREQUEST_H
