// Generated by gencpp from file jointControl/RobotMoveRequest.msg
// DO NOT EDIT!


#ifndef JOINTCONTROL_MESSAGE_ROBOTMOVEREQUEST_H
#define JOINTCONTROL_MESSAGE_ROBOTMOVEREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Pose.h>

namespace jointControl
{
template <class ContainerAllocator>
struct RobotMoveRequest_
{
  typedef RobotMoveRequest_<ContainerAllocator> Type;

  RobotMoveRequest_()
    : target_pose()  {
    }
  RobotMoveRequest_(const ContainerAllocator& _alloc)
    : target_pose(_alloc)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _target_pose_type;
  _target_pose_type target_pose;





  typedef boost::shared_ptr< ::jointControl::RobotMoveRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::jointControl::RobotMoveRequest_<ContainerAllocator> const> ConstPtr;

}; // struct RobotMoveRequest_

typedef ::jointControl::RobotMoveRequest_<std::allocator<void> > RobotMoveRequest;

typedef boost::shared_ptr< ::jointControl::RobotMoveRequest > RobotMoveRequestPtr;
typedef boost::shared_ptr< ::jointControl::RobotMoveRequest const> RobotMoveRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::jointControl::RobotMoveRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::jointControl::RobotMoveRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::jointControl::RobotMoveRequest_<ContainerAllocator1> & lhs, const ::jointControl::RobotMoveRequest_<ContainerAllocator2> & rhs)
{
  return lhs.target_pose == rhs.target_pose;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::jointControl::RobotMoveRequest_<ContainerAllocator1> & lhs, const ::jointControl::RobotMoveRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace jointControl

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::jointControl::RobotMoveRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::jointControl::RobotMoveRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jointControl::RobotMoveRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::jointControl::RobotMoveRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jointControl::RobotMoveRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::jointControl::RobotMoveRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::jointControl::RobotMoveRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "671ab81f81ce3fa4e8a3ac70c41ddb7c";
  }

  static const char* value(const ::jointControl::RobotMoveRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x671ab81f81ce3fa4ULL;
  static const uint64_t static_value2 = 0xe8a3ac70c41ddb7cULL;
};

template<class ContainerAllocator>
struct DataType< ::jointControl::RobotMoveRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "jointControl/RobotMoveRequest";
  }

  static const char* value(const ::jointControl::RobotMoveRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::jointControl::RobotMoveRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Pose target_pose\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
;
  }

  static const char* value(const ::jointControl::RobotMoveRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::jointControl::RobotMoveRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.target_pose);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotMoveRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::jointControl::RobotMoveRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::jointControl::RobotMoveRequest_<ContainerAllocator>& v)
  {
    s << indent << "target_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.target_pose);
  }
};

} // namespace message_operations
} // namespace ros

#endif // JOINTCONTROL_MESSAGE_ROBOTMOVEREQUEST_H
