// Generated by gencpp from file robotic_arm/joint_position.msg
// DO NOT EDIT!


#ifndef ROBOTIC_ARM_MESSAGE_JOINT_POSITION_H
#define ROBOTIC_ARM_MESSAGE_JOINT_POSITION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace robotic_arm
{
template <class ContainerAllocator>
struct joint_position_
{
  typedef joint_position_<ContainerAllocator> Type;

  joint_position_()
    : data()  {
    }
  joint_position_(const ContainerAllocator& _alloc)
    : data(_alloc)  {
  (void)_alloc;
    }



   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::robotic_arm::joint_position_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::robotic_arm::joint_position_<ContainerAllocator> const> ConstPtr;

}; // struct joint_position_

typedef ::robotic_arm::joint_position_<std::allocator<void> > joint_position;

typedef boost::shared_ptr< ::robotic_arm::joint_position > joint_positionPtr;
typedef boost::shared_ptr< ::robotic_arm::joint_position const> joint_positionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::robotic_arm::joint_position_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::robotic_arm::joint_position_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::robotic_arm::joint_position_<ContainerAllocator1> & lhs, const ::robotic_arm::joint_position_<ContainerAllocator2> & rhs)
{
  return lhs.data == rhs.data;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::robotic_arm::joint_position_<ContainerAllocator1> & lhs, const ::robotic_arm::joint_position_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace robotic_arm

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::robotic_arm::joint_position_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::robotic_arm::joint_position_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotic_arm::joint_position_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::robotic_arm::joint_position_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotic_arm::joint_position_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::robotic_arm::joint_position_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::robotic_arm::joint_position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "420cd38b6b071cd49f2970c3e2cee511";
  }

  static const char* value(const ::robotic_arm::joint_position_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x420cd38b6b071cd4ULL;
  static const uint64_t static_value2 = 0x9f2970c3e2cee511ULL;
};

template<class ContainerAllocator>
struct DataType< ::robotic_arm::joint_position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "robotic_arm/joint_position";
  }

  static const char* value(const ::robotic_arm::joint_position_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::robotic_arm::joint_position_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float32[] data\n"
;
  }

  static const char* value(const ::robotic_arm::joint_position_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::robotic_arm::joint_position_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct joint_position_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::robotic_arm::joint_position_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::robotic_arm::joint_position_<ContainerAllocator>& v)
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.data[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // ROBOTIC_ARM_MESSAGE_JOINT_POSITION_H
