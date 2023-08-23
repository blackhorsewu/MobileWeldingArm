// Generated by gencpp from file bunker_msgs/BunkerMotorState.msg
// DO NOT EDIT!


#ifndef BUNKER_MSGS_MESSAGE_BUNKERMOTORSTATE_H
#define BUNKER_MSGS_MESSAGE_BUNKERMOTORSTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace bunker_msgs
{
template <class ContainerAllocator>
struct BunkerMotorState_
{
  typedef BunkerMotorState_<ContainerAllocator> Type;

  BunkerMotorState_()
    : current(0.0)
    , rpm(0.0)
    , temperature(0.0)  {
    }
  BunkerMotorState_(const ContainerAllocator& _alloc)
    : current(0.0)
    , rpm(0.0)
    , temperature(0.0)  {
  (void)_alloc;
    }



   typedef double _current_type;
  _current_type current;

   typedef double _rpm_type;
  _rpm_type rpm;

   typedef double _temperature_type;
  _temperature_type temperature;





  typedef boost::shared_ptr< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> const> ConstPtr;

}; // struct BunkerMotorState_

typedef ::bunker_msgs::BunkerMotorState_<std::allocator<void> > BunkerMotorState;

typedef boost::shared_ptr< ::bunker_msgs::BunkerMotorState > BunkerMotorStatePtr;
typedef boost::shared_ptr< ::bunker_msgs::BunkerMotorState const> BunkerMotorStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::bunker_msgs::BunkerMotorState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::bunker_msgs::BunkerMotorState_<ContainerAllocator1> & lhs, const ::bunker_msgs::BunkerMotorState_<ContainerAllocator2> & rhs)
{
  return lhs.current == rhs.current &&
    lhs.rpm == rhs.rpm &&
    lhs.temperature == rhs.temperature;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::bunker_msgs::BunkerMotorState_<ContainerAllocator1> & lhs, const ::bunker_msgs::BunkerMotorState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace bunker_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9380628b50ebdc90ce46d4147360680d";
  }

  static const char* value(const ::bunker_msgs::BunkerMotorState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9380628b50ebdc90ULL;
  static const uint64_t static_value2 = 0xce46d4147360680dULL;
};

template<class ContainerAllocator>
struct DataType< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bunker_msgs/BunkerMotorState";
  }

  static const char* value(const ::bunker_msgs::BunkerMotorState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 current\n"
"float64 rpm\n"
"float64 temperature\n"
;
  }

  static const char* value(const ::bunker_msgs::BunkerMotorState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current);
      stream.next(m.rpm);
      stream.next(m.temperature);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BunkerMotorState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::bunker_msgs::BunkerMotorState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::bunker_msgs::BunkerMotorState_<ContainerAllocator>& v)
  {
    s << indent << "current: ";
    Printer<double>::stream(s, indent + "  ", v.current);
    s << indent << "rpm: ";
    Printer<double>::stream(s, indent + "  ", v.rpm);
    s << indent << "temperature: ";
    Printer<double>::stream(s, indent + "  ", v.temperature);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BUNKER_MSGS_MESSAGE_BUNKERMOTORSTATE_H
