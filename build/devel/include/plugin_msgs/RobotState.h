// Generated by gencpp from file plugin_msgs/RobotState.msg
// DO NOT EDIT!


#ifndef PLUGIN_MSGS_MESSAGE_ROBOTSTATE_H
#define PLUGIN_MSGS_MESSAGE_ROBOTSTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>

namespace plugin_msgs
{
template <class ContainerAllocator>
struct RobotState_
{
  typedef RobotState_<ContainerAllocator> Type;

  RobotState_()
    : header()
    , joint_position()
    , joint_velocity()
    , joint_position_goal()
    , joint_velocity_goal()
    , joint_position_error()
    , joint_velocity_error()
    , end_effector_frame()
    , cartesian_position()
    , cartesian_velocity()
    , cartesian_position_goal()
    , cartesian_velocity_goal()
    , cartesian_position_error()
    , cartesian_velocity_error()  {
    }
  RobotState_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , joint_position(_alloc)
    , joint_velocity(_alloc)
    , joint_position_goal(_alloc)
    , joint_velocity_goal(_alloc)
    , joint_position_error(_alloc)
    , joint_velocity_error(_alloc)
    , end_effector_frame(_alloc)
    , cartesian_position(_alloc)
    , cartesian_velocity(_alloc)
    , cartesian_position_goal(_alloc)
    , cartesian_velocity_goal(_alloc)
    , cartesian_position_error(_alloc)
    , cartesian_velocity_error(_alloc)  {
  (void)_alloc;
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _joint_position_type;
  _joint_position_type joint_position;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _joint_velocity_type;
  _joint_velocity_type joint_velocity;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _joint_position_goal_type;
  _joint_position_goal_type joint_position_goal;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _joint_velocity_goal_type;
  _joint_velocity_goal_type joint_velocity_goal;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _joint_position_error_type;
  _joint_position_error_type joint_position_error;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _joint_velocity_error_type;
  _joint_velocity_error_type joint_velocity_error;

   typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _end_effector_frame_type;
  _end_effector_frame_type end_effector_frame;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _cartesian_position_type;
  _cartesian_position_type cartesian_position;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _cartesian_velocity_type;
  _cartesian_velocity_type cartesian_velocity;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _cartesian_position_goal_type;
  _cartesian_position_goal_type cartesian_position_goal;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _cartesian_velocity_goal_type;
  _cartesian_velocity_goal_type cartesian_velocity_goal;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _cartesian_position_error_type;
  _cartesian_position_error_type cartesian_position_error;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _cartesian_velocity_error_type;
  _cartesian_velocity_error_type cartesian_velocity_error;





  typedef boost::shared_ptr< ::plugin_msgs::RobotState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::plugin_msgs::RobotState_<ContainerAllocator> const> ConstPtr;

}; // struct RobotState_

typedef ::plugin_msgs::RobotState_<std::allocator<void> > RobotState;

typedef boost::shared_ptr< ::plugin_msgs::RobotState > RobotStatePtr;
typedef boost::shared_ptr< ::plugin_msgs::RobotState const> RobotStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::plugin_msgs::RobotState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::plugin_msgs::RobotState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::plugin_msgs::RobotState_<ContainerAllocator1> & lhs, const ::plugin_msgs::RobotState_<ContainerAllocator2> & rhs)
{
  return lhs.header == rhs.header &&
    lhs.joint_position == rhs.joint_position &&
    lhs.joint_velocity == rhs.joint_velocity &&
    lhs.joint_position_goal == rhs.joint_position_goal &&
    lhs.joint_velocity_goal == rhs.joint_velocity_goal &&
    lhs.joint_position_error == rhs.joint_position_error &&
    lhs.joint_velocity_error == rhs.joint_velocity_error &&
    lhs.end_effector_frame == rhs.end_effector_frame &&
    lhs.cartesian_position == rhs.cartesian_position &&
    lhs.cartesian_velocity == rhs.cartesian_velocity &&
    lhs.cartesian_position_goal == rhs.cartesian_position_goal &&
    lhs.cartesian_velocity_goal == rhs.cartesian_velocity_goal &&
    lhs.cartesian_position_error == rhs.cartesian_position_error &&
    lhs.cartesian_velocity_error == rhs.cartesian_velocity_error;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::plugin_msgs::RobotState_<ContainerAllocator1> & lhs, const ::plugin_msgs::RobotState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace plugin_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::plugin_msgs::RobotState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plugin_msgs::RobotState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plugin_msgs::RobotState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plugin_msgs::RobotState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plugin_msgs::RobotState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plugin_msgs::RobotState_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::plugin_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8eadd78fb47ede9ac5103ae9c18c045";
  }

  static const char* value(const ::plugin_msgs::RobotState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8eadd78fb47ede9ULL;
  static const uint64_t static_value2 = 0xac5103ae9c18c045ULL;
};

template<class ContainerAllocator>
struct DataType< ::plugin_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "plugin_msgs/RobotState";
  }

  static const char* value(const ::plugin_msgs::RobotState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::plugin_msgs::RobotState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header     header\n"
"float64[]  joint_position\n"
"float64[]  joint_velocity\n"
"float64[]  joint_position_goal\n"
"float64[]  joint_velocity_goal\n"
"float64[]  joint_position_error\n"
"float64[]  joint_velocity_error\n"
"\n"
"geometry_msgs/Pose             end_effector_frame\n"
"\n"
"float64[] cartesian_position\n"
"float64[] cartesian_velocity\n"
"float64[] cartesian_position_goal\n"
"float64[] cartesian_velocity_goal\n"
"float64[] cartesian_position_error\n"
"float64[] cartesian_velocity_error\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
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

  static const char* value(const ::plugin_msgs::RobotState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::plugin_msgs::RobotState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.joint_position);
      stream.next(m.joint_velocity);
      stream.next(m.joint_position_goal);
      stream.next(m.joint_velocity_goal);
      stream.next(m.joint_position_error);
      stream.next(m.joint_velocity_error);
      stream.next(m.end_effector_frame);
      stream.next(m.cartesian_position);
      stream.next(m.cartesian_velocity);
      stream.next(m.cartesian_position_goal);
      stream.next(m.cartesian_velocity_goal);
      stream.next(m.cartesian_position_error);
      stream.next(m.cartesian_velocity_error);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct RobotState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::plugin_msgs::RobotState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::plugin_msgs::RobotState_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "joint_position[]" << std::endl;
    for (size_t i = 0; i < v.joint_position.size(); ++i)
    {
      s << indent << "  joint_position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_position[i]);
    }
    s << indent << "joint_velocity[]" << std::endl;
    for (size_t i = 0; i < v.joint_velocity.size(); ++i)
    {
      s << indent << "  joint_velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_velocity[i]);
    }
    s << indent << "joint_position_goal[]" << std::endl;
    for (size_t i = 0; i < v.joint_position_goal.size(); ++i)
    {
      s << indent << "  joint_position_goal[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_position_goal[i]);
    }
    s << indent << "joint_velocity_goal[]" << std::endl;
    for (size_t i = 0; i < v.joint_velocity_goal.size(); ++i)
    {
      s << indent << "  joint_velocity_goal[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_velocity_goal[i]);
    }
    s << indent << "joint_position_error[]" << std::endl;
    for (size_t i = 0; i < v.joint_position_error.size(); ++i)
    {
      s << indent << "  joint_position_error[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_position_error[i]);
    }
    s << indent << "joint_velocity_error[]" << std::endl;
    for (size_t i = 0; i < v.joint_velocity_error.size(); ++i)
    {
      s << indent << "  joint_velocity_error[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.joint_velocity_error[i]);
    }
    s << indent << "end_effector_frame: ";
    s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.end_effector_frame);
    s << indent << "cartesian_position[]" << std::endl;
    for (size_t i = 0; i < v.cartesian_position.size(); ++i)
    {
      s << indent << "  cartesian_position[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cartesian_position[i]);
    }
    s << indent << "cartesian_velocity[]" << std::endl;
    for (size_t i = 0; i < v.cartesian_velocity.size(); ++i)
    {
      s << indent << "  cartesian_velocity[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cartesian_velocity[i]);
    }
    s << indent << "cartesian_position_goal[]" << std::endl;
    for (size_t i = 0; i < v.cartesian_position_goal.size(); ++i)
    {
      s << indent << "  cartesian_position_goal[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cartesian_position_goal[i]);
    }
    s << indent << "cartesian_velocity_goal[]" << std::endl;
    for (size_t i = 0; i < v.cartesian_velocity_goal.size(); ++i)
    {
      s << indent << "  cartesian_velocity_goal[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cartesian_velocity_goal[i]);
    }
    s << indent << "cartesian_position_error[]" << std::endl;
    for (size_t i = 0; i < v.cartesian_position_error.size(); ++i)
    {
      s << indent << "  cartesian_position_error[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cartesian_position_error[i]);
    }
    s << indent << "cartesian_velocity_error[]" << std::endl;
    for (size_t i = 0; i < v.cartesian_velocity_error.size(); ++i)
    {
      s << indent << "  cartesian_velocity_error[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.cartesian_velocity_error[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLUGIN_MSGS_MESSAGE_ROBOTSTATE_H