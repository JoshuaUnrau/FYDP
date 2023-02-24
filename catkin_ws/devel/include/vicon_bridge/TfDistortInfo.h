// Generated by gencpp from file vicon_bridge/TfDistortInfo.msg
// DO NOT EDIT!


#ifndef VICON_BRIDGE_MESSAGE_TFDISTORTINFO_H
#define VICON_BRIDGE_MESSAGE_TFDISTORTINFO_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace vicon_bridge
{
template <class ContainerAllocator>
struct TfDistortInfo_
{
  typedef TfDistortInfo_<ContainerAllocator> Type;

  TfDistortInfo_()
    : tf_pub_rate(0.0)
    , tf_ref_frame()
    , tf_frame_in()
    , tf_frame_out()
    , delay(0)
    , position_scale(0.0)
    , noise_type()
    , sigma_roll_pitch(0.0)
    , sigma_yaw(0.0)
    , sigma_xy(0.0)
    , sigma_z(0.0)
    , random_walk_k_xy(0.0)
    , random_walk_k_z(0.0)
    , random_walk_sigma_xy(0.0)
    , random_walk_sigma_z(0.0)
    , random_walk_tau_xy(0.0)
    , random_walk_tau_z(0.0)  {
    }
  TfDistortInfo_(const ContainerAllocator& _alloc)
    : tf_pub_rate(0.0)
    , tf_ref_frame(_alloc)
    , tf_frame_in(_alloc)
    , tf_frame_out(_alloc)
    , delay(0)
    , position_scale(0.0)
    , noise_type(_alloc)
    , sigma_roll_pitch(0.0)
    , sigma_yaw(0.0)
    , sigma_xy(0.0)
    , sigma_z(0.0)
    , random_walk_k_xy(0.0)
    , random_walk_k_z(0.0)
    , random_walk_sigma_xy(0.0)
    , random_walk_sigma_z(0.0)
    , random_walk_tau_xy(0.0)
    , random_walk_tau_z(0.0)  {
  (void)_alloc;
    }



   typedef double _tf_pub_rate_type;
  _tf_pub_rate_type tf_pub_rate;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _tf_ref_frame_type;
  _tf_ref_frame_type tf_ref_frame;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _tf_frame_in_type;
  _tf_frame_in_type tf_frame_in;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _tf_frame_out_type;
  _tf_frame_out_type tf_frame_out;

   typedef int32_t _delay_type;
  _delay_type delay;

   typedef double _position_scale_type;
  _position_scale_type position_scale;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _noise_type_type;
  _noise_type_type noise_type;

   typedef double _sigma_roll_pitch_type;
  _sigma_roll_pitch_type sigma_roll_pitch;

   typedef double _sigma_yaw_type;
  _sigma_yaw_type sigma_yaw;

   typedef double _sigma_xy_type;
  _sigma_xy_type sigma_xy;

   typedef double _sigma_z_type;
  _sigma_z_type sigma_z;

   typedef double _random_walk_k_xy_type;
  _random_walk_k_xy_type random_walk_k_xy;

   typedef double _random_walk_k_z_type;
  _random_walk_k_z_type random_walk_k_z;

   typedef double _random_walk_sigma_xy_type;
  _random_walk_sigma_xy_type random_walk_sigma_xy;

   typedef double _random_walk_sigma_z_type;
  _random_walk_sigma_z_type random_walk_sigma_z;

   typedef double _random_walk_tau_xy_type;
  _random_walk_tau_xy_type random_walk_tau_xy;

   typedef double _random_walk_tau_z_type;
  _random_walk_tau_z_type random_walk_tau_z;





  typedef boost::shared_ptr< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> const> ConstPtr;

}; // struct TfDistortInfo_

typedef ::vicon_bridge::TfDistortInfo_<std::allocator<void> > TfDistortInfo;

typedef boost::shared_ptr< ::vicon_bridge::TfDistortInfo > TfDistortInfoPtr;
typedef boost::shared_ptr< ::vicon_bridge::TfDistortInfo const> TfDistortInfoConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::vicon_bridge::TfDistortInfo_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::vicon_bridge::TfDistortInfo_<ContainerAllocator1> & lhs, const ::vicon_bridge::TfDistortInfo_<ContainerAllocator2> & rhs)
{
  return lhs.tf_pub_rate == rhs.tf_pub_rate &&
    lhs.tf_ref_frame == rhs.tf_ref_frame &&
    lhs.tf_frame_in == rhs.tf_frame_in &&
    lhs.tf_frame_out == rhs.tf_frame_out &&
    lhs.delay == rhs.delay &&
    lhs.position_scale == rhs.position_scale &&
    lhs.noise_type == rhs.noise_type &&
    lhs.sigma_roll_pitch == rhs.sigma_roll_pitch &&
    lhs.sigma_yaw == rhs.sigma_yaw &&
    lhs.sigma_xy == rhs.sigma_xy &&
    lhs.sigma_z == rhs.sigma_z &&
    lhs.random_walk_k_xy == rhs.random_walk_k_xy &&
    lhs.random_walk_k_z == rhs.random_walk_k_z &&
    lhs.random_walk_sigma_xy == rhs.random_walk_sigma_xy &&
    lhs.random_walk_sigma_z == rhs.random_walk_sigma_z &&
    lhs.random_walk_tau_xy == rhs.random_walk_tau_xy &&
    lhs.random_walk_tau_z == rhs.random_walk_tau_z;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::vicon_bridge::TfDistortInfo_<ContainerAllocator1> & lhs, const ::vicon_bridge::TfDistortInfo_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace vicon_bridge

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a7025291415a53264a70b836a949be8d";
  }

  static const char* value(const ::vicon_bridge::TfDistortInfo_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa7025291415a5326ULL;
  static const uint64_t static_value2 = 0x4a70b836a949be8dULL;
};

template<class ContainerAllocator>
struct DataType< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "vicon_bridge/TfDistortInfo";
  }

  static const char* value(const ::vicon_bridge::TfDistortInfo_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 tf_pub_rate\n"
"string tf_ref_frame\n"
"string tf_frame_in\n"
"string tf_frame_out\n"
"int32 delay\n"
"float64 position_scale\n"
"string noise_type\n"
"float64 sigma_roll_pitch\n"
"float64 sigma_yaw\n"
"float64 sigma_xy\n"
"float64 sigma_z\n"
"float64 random_walk_k_xy\n"
"float64 random_walk_k_z\n"
"float64 random_walk_sigma_xy\n"
"float64 random_walk_sigma_z\n"
"float64 random_walk_tau_xy\n"
"float64 random_walk_tau_z\n"
;
  }

  static const char* value(const ::vicon_bridge::TfDistortInfo_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.tf_pub_rate);
      stream.next(m.tf_ref_frame);
      stream.next(m.tf_frame_in);
      stream.next(m.tf_frame_out);
      stream.next(m.delay);
      stream.next(m.position_scale);
      stream.next(m.noise_type);
      stream.next(m.sigma_roll_pitch);
      stream.next(m.sigma_yaw);
      stream.next(m.sigma_xy);
      stream.next(m.sigma_z);
      stream.next(m.random_walk_k_xy);
      stream.next(m.random_walk_k_z);
      stream.next(m.random_walk_sigma_xy);
      stream.next(m.random_walk_sigma_z);
      stream.next(m.random_walk_tau_xy);
      stream.next(m.random_walk_tau_z);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct TfDistortInfo_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::vicon_bridge::TfDistortInfo_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::vicon_bridge::TfDistortInfo_<ContainerAllocator>& v)
  {
    s << indent << "tf_pub_rate: ";
    Printer<double>::stream(s, indent + "  ", v.tf_pub_rate);
    s << indent << "tf_ref_frame: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.tf_ref_frame);
    s << indent << "tf_frame_in: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.tf_frame_in);
    s << indent << "tf_frame_out: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.tf_frame_out);
    s << indent << "delay: ";
    Printer<int32_t>::stream(s, indent + "  ", v.delay);
    s << indent << "position_scale: ";
    Printer<double>::stream(s, indent + "  ", v.position_scale);
    s << indent << "noise_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.noise_type);
    s << indent << "sigma_roll_pitch: ";
    Printer<double>::stream(s, indent + "  ", v.sigma_roll_pitch);
    s << indent << "sigma_yaw: ";
    Printer<double>::stream(s, indent + "  ", v.sigma_yaw);
    s << indent << "sigma_xy: ";
    Printer<double>::stream(s, indent + "  ", v.sigma_xy);
    s << indent << "sigma_z: ";
    Printer<double>::stream(s, indent + "  ", v.sigma_z);
    s << indent << "random_walk_k_xy: ";
    Printer<double>::stream(s, indent + "  ", v.random_walk_k_xy);
    s << indent << "random_walk_k_z: ";
    Printer<double>::stream(s, indent + "  ", v.random_walk_k_z);
    s << indent << "random_walk_sigma_xy: ";
    Printer<double>::stream(s, indent + "  ", v.random_walk_sigma_xy);
    s << indent << "random_walk_sigma_z: ";
    Printer<double>::stream(s, indent + "  ", v.random_walk_sigma_z);
    s << indent << "random_walk_tau_xy: ";
    Printer<double>::stream(s, indent + "  ", v.random_walk_tau_xy);
    s << indent << "random_walk_tau_z: ";
    Printer<double>::stream(s, indent + "  ", v.random_walk_tau_z);
  }
};

} // namespace message_operations
} // namespace ros

#endif // VICON_BRIDGE_MESSAGE_TFDISTORTINFO_H
