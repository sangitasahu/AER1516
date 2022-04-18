// Generated by gencpp from file fla_msgs/Detection.msg
// DO NOT EDIT!


#ifndef FLA_MSGS_MESSAGE_DETECTION_H
#define FLA_MSGS_MESSAGE_DETECTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace fla_msgs
{
template <class ContainerAllocator>
struct Detection_
{
  typedef Detection_<ContainerAllocator> Type;

  Detection_()
    : class_id(0)
    , class_name()
    , confidence(0.0)
    , x_min(0.0)
    , y_min(0.0)
    , x_max(0.0)
    , y_max(0.0)
    , object_id(0)
    , x_pos(0.0)
    , y_pos(0.0)
    , z_pos(0.0)  {
    }
  Detection_(const ContainerAllocator& _alloc)
    : class_id(0)
    , class_name(_alloc)
    , confidence(0.0)
    , x_min(0.0)
    , y_min(0.0)
    , x_max(0.0)
    , y_max(0.0)
    , object_id(0)
    , x_pos(0.0)
    , y_pos(0.0)
    , z_pos(0.0)  {
  (void)_alloc;
    }



   typedef uint32_t _class_id_type;
  _class_id_type class_id;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _class_name_type;
  _class_name_type class_name;

   typedef float _confidence_type;
  _confidence_type confidence;

   typedef float _x_min_type;
  _x_min_type x_min;

   typedef float _y_min_type;
  _y_min_type y_min;

   typedef float _x_max_type;
  _x_max_type x_max;

   typedef float _y_max_type;
  _y_max_type y_max;

   typedef uint32_t _object_id_type;
  _object_id_type object_id;

   typedef float _x_pos_type;
  _x_pos_type x_pos;

   typedef float _y_pos_type;
  _y_pos_type y_pos;

   typedef float _z_pos_type;
  _z_pos_type z_pos;





  typedef boost::shared_ptr< ::fla_msgs::Detection_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::fla_msgs::Detection_<ContainerAllocator> const> ConstPtr;

}; // struct Detection_

typedef ::fla_msgs::Detection_<std::allocator<void> > Detection;

typedef boost::shared_ptr< ::fla_msgs::Detection > DetectionPtr;
typedef boost::shared_ptr< ::fla_msgs::Detection const> DetectionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::fla_msgs::Detection_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::fla_msgs::Detection_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::fla_msgs::Detection_<ContainerAllocator1> & lhs, const ::fla_msgs::Detection_<ContainerAllocator2> & rhs)
{
  return lhs.class_id == rhs.class_id &&
    lhs.class_name == rhs.class_name &&
    lhs.confidence == rhs.confidence &&
    lhs.x_min == rhs.x_min &&
    lhs.y_min == rhs.y_min &&
    lhs.x_max == rhs.x_max &&
    lhs.y_max == rhs.y_max &&
    lhs.object_id == rhs.object_id &&
    lhs.x_pos == rhs.x_pos &&
    lhs.y_pos == rhs.y_pos &&
    lhs.z_pos == rhs.z_pos;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::fla_msgs::Detection_<ContainerAllocator1> & lhs, const ::fla_msgs::Detection_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace fla_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::fla_msgs::Detection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::fla_msgs::Detection_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fla_msgs::Detection_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::fla_msgs::Detection_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fla_msgs::Detection_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::fla_msgs::Detection_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::fla_msgs::Detection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "f4c53395944a41874a6ab9783c6e93b6";
  }

  static const char* value(const ::fla_msgs::Detection_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xf4c53395944a4187ULL;
  static const uint64_t static_value2 = 0x4a6ab9783c6e93b6ULL;
};

template<class ContainerAllocator>
struct DataType< ::fla_msgs::Detection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fla_msgs/Detection";
  }

  static const char* value(const ::fla_msgs::Detection_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::fla_msgs::Detection_<ContainerAllocator> >
{
  static const char* value()
  {
    return "uint32 class_id\n"
"string class_name\n"
"float32 confidence\n"
"\n"
"# (x_min, y_min), (x_max, y_max) define the bounding box of the detection in pixel coordinates\n"
"float32 x_min \n"
"float32 y_min\n"
"float32 x_max\n"
"float32 y_max\n"
"\n"
"# ground truth unique identifier and position of the detected object (Simulation only) \n"
"uint32 object_id\n"
"float32 x_pos\n"
"float32 y_pos\n"
"float32 z_pos\n"
;
  }

  static const char* value(const ::fla_msgs::Detection_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::fla_msgs::Detection_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.class_id);
      stream.next(m.class_name);
      stream.next(m.confidence);
      stream.next(m.x_min);
      stream.next(m.y_min);
      stream.next(m.x_max);
      stream.next(m.y_max);
      stream.next(m.object_id);
      stream.next(m.x_pos);
      stream.next(m.y_pos);
      stream.next(m.z_pos);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Detection_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::fla_msgs::Detection_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::fla_msgs::Detection_<ContainerAllocator>& v)
  {
    s << indent << "class_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.class_id);
    s << indent << "class_name: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.class_name);
    s << indent << "confidence: ";
    Printer<float>::stream(s, indent + "  ", v.confidence);
    s << indent << "x_min: ";
    Printer<float>::stream(s, indent + "  ", v.x_min);
    s << indent << "y_min: ";
    Printer<float>::stream(s, indent + "  ", v.y_min);
    s << indent << "x_max: ";
    Printer<float>::stream(s, indent + "  ", v.x_max);
    s << indent << "y_max: ";
    Printer<float>::stream(s, indent + "  ", v.y_max);
    s << indent << "object_id: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.object_id);
    s << indent << "x_pos: ";
    Printer<float>::stream(s, indent + "  ", v.x_pos);
    s << indent << "y_pos: ";
    Printer<float>::stream(s, indent + "  ", v.y_pos);
    s << indent << "z_pos: ";
    Printer<float>::stream(s, indent + "  ", v.z_pos);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FLA_MSGS_MESSAGE_DETECTION_H
