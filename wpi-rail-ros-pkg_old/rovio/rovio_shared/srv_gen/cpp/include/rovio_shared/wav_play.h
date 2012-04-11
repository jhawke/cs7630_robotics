/* Auto-generated by genmsg_cpp for file /home/dev/cs7630_robotics/wpi-rail-ros-pkg_old/rovio/rovio_shared/srv/wav_play.srv */
#ifndef ROVIO_SHARED_SERVICE_WAV_PLAY_H
#define ROVIO_SHARED_SERVICE_WAV_PLAY_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace rovio_shared
{
template <class ContainerAllocator>
struct wav_playRequest_ {
  typedef wav_playRequest_<ContainerAllocator> Type;

  wav_playRequest_()
  : f()
  {
  }

  wav_playRequest_(const ContainerAllocator& _alloc)
  : f(_alloc)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _f_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  f;


private:
  static const char* __s_getDataType_() { return "rovio_shared/wav_playRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "b7ec5ba08b681050147d22f3cf073480"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "b7ec5ba08b681050147d22f3cf073480"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
string f\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, f);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, f);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(f);
    return size;
  }

  typedef boost::shared_ptr< ::rovio_shared::wav_playRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rovio_shared::wav_playRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct wav_playRequest
typedef  ::rovio_shared::wav_playRequest_<std::allocator<void> > wav_playRequest;

typedef boost::shared_ptr< ::rovio_shared::wav_playRequest> wav_playRequestPtr;
typedef boost::shared_ptr< ::rovio_shared::wav_playRequest const> wav_playRequestConstPtr;


template <class ContainerAllocator>
struct wav_playResponse_ {
  typedef wav_playResponse_<ContainerAllocator> Type;

  wav_playResponse_()
  {
  }

  wav_playResponse_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "rovio_shared/wav_playResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "b7ec5ba08b681050147d22f3cf073480"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    return size;
  }

  typedef boost::shared_ptr< ::rovio_shared::wav_playResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rovio_shared::wav_playResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct wav_playResponse
typedef  ::rovio_shared::wav_playResponse_<std::allocator<void> > wav_playResponse;

typedef boost::shared_ptr< ::rovio_shared::wav_playResponse> wav_playResponsePtr;
typedef boost::shared_ptr< ::rovio_shared::wav_playResponse const> wav_playResponseConstPtr;

struct wav_play
{

typedef wav_playRequest Request;
typedef wav_playResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct wav_play
} // namespace rovio_shared

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rovio_shared::wav_playRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rovio_shared::wav_playRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rovio_shared::wav_playRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b7ec5ba08b681050147d22f3cf073480";
  }

  static const char* value(const  ::rovio_shared::wav_playRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb7ec5ba08b681050ULL;
  static const uint64_t static_value2 = 0x147d22f3cf073480ULL;
};

template<class ContainerAllocator>
struct DataType< ::rovio_shared::wav_playRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rovio_shared/wav_playRequest";
  }

  static const char* value(const  ::rovio_shared::wav_playRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rovio_shared::wav_playRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
string f\n\
\n\
";
  }

  static const char* value(const  ::rovio_shared::wav_playRequest_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rovio_shared::wav_playResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rovio_shared::wav_playResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rovio_shared::wav_playResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::rovio_shared::wav_playResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::rovio_shared::wav_playResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rovio_shared/wav_playResponse";
  }

  static const char* value(const  ::rovio_shared::wav_playResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rovio_shared::wav_playResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
\n\
\n\
";
  }

  static const char* value(const  ::rovio_shared::wav_playResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rovio_shared::wav_playResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rovio_shared::wav_playRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.f);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct wav_playRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rovio_shared::wav_playResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct wav_playResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<rovio_shared::wav_play> {
  static const char* value() 
  {
    return "b7ec5ba08b681050147d22f3cf073480";
  }

  static const char* value(const rovio_shared::wav_play&) { return value(); } 
};

template<>
struct DataType<rovio_shared::wav_play> {
  static const char* value() 
  {
    return "rovio_shared/wav_play";
  }

  static const char* value(const rovio_shared::wav_play&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rovio_shared::wav_playRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b7ec5ba08b681050147d22f3cf073480";
  }

  static const char* value(const rovio_shared::wav_playRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rovio_shared::wav_playRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rovio_shared/wav_play";
  }

  static const char* value(const rovio_shared::wav_playRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rovio_shared::wav_playResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b7ec5ba08b681050147d22f3cf073480";
  }

  static const char* value(const rovio_shared::wav_playResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rovio_shared::wav_playResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rovio_shared/wav_play";
  }

  static const char* value(const rovio_shared::wav_playResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ROVIO_SHARED_SERVICE_WAV_PLAY_H

