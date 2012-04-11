/* Auto-generated by genmsg_cpp for file /home/dev/cs7630_robotics/raptor/srv/light_level_srv.srv */
#ifndef RAPTOR_SERVICE_LIGHT_LEVEL_SRV_H
#define RAPTOR_SERVICE_LIGHT_LEVEL_SRV_H
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




namespace raptor
{
template <class ContainerAllocator>
struct light_level_srvRequest_ {
  typedef light_level_srvRequest_<ContainerAllocator> Type;

  light_level_srvRequest_()
  {
  }

  light_level_srvRequest_(const ContainerAllocator& _alloc)
  {
  }


private:
  static const char* __s_getDataType_() { return "raptor/light_level_srvRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "d41d8cd98f00b204e9800998ecf8427e"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "883936641ce951bb1dc9b3fc31b6bb04"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
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

  typedef boost::shared_ptr< ::raptor::light_level_srvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::raptor::light_level_srvRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct light_level_srvRequest
typedef  ::raptor::light_level_srvRequest_<std::allocator<void> > light_level_srvRequest;

typedef boost::shared_ptr< ::raptor::light_level_srvRequest> light_level_srvRequestPtr;
typedef boost::shared_ptr< ::raptor::light_level_srvRequest const> light_level_srvRequestConstPtr;


template <class ContainerAllocator>
struct light_level_srvResponse_ {
  typedef light_level_srvResponse_<ContainerAllocator> Type;

  light_level_srvResponse_()
  : light_level(0)
  {
  }

  light_level_srvResponse_(const ContainerAllocator& _alloc)
  : light_level(0)
  {
  }

  typedef int32_t _light_level_type;
  int32_t light_level;


private:
  static const char* __s_getDataType_() { return "raptor/light_level_srvResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "883936641ce951bb1dc9b3fc31b6bb04"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "883936641ce951bb1dc9b3fc31b6bb04"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "int32 light_level\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, light_level);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, light_level);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(light_level);
    return size;
  }

  typedef boost::shared_ptr< ::raptor::light_level_srvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::raptor::light_level_srvResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct light_level_srvResponse
typedef  ::raptor::light_level_srvResponse_<std::allocator<void> > light_level_srvResponse;

typedef boost::shared_ptr< ::raptor::light_level_srvResponse> light_level_srvResponsePtr;
typedef boost::shared_ptr< ::raptor::light_level_srvResponse const> light_level_srvResponseConstPtr;

struct light_level_srv
{

typedef light_level_srvRequest Request;
typedef light_level_srvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct light_level_srv
} // namespace raptor

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::raptor::light_level_srvRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::raptor::light_level_srvRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::raptor::light_level_srvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const  ::raptor::light_level_srvRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::raptor::light_level_srvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "raptor/light_level_srvRequest";
  }

  static const char* value(const  ::raptor::light_level_srvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::raptor::light_level_srvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
";
  }

  static const char* value(const  ::raptor::light_level_srvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::raptor::light_level_srvRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::raptor::light_level_srvResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::raptor::light_level_srvResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::raptor::light_level_srvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "883936641ce951bb1dc9b3fc31b6bb04";
  }

  static const char* value(const  ::raptor::light_level_srvResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x883936641ce951bbULL;
  static const uint64_t static_value2 = 0x1dc9b3fc31b6bb04ULL;
};

template<class ContainerAllocator>
struct DataType< ::raptor::light_level_srvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "raptor/light_level_srvResponse";
  }

  static const char* value(const  ::raptor::light_level_srvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::raptor::light_level_srvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int32 light_level\n\
\n\
";
  }

  static const char* value(const  ::raptor::light_level_srvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::raptor::light_level_srvResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::raptor::light_level_srvRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct light_level_srvRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::raptor::light_level_srvResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.light_level);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct light_level_srvResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<raptor::light_level_srv> {
  static const char* value() 
  {
    return "883936641ce951bb1dc9b3fc31b6bb04";
  }

  static const char* value(const raptor::light_level_srv&) { return value(); } 
};

template<>
struct DataType<raptor::light_level_srv> {
  static const char* value() 
  {
    return "raptor/light_level_srv";
  }

  static const char* value(const raptor::light_level_srv&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<raptor::light_level_srvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "883936641ce951bb1dc9b3fc31b6bb04";
  }

  static const char* value(const raptor::light_level_srvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<raptor::light_level_srvRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "raptor/light_level_srv";
  }

  static const char* value(const raptor::light_level_srvRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<raptor::light_level_srvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "883936641ce951bb1dc9b3fc31b6bb04";
  }

  static const char* value(const raptor::light_level_srvResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<raptor::light_level_srvResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "raptor/light_level_srv";
  }

  static const char* value(const raptor::light_level_srvResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // RAPTOR_SERVICE_LIGHT_LEVEL_SRV_H
