/* Auto-generated by genmsg_cpp for file /home/dev/cs7630_robotics/wpi-rail-ros-pkg/rovio/rovio_shared/srv/head_ctrl.srv */
#ifndef ROVIO_SHARED_SERVICE_HEAD_CTRL_H
#define ROVIO_SHARED_SERVICE_HEAD_CTRL_H
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
struct head_ctrlRequest_ {
  typedef head_ctrlRequest_<ContainerAllocator> Type;

  head_ctrlRequest_()
  : head_pos(0)
  {
  }

  head_ctrlRequest_(const ContainerAllocator& _alloc)
  : head_pos(0)
  {
  }

  typedef int8_t _head_pos_type;
  int8_t head_pos;

  enum { HEAD_UP = 11 };
  enum { HEAD_DOWN = 12 };
  enum { HEAD_MIDDLE = 13 };

private:
  static const char* __s_getDataType_() { return "rovio_shared/head_ctrlRequest"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "522f8591c845ace5ac8c5c5852170802"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "8fc91ecf3dc7f4ab832a70ed14ec95b7"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
int8 HEAD_UP=11\n\
int8 HEAD_DOWN=12\n\
int8 HEAD_MIDDLE=13\n\
\n\
int8 head_pos\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, head_pos);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, head_pos);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(head_pos);
    return size;
  }

  typedef boost::shared_ptr< ::rovio_shared::head_ctrlRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rovio_shared::head_ctrlRequest_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct head_ctrlRequest
typedef  ::rovio_shared::head_ctrlRequest_<std::allocator<void> > head_ctrlRequest;

typedef boost::shared_ptr< ::rovio_shared::head_ctrlRequest> head_ctrlRequestPtr;
typedef boost::shared_ptr< ::rovio_shared::head_ctrlRequest const> head_ctrlRequestConstPtr;


template <class ContainerAllocator>
struct head_ctrlResponse_ {
  typedef head_ctrlResponse_<ContainerAllocator> Type;

  head_ctrlResponse_()
  : response(0)
  {
  }

  head_ctrlResponse_(const ContainerAllocator& _alloc)
  : response(0)
  {
  }

  typedef int8_t _response_type;
  int8_t response;

  enum { SUCCESS = 0 };
  enum { FAILURE = 1 };
  enum { ROBOT_BUSY = 2 };
  enum { FEATURE_NOT_IMPLEMENTED = 3 };
  enum { UNKNOWN_CGI_ACTION = 4 };
  enum { NO_NS_SIGNAL = 5 };
  enum { NO_EMPTY_PATH_AVAILABLE = 6 };
  enum { FAILED_TO_READ_PATH = 7 };
  enum { PATH_BASEADDRESS_NOT_INITIALIZED = 8 };
  enum { PATH_NOT_FOUND = 9 };
  enum { PATH_NAME_NOT_SPECIFIED = 10 };
  enum { NOT_RECORDING_PATH = 11 };
  enum { FLASH_NOT_INITIALIZED = 12 };
  enum { FAILED_TO_DELETE_PATH = 13 };
  enum { FAILED_TO_READ_FROM_FLASH = 14 };
  enum { FAILED_TO_WRITE_TO_FLASH = 15 };
  enum { FLASH_NOT_READY = 16 };
  enum { NO_MEMORY_AVAILABLE = 17 };
  enum { NO_MCU_PORT_AVAILABLE = 18 };
  enum { NO_NS_PORT_AVAILABLE = 19 };
  enum { NS_PACKET_CHECKSUM_ERROR = 20 };
  enum { NS_UART_READ_ERROR = 21 };
  enum { PARAMETER_OUTOFRANGE = 22 };
  enum { NO_PARAMETER = 23 };

private:
  static const char* __s_getDataType_() { return "rovio_shared/head_ctrlResponse"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "208ac5fdc497ef2f1bd168d28a2b5e05"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getServerMD5Sum_() { return "8fc91ecf3dc7f4ab832a70ed14ec95b7"; }
public:
  ROS_DEPRECATED static const std::string __s_getServerMD5Sum() { return __s_getServerMD5Sum_(); }

  ROS_DEPRECATED const std::string __getServerMD5Sum() const { return __s_getServerMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "\n\
int8 SUCCESS=0\n\
int8 FAILURE=1\n\
int8 ROBOT_BUSY=2\n\
int8 FEATURE_NOT_IMPLEMENTED=3\n\
int8 UNKNOWN_CGI_ACTION=4\n\
int8 NO_NS_SIGNAL=5\n\
int8 NO_EMPTY_PATH_AVAILABLE=6\n\
int8 FAILED_TO_READ_PATH=7\n\
int8 PATH_BASEADDRESS_NOT_INITIALIZED=8\n\
int8 PATH_NOT_FOUND=9\n\
int8 PATH_NAME_NOT_SPECIFIED=10\n\
int8 NOT_RECORDING_PATH=11\n\
int8 FLASH_NOT_INITIALIZED=12\n\
int8 FAILED_TO_DELETE_PATH=13\n\
int8 FAILED_TO_READ_FROM_FLASH=14\n\
int8 FAILED_TO_WRITE_TO_FLASH=15\n\
int8 FLASH_NOT_READY=16\n\
int8 NO_MEMORY_AVAILABLE=17\n\
int8 NO_MCU_PORT_AVAILABLE=18\n\
int8 NO_NS_PORT_AVAILABLE=19\n\
int8 NS_PACKET_CHECKSUM_ERROR=20\n\
int8 NS_UART_READ_ERROR=21\n\
int8 PARAMETER_OUTOFRANGE=22\n\
int8 NO_PARAMETER=23\n\
\n\
int8 response\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, response);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, response);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(response);
    return size;
  }

  typedef boost::shared_ptr< ::rovio_shared::head_ctrlResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rovio_shared::head_ctrlResponse_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct head_ctrlResponse
typedef  ::rovio_shared::head_ctrlResponse_<std::allocator<void> > head_ctrlResponse;

typedef boost::shared_ptr< ::rovio_shared::head_ctrlResponse> head_ctrlResponsePtr;
typedef boost::shared_ptr< ::rovio_shared::head_ctrlResponse const> head_ctrlResponseConstPtr;

struct head_ctrl
{

typedef head_ctrlRequest Request;
typedef head_ctrlResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct head_ctrl
} // namespace rovio_shared

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rovio_shared::head_ctrlRequest_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rovio_shared::head_ctrlRequest_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rovio_shared::head_ctrlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "522f8591c845ace5ac8c5c5852170802";
  }

  static const char* value(const  ::rovio_shared::head_ctrlRequest_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x522f8591c845ace5ULL;
  static const uint64_t static_value2 = 0xac8c5c5852170802ULL;
};

template<class ContainerAllocator>
struct DataType< ::rovio_shared::head_ctrlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rovio_shared/head_ctrlRequest";
  }

  static const char* value(const  ::rovio_shared::head_ctrlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rovio_shared::head_ctrlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
int8 HEAD_UP=11\n\
int8 HEAD_DOWN=12\n\
int8 HEAD_MIDDLE=13\n\
\n\
int8 head_pos\n\
\n\
";
  }

  static const char* value(const  ::rovio_shared::head_ctrlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rovio_shared::head_ctrlRequest_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rovio_shared::head_ctrlResponse_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rovio_shared::head_ctrlResponse_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rovio_shared::head_ctrlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "208ac5fdc497ef2f1bd168d28a2b5e05";
  }

  static const char* value(const  ::rovio_shared::head_ctrlResponse_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x208ac5fdc497ef2fULL;
  static const uint64_t static_value2 = 0x1bd168d28a2b5e05ULL;
};

template<class ContainerAllocator>
struct DataType< ::rovio_shared::head_ctrlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rovio_shared/head_ctrlResponse";
  }

  static const char* value(const  ::rovio_shared::head_ctrlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rovio_shared::head_ctrlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "\n\
int8 SUCCESS=0\n\
int8 FAILURE=1\n\
int8 ROBOT_BUSY=2\n\
int8 FEATURE_NOT_IMPLEMENTED=3\n\
int8 UNKNOWN_CGI_ACTION=4\n\
int8 NO_NS_SIGNAL=5\n\
int8 NO_EMPTY_PATH_AVAILABLE=6\n\
int8 FAILED_TO_READ_PATH=7\n\
int8 PATH_BASEADDRESS_NOT_INITIALIZED=8\n\
int8 PATH_NOT_FOUND=9\n\
int8 PATH_NAME_NOT_SPECIFIED=10\n\
int8 NOT_RECORDING_PATH=11\n\
int8 FLASH_NOT_INITIALIZED=12\n\
int8 FAILED_TO_DELETE_PATH=13\n\
int8 FAILED_TO_READ_FROM_FLASH=14\n\
int8 FAILED_TO_WRITE_TO_FLASH=15\n\
int8 FLASH_NOT_READY=16\n\
int8 NO_MEMORY_AVAILABLE=17\n\
int8 NO_MCU_PORT_AVAILABLE=18\n\
int8 NO_NS_PORT_AVAILABLE=19\n\
int8 NS_PACKET_CHECKSUM_ERROR=20\n\
int8 NS_UART_READ_ERROR=21\n\
int8 PARAMETER_OUTOFRANGE=22\n\
int8 NO_PARAMETER=23\n\
\n\
int8 response\n\
\n\
";
  }

  static const char* value(const  ::rovio_shared::head_ctrlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rovio_shared::head_ctrlResponse_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rovio_shared::head_ctrlRequest_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.head_pos);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct head_ctrlRequest_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rovio_shared::head_ctrlResponse_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.response);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct head_ctrlResponse_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<rovio_shared::head_ctrl> {
  static const char* value() 
  {
    return "8fc91ecf3dc7f4ab832a70ed14ec95b7";
  }

  static const char* value(const rovio_shared::head_ctrl&) { return value(); } 
};

template<>
struct DataType<rovio_shared::head_ctrl> {
  static const char* value() 
  {
    return "rovio_shared/head_ctrl";
  }

  static const char* value(const rovio_shared::head_ctrl&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rovio_shared::head_ctrlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8fc91ecf3dc7f4ab832a70ed14ec95b7";
  }

  static const char* value(const rovio_shared::head_ctrlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rovio_shared::head_ctrlRequest_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rovio_shared/head_ctrl";
  }

  static const char* value(const rovio_shared::head_ctrlRequest_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rovio_shared::head_ctrlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8fc91ecf3dc7f4ab832a70ed14ec95b7";
  }

  static const char* value(const rovio_shared::head_ctrlResponse_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rovio_shared::head_ctrlResponse_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rovio_shared/head_ctrl";
  }

  static const char* value(const rovio_shared::head_ctrlResponse_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // ROVIO_SHARED_SERVICE_HEAD_CTRL_H

