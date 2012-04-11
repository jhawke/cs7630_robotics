"""autogenerated by genmsg_py from man_drv_srvRequest.msg. Do not edit."""
import roslib.message
import struct


class man_drv_srvRequest(roslib.message.Message):
  _md5sum = "14675853bd9417686a1390c3fb2eaae6"
  _type = "rovio_shared/man_drv_srvRequest"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
int8 STOP=0
int8 FORWARD=1
int8 BACKWARD=2
int8 STRAIGHT_LEFT=3
int8 STRAIGHT_RIGHT=4
int8 ROTATE_LEFT=5
int8 ROTATE_RIGHT=6
int8 DIAGONAL_FORWARD_LEFT=7
int8 DIAGONAL_FORWARD_RIGHT=8
int8 DIAGONAL_BACKWARD_LEFT=9
int8 DIAGONAL_BACKWARD_RIGHT=10
int8 HEAD_UP=11
int8 HEAD_DOWN=12
int8 HEAD_MIDDLE=13

int8 ROTATE_LEFT_20_DEG=17
int8 ROTATE_RIGHT_20_DEG=18
int8 MIN_DRIVE_VAL=0
int8 MAX_DRIVE_VAL=18


int8 FASTEST=1
int8 SLOWEST=10
int8 MIN_SPEED_VAL=1
int8 MAX_SPEED_VAL=10


int8 drive
int8 speed

"""
  # Pseudo-constants
  STOP = 0
  FORWARD = 1
  BACKWARD = 2
  STRAIGHT_LEFT = 3
  STRAIGHT_RIGHT = 4
  ROTATE_LEFT = 5
  ROTATE_RIGHT = 6
  DIAGONAL_FORWARD_LEFT = 7
  DIAGONAL_FORWARD_RIGHT = 8
  DIAGONAL_BACKWARD_LEFT = 9
  DIAGONAL_BACKWARD_RIGHT = 10
  HEAD_UP = 11
  HEAD_DOWN = 12
  HEAD_MIDDLE = 13
  ROTATE_LEFT_20_DEG = 17
  ROTATE_RIGHT_20_DEG = 18
  MIN_DRIVE_VAL = 0
  MAX_DRIVE_VAL = 18
  FASTEST = 1
  SLOWEST = 10
  MIN_SPEED_VAL = 1
  MAX_SPEED_VAL = 10

  __slots__ = ['drive','speed']
  _slot_types = ['int8','int8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       drive,speed
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(man_drv_srvRequest, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.drive is None:
        self.drive = 0
      if self.speed is None:
        self.speed = 0
    else:
      self.drive = 0
      self.speed = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      _x = self
      buff.write(_struct_2b.pack(_x.drive, _x.speed))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.drive, _x.speed,) = _struct_2b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      _x = self
      buff.write(_struct_2b.pack(_x.drive, _x.speed))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 2
      (_x.drive, _x.speed,) = _struct_2b.unpack(str[start:end])
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
_struct_2b = struct.Struct("<2b")
"""autogenerated by genmsg_py from man_drv_srvResponse.msg. Do not edit."""
import roslib.message
import struct


class man_drv_srvResponse(roslib.message.Message):
  _md5sum = "d41d8cd98f00b204e9800998ecf8427e"
  _type = "rovio_shared/man_drv_srvResponse"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """
"""
  __slots__ = []
  _slot_types = []

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.
    
    The available fields are:
       
    
    @param args: complete set of field values, in .msg order
    @param kwds: use keyword arguments corresponding to message field names
    to set specific fields. 
    """
    if args or kwds:
      super(man_drv_srvResponse, self).__init__(*args, **kwds)

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    @param buff: buffer
    @type  buff: StringIO
    """
    try:
      pass
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    @param str: byte array of serialized message
    @type  str: str
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    @param buff: buffer
    @type  buff: StringIO
    @param numpy: numpy python module
    @type  numpy module
    """
    try:
      pass
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    @param str: byte array of serialized message
    @type  str: str
    @param numpy: numpy python module
    @type  numpy: module
    """
    try:
      end = 0
      return self
    except struct.error as e:
      raise roslib.message.DeserializationError(e) #most likely buffer underfill

_struct_I = roslib.message.struct_I
class man_drv_srv(roslib.message.ServiceDefinition):
  _type          = 'rovio_shared/man_drv_srv'
  _md5sum = '14675853bd9417686a1390c3fb2eaae6'
  _request_class  = man_drv_srvRequest
  _response_class = man_drv_srvResponse