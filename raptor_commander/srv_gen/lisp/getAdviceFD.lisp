; Auto-generated. Do not edit!


(cl:in-package raptor_commander-srv)


;//! \htmlinclude getAdviceFD-request.msg.html

(cl:defclass <getAdviceFD-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getAdviceFD-request (<getAdviceFD-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAdviceFD-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAdviceFD-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_commander-srv:<getAdviceFD-request> is deprecated: use raptor_commander-srv:getAdviceFD-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAdviceFD-request>) ostream)
  "Serializes a message object of type '<getAdviceFD-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAdviceFD-request>) istream)
  "Deserializes a message object of type '<getAdviceFD-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAdviceFD-request>)))
  "Returns string type for a service object of type '<getAdviceFD-request>"
  "raptor_commander/getAdviceFDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceFD-request)))
  "Returns string type for a service object of type 'getAdviceFD-request"
  "raptor_commander/getAdviceFDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAdviceFD-request>)))
  "Returns md5sum for a message object of type '<getAdviceFD-request>"
  "030e1a5167ee4ae90c06ed17870ddc58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAdviceFD-request)))
  "Returns md5sum for a message object of type 'getAdviceFD-request"
  "030e1a5167ee4ae90c06ed17870ddc58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAdviceFD-request>)))
  "Returns full string definition for message of type '<getAdviceFD-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAdviceFD-request)))
  "Returns full string definition for message of type 'getAdviceFD-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAdviceFD-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAdviceFD-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getAdviceFD-request
))
;//! \htmlinclude getAdviceFD-response.msg.html

(cl:defclass <getAdviceFD-response> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:integer
    :initform 0)
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass getAdviceFD-response (<getAdviceFD-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAdviceFD-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAdviceFD-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_commander-srv:<getAdviceFD-response> is deprecated: use raptor_commander-srv:getAdviceFD-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <getAdviceFD-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor_commander-srv:x-val is deprecated.  Use raptor_commander-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <getAdviceFD-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor_commander-srv:y-val is deprecated.  Use raptor_commander-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <getAdviceFD-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor_commander-srv:theta-val is deprecated.  Use raptor_commander-srv:theta instead.")
  (theta m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <getAdviceFD-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor_commander-srv:value-val is deprecated.  Use raptor_commander-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAdviceFD-response>) ostream)
  "Serializes a message object of type '<getAdviceFD-response>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'theta)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAdviceFD-response>) istream)
  "Deserializes a message object of type '<getAdviceFD-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'theta) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAdviceFD-response>)))
  "Returns string type for a service object of type '<getAdviceFD-response>"
  "raptor_commander/getAdviceFDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceFD-response)))
  "Returns string type for a service object of type 'getAdviceFD-response"
  "raptor_commander/getAdviceFDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAdviceFD-response>)))
  "Returns md5sum for a message object of type '<getAdviceFD-response>"
  "030e1a5167ee4ae90c06ed17870ddc58")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAdviceFD-response)))
  "Returns md5sum for a message object of type 'getAdviceFD-response"
  "030e1a5167ee4ae90c06ed17870ddc58")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAdviceFD-response>)))
  "Returns full string definition for message of type '<getAdviceFD-response>"
  (cl:format cl:nil "int32 x~%int32 y~%int32 theta~%int32 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAdviceFD-response)))
  "Returns full string definition for message of type 'getAdviceFD-response"
  (cl:format cl:nil "int32 x~%int32 y~%int32 theta~%int32 value~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAdviceFD-response>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAdviceFD-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getAdviceFD-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':value (value msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getAdviceFD)))
  'getAdviceFD-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getAdviceFD)))
  'getAdviceFD-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceFD)))
  "Returns string type for a service object of type '<getAdviceFD>"
  "raptor_commander/getAdviceFD")