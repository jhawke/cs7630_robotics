; Auto-generated. Do not edit!


(cl:in-package raptor-srv)


;//! \htmlinclude position_request_srv-request.msg.html

(cl:defclass <position_request_srv-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:fixnum
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:fixnum
    :initform 0)
   (val
    :reader val
    :initarg :val
    :type cl:float
    :initform 0.0)
   (time_sec
    :reader time_sec
    :initarg :time_sec
    :type cl:float
    :initform 0.0))
)

(cl:defclass position_request_srv-request (<position_request_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <position_request_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'position_request_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<position_request_srv-request> is deprecated: use raptor-srv:position_request_srv-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <position_request_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:x-val is deprecated.  Use raptor-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <position_request_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:y-val is deprecated.  Use raptor-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <position_request_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:val-val is deprecated.  Use raptor-srv:val instead.")
  (val m))

(cl:ensure-generic-function 'time_sec-val :lambda-list '(m))
(cl:defmethod time_sec-val ((m <position_request_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:time_sec-val is deprecated.  Use raptor-srv:time_sec instead.")
  (time_sec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <position_request_srv-request>) ostream)
  "Serializes a message object of type '<position_request_srv-request>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'val))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'time_sec))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <position_request_srv-request>) istream)
  "Deserializes a message object of type '<position_request_srv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'val) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'time_sec) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<position_request_srv-request>)))
  "Returns string type for a service object of type '<position_request_srv-request>"
  "raptor/position_request_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'position_request_srv-request)))
  "Returns string type for a service object of type 'position_request_srv-request"
  "raptor/position_request_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<position_request_srv-request>)))
  "Returns md5sum for a message object of type '<position_request_srv-request>"
  "2a22c4eddd9fc3ec356ab2303dd85ac7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'position_request_srv-request)))
  "Returns md5sum for a message object of type 'position_request_srv-request"
  "2a22c4eddd9fc3ec356ab2303dd85ac7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<position_request_srv-request>)))
  "Returns full string definition for message of type '<position_request_srv-request>"
  (cl:format cl:nil "int16 x~%int16 y~%float32 val~%float32 time_sec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'position_request_srv-request)))
  "Returns full string definition for message of type 'position_request_srv-request"
  (cl:format cl:nil "int16 x~%int16 y~%float32 val~%float32 time_sec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <position_request_srv-request>))
  (cl:+ 0
     2
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <position_request_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'position_request_srv-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':val (val msg))
    (cl:cons ':time_sec (time_sec msg))
))
;//! \htmlinclude position_request_srv-response.msg.html

(cl:defclass <position_request_srv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass position_request_srv-response (<position_request_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <position_request_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'position_request_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<position_request_srv-response> is deprecated: use raptor-srv:position_request_srv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <position_request_srv-response>) ostream)
  "Serializes a message object of type '<position_request_srv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <position_request_srv-response>) istream)
  "Deserializes a message object of type '<position_request_srv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<position_request_srv-response>)))
  "Returns string type for a service object of type '<position_request_srv-response>"
  "raptor/position_request_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'position_request_srv-response)))
  "Returns string type for a service object of type 'position_request_srv-response"
  "raptor/position_request_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<position_request_srv-response>)))
  "Returns md5sum for a message object of type '<position_request_srv-response>"
  "2a22c4eddd9fc3ec356ab2303dd85ac7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'position_request_srv-response)))
  "Returns md5sum for a message object of type 'position_request_srv-response"
  "2a22c4eddd9fc3ec356ab2303dd85ac7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<position_request_srv-response>)))
  "Returns full string definition for message of type '<position_request_srv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'position_request_srv-response)))
  "Returns full string definition for message of type 'position_request_srv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <position_request_srv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <position_request_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'position_request_srv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'position_request_srv)))
  'position_request_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'position_request_srv)))
  'position_request_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'position_request_srv)))
  "Returns string type for a service object of type '<position_request_srv>"
  "raptor/position_request_srv")