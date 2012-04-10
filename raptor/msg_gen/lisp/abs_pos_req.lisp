; Auto-generated. Do not edit!


(cl:in-package raptor-msg)


;//! \htmlinclude abs_pos_req.msg.html

(cl:defclass <abs_pos_req> (roslisp-msg-protocol:ros-message)
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

(cl:defclass abs_pos_req (<abs_pos_req>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <abs_pos_req>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'abs_pos_req)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-msg:<abs_pos_req> is deprecated: use raptor-msg:abs_pos_req instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <abs_pos_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-msg:x-val is deprecated.  Use raptor-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <abs_pos_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-msg:y-val is deprecated.  Use raptor-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <abs_pos_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-msg:val-val is deprecated.  Use raptor-msg:val instead.")
  (val m))

(cl:ensure-generic-function 'time_sec-val :lambda-list '(m))
(cl:defmethod time_sec-val ((m <abs_pos_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-msg:time_sec-val is deprecated.  Use raptor-msg:time_sec instead.")
  (time_sec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <abs_pos_req>) ostream)
  "Serializes a message object of type '<abs_pos_req>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <abs_pos_req>) istream)
  "Deserializes a message object of type '<abs_pos_req>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<abs_pos_req>)))
  "Returns string type for a message object of type '<abs_pos_req>"
  "raptor/abs_pos_req")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'abs_pos_req)))
  "Returns string type for a message object of type 'abs_pos_req"
  "raptor/abs_pos_req")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<abs_pos_req>)))
  "Returns md5sum for a message object of type '<abs_pos_req>"
  "2a22c4eddd9fc3ec356ab2303dd85ac7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'abs_pos_req)))
  "Returns md5sum for a message object of type 'abs_pos_req"
  "2a22c4eddd9fc3ec356ab2303dd85ac7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<abs_pos_req>)))
  "Returns full string definition for message of type '<abs_pos_req>"
  (cl:format cl:nil "int16 x~%int16 y~%float32 val~%float32 time_sec~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'abs_pos_req)))
  "Returns full string definition for message of type 'abs_pos_req"
  (cl:format cl:nil "int16 x~%int16 y~%float32 val~%float32 time_sec~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <abs_pos_req>))
  (cl:+ 0
     2
     2
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <abs_pos_req>))
  "Converts a ROS message object to a list"
  (cl:list 'abs_pos_req
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':val (val msg))
    (cl:cons ':time_sec (time_sec msg))
))
