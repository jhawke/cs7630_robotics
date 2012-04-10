; Auto-generated. Do not edit!


(cl:in-package raptor-msg)


;//! \htmlinclude rel_pos_req.msg.html

(cl:defclass <rel_pos_req> (roslisp-msg-protocol:ros-message)
  ((theta_rad
    :reader theta_rad
    :initarg :theta_rad
    :type cl:float
    :initform 0.0)
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

(cl:defclass rel_pos_req (<rel_pos_req>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rel_pos_req>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rel_pos_req)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-msg:<rel_pos_req> is deprecated: use raptor-msg:rel_pos_req instead.")))

(cl:ensure-generic-function 'theta_rad-val :lambda-list '(m))
(cl:defmethod theta_rad-val ((m <rel_pos_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-msg:theta_rad-val is deprecated.  Use raptor-msg:theta_rad instead.")
  (theta_rad m))

(cl:ensure-generic-function 'val-val :lambda-list '(m))
(cl:defmethod val-val ((m <rel_pos_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-msg:val-val is deprecated.  Use raptor-msg:val instead.")
  (val m))

(cl:ensure-generic-function 'time_sec-val :lambda-list '(m))
(cl:defmethod time_sec-val ((m <rel_pos_req>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-msg:time_sec-val is deprecated.  Use raptor-msg:time_sec instead.")
  (time_sec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rel_pos_req>) ostream)
  "Serializes a message object of type '<rel_pos_req>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta_rad))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rel_pos_req>) istream)
  "Deserializes a message object of type '<rel_pos_req>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta_rad) (roslisp-utils:decode-single-float-bits bits)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rel_pos_req>)))
  "Returns string type for a message object of type '<rel_pos_req>"
  "raptor/rel_pos_req")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rel_pos_req)))
  "Returns string type for a message object of type 'rel_pos_req"
  "raptor/rel_pos_req")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rel_pos_req>)))
  "Returns md5sum for a message object of type '<rel_pos_req>"
  "8a051cbaa402dd0d06d01a30168d23d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rel_pos_req)))
  "Returns md5sum for a message object of type 'rel_pos_req"
  "8a051cbaa402dd0d06d01a30168d23d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rel_pos_req>)))
  "Returns full string definition for message of type '<rel_pos_req>"
  (cl:format cl:nil "float32 theta_rad~%float32 val~%float32 time_sec~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rel_pos_req)))
  "Returns full string definition for message of type 'rel_pos_req"
  (cl:format cl:nil "float32 theta_rad~%float32 val~%float32 time_sec~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rel_pos_req>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rel_pos_req>))
  "Converts a ROS message object to a list"
  (cl:list 'rel_pos_req
    (cl:cons ':theta_rad (theta_rad msg))
    (cl:cons ':val (val msg))
    (cl:cons ':time_sec (time_sec msg))
))
