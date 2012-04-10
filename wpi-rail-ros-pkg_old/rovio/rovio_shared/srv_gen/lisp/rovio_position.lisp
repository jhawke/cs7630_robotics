; Auto-generated. Do not edit!


(cl:in-package rovio_shared-srv)


;//! \htmlinclude rovio_position-request.msg.html

(cl:defclass <rovio_position-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass rovio_position-request (<rovio_position-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rovio_position-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rovio_position-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rovio_shared-srv:<rovio_position-request> is deprecated: use rovio_shared-srv:rovio_position-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rovio_position-request>) ostream)
  "Serializes a message object of type '<rovio_position-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rovio_position-request>) istream)
  "Deserializes a message object of type '<rovio_position-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rovio_position-request>)))
  "Returns string type for a service object of type '<rovio_position-request>"
  "rovio_shared/rovio_positionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rovio_position-request)))
  "Returns string type for a service object of type 'rovio_position-request"
  "rovio_shared/rovio_positionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rovio_position-request>)))
  "Returns md5sum for a message object of type '<rovio_position-request>"
  "3685765a49e4490fc51d6759409a036a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rovio_position-request)))
  "Returns md5sum for a message object of type 'rovio_position-request"
  "3685765a49e4490fc51d6759409a036a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rovio_position-request>)))
  "Returns full string definition for message of type '<rovio_position-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rovio_position-request)))
  "Returns full string definition for message of type 'rovio_position-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rovio_position-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rovio_position-request>))
  "Converts a ROS message object to a list"
  (cl:list 'rovio_position-request
))
;//! \htmlinclude rovio_position-response.msg.html

(cl:defclass <rovio_position-response> (roslisp-msg-protocol:ros-message)
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
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass rovio_position-response (<rovio_position-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <rovio_position-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'rovio_position-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rovio_shared-srv:<rovio_position-response> is deprecated: use rovio_shared-srv:rovio_position-response instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <rovio_position-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rovio_shared-srv:x-val is deprecated.  Use rovio_shared-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <rovio_position-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rovio_shared-srv:y-val is deprecated.  Use rovio_shared-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <rovio_position-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rovio_shared-srv:theta-val is deprecated.  Use rovio_shared-srv:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <rovio_position-response>) ostream)
  "Serializes a message object of type '<rovio_position-response>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <rovio_position-response>) istream)
  "Deserializes a message object of type '<rovio_position-response>"
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
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<rovio_position-response>)))
  "Returns string type for a service object of type '<rovio_position-response>"
  "rovio_shared/rovio_positionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rovio_position-response)))
  "Returns string type for a service object of type 'rovio_position-response"
  "rovio_shared/rovio_positionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<rovio_position-response>)))
  "Returns md5sum for a message object of type '<rovio_position-response>"
  "3685765a49e4490fc51d6759409a036a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'rovio_position-response)))
  "Returns md5sum for a message object of type 'rovio_position-response"
  "3685765a49e4490fc51d6759409a036a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<rovio_position-response>)))
  "Returns full string definition for message of type '<rovio_position-response>"
  (cl:format cl:nil "int16 	x~%int16 	y~%float32 theta~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'rovio_position-response)))
  "Returns full string definition for message of type 'rovio_position-response"
  (cl:format cl:nil "int16 	x~%int16 	y~%float32 theta~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <rovio_position-response>))
  (cl:+ 0
     2
     2
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <rovio_position-response>))
  "Converts a ROS message object to a list"
  (cl:list 'rovio_position-response
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'rovio_position)))
  'rovio_position-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'rovio_position)))
  'rovio_position-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'rovio_position)))
  "Returns string type for a service object of type '<rovio_position>"
  "rovio_shared/rovio_position")