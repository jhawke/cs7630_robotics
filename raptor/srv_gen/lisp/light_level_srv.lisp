; Auto-generated. Do not edit!


(cl:in-package raptor-srv)


;//! \htmlinclude light_level_srv-request.msg.html

(cl:defclass <light_level_srv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass light_level_srv-request (<light_level_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <light_level_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'light_level_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<light_level_srv-request> is deprecated: use raptor-srv:light_level_srv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <light_level_srv-request>) ostream)
  "Serializes a message object of type '<light_level_srv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <light_level_srv-request>) istream)
  "Deserializes a message object of type '<light_level_srv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<light_level_srv-request>)))
  "Returns string type for a service object of type '<light_level_srv-request>"
  "raptor/light_level_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'light_level_srv-request)))
  "Returns string type for a service object of type 'light_level_srv-request"
  "raptor/light_level_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<light_level_srv-request>)))
  "Returns md5sum for a message object of type '<light_level_srv-request>"
  "883936641ce951bb1dc9b3fc31b6bb04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'light_level_srv-request)))
  "Returns md5sum for a message object of type 'light_level_srv-request"
  "883936641ce951bb1dc9b3fc31b6bb04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<light_level_srv-request>)))
  "Returns full string definition for message of type '<light_level_srv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'light_level_srv-request)))
  "Returns full string definition for message of type 'light_level_srv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <light_level_srv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <light_level_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'light_level_srv-request
))
;//! \htmlinclude light_level_srv-response.msg.html

(cl:defclass <light_level_srv-response> (roslisp-msg-protocol:ros-message)
  ((light_level
    :reader light_level
    :initarg :light_level
    :type cl:integer
    :initform 0))
)

(cl:defclass light_level_srv-response (<light_level_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <light_level_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'light_level_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<light_level_srv-response> is deprecated: use raptor-srv:light_level_srv-response instead.")))

(cl:ensure-generic-function 'light_level-val :lambda-list '(m))
(cl:defmethod light_level-val ((m <light_level_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:light_level-val is deprecated.  Use raptor-srv:light_level instead.")
  (light_level m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <light_level_srv-response>) ostream)
  "Serializes a message object of type '<light_level_srv-response>"
  (cl:let* ((signed (cl:slot-value msg 'light_level)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <light_level_srv-response>) istream)
  "Deserializes a message object of type '<light_level_srv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'light_level) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<light_level_srv-response>)))
  "Returns string type for a service object of type '<light_level_srv-response>"
  "raptor/light_level_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'light_level_srv-response)))
  "Returns string type for a service object of type 'light_level_srv-response"
  "raptor/light_level_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<light_level_srv-response>)))
  "Returns md5sum for a message object of type '<light_level_srv-response>"
  "883936641ce951bb1dc9b3fc31b6bb04")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'light_level_srv-response)))
  "Returns md5sum for a message object of type 'light_level_srv-response"
  "883936641ce951bb1dc9b3fc31b6bb04")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<light_level_srv-response>)))
  "Returns full string definition for message of type '<light_level_srv-response>"
  (cl:format cl:nil "int32 light_level~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'light_level_srv-response)))
  "Returns full string definition for message of type 'light_level_srv-response"
  (cl:format cl:nil "int32 light_level~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <light_level_srv-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <light_level_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'light_level_srv-response
    (cl:cons ':light_level (light_level msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'light_level_srv)))
  'light_level_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'light_level_srv)))
  'light_level_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'light_level_srv)))
  "Returns string type for a service object of type '<light_level_srv>"
  "raptor/light_level_srv")