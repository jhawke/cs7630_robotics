; Auto-generated. Do not edit!


(cl:in-package raptor_commander-srv)


;//! \htmlinclude getAdviceFLEE-request.msg.html

(cl:defclass <getAdviceFLEE-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getAdviceFLEE-request (<getAdviceFLEE-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAdviceFLEE-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAdviceFLEE-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_commander-srv:<getAdviceFLEE-request> is deprecated: use raptor_commander-srv:getAdviceFLEE-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAdviceFLEE-request>) ostream)
  "Serializes a message object of type '<getAdviceFLEE-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAdviceFLEE-request>) istream)
  "Deserializes a message object of type '<getAdviceFLEE-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAdviceFLEE-request>)))
  "Returns string type for a service object of type '<getAdviceFLEE-request>"
  "raptor_commander/getAdviceFLEERequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceFLEE-request)))
  "Returns string type for a service object of type 'getAdviceFLEE-request"
  "raptor_commander/getAdviceFLEERequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAdviceFLEE-request>)))
  "Returns md5sum for a message object of type '<getAdviceFLEE-request>"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAdviceFLEE-request)))
  "Returns md5sum for a message object of type 'getAdviceFLEE-request"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAdviceFLEE-request>)))
  "Returns full string definition for message of type '<getAdviceFLEE-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAdviceFLEE-request)))
  "Returns full string definition for message of type 'getAdviceFLEE-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAdviceFLEE-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAdviceFLEE-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getAdviceFLEE-request
))
;//! \htmlinclude getAdviceFLEE-response.msg.html

(cl:defclass <getAdviceFLEE-response> (roslisp-msg-protocol:ros-message)
  ((approachSpeed
    :reader approachSpeed
    :initarg :approachSpeed
    :type cl:integer
    :initform 0))
)

(cl:defclass getAdviceFLEE-response (<getAdviceFLEE-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAdviceFLEE-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAdviceFLEE-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_commander-srv:<getAdviceFLEE-response> is deprecated: use raptor_commander-srv:getAdviceFLEE-response instead.")))

(cl:ensure-generic-function 'approachSpeed-val :lambda-list '(m))
(cl:defmethod approachSpeed-val ((m <getAdviceFLEE-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor_commander-srv:approachSpeed-val is deprecated.  Use raptor_commander-srv:approachSpeed instead.")
  (approachSpeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAdviceFLEE-response>) ostream)
  "Serializes a message object of type '<getAdviceFLEE-response>"
  (cl:let* ((signed (cl:slot-value msg 'approachSpeed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAdviceFLEE-response>) istream)
  "Deserializes a message object of type '<getAdviceFLEE-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'approachSpeed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAdviceFLEE-response>)))
  "Returns string type for a service object of type '<getAdviceFLEE-response>"
  "raptor_commander/getAdviceFLEEResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceFLEE-response)))
  "Returns string type for a service object of type 'getAdviceFLEE-response"
  "raptor_commander/getAdviceFLEEResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAdviceFLEE-response>)))
  "Returns md5sum for a message object of type '<getAdviceFLEE-response>"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAdviceFLEE-response)))
  "Returns md5sum for a message object of type 'getAdviceFLEE-response"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAdviceFLEE-response>)))
  "Returns full string definition for message of type '<getAdviceFLEE-response>"
  (cl:format cl:nil "int32 approachSpeed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAdviceFLEE-response)))
  "Returns full string definition for message of type 'getAdviceFLEE-response"
  (cl:format cl:nil "int32 approachSpeed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAdviceFLEE-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAdviceFLEE-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getAdviceFLEE-response
    (cl:cons ':approachSpeed (approachSpeed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getAdviceFLEE)))
  'getAdviceFLEE-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getAdviceFLEE)))
  'getAdviceFLEE-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceFLEE)))
  "Returns string type for a service object of type '<getAdviceFLEE>"
  "raptor_commander/getAdviceFLEE")