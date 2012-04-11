; Auto-generated. Do not edit!


(cl:in-package raptor_commander-srv)


;//! \htmlinclude getAdvicePD-request.msg.html

(cl:defclass <getAdvicePD-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getAdvicePD-request (<getAdvicePD-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAdvicePD-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAdvicePD-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_commander-srv:<getAdvicePD-request> is deprecated: use raptor_commander-srv:getAdvicePD-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAdvicePD-request>) ostream)
  "Serializes a message object of type '<getAdvicePD-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAdvicePD-request>) istream)
  "Deserializes a message object of type '<getAdvicePD-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAdvicePD-request>)))
  "Returns string type for a service object of type '<getAdvicePD-request>"
  "raptor_commander/getAdvicePDRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdvicePD-request)))
  "Returns string type for a service object of type 'getAdvicePD-request"
  "raptor_commander/getAdvicePDRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAdvicePD-request>)))
  "Returns md5sum for a message object of type '<getAdvicePD-request>"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAdvicePD-request)))
  "Returns md5sum for a message object of type 'getAdvicePD-request"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAdvicePD-request>)))
  "Returns full string definition for message of type '<getAdvicePD-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAdvicePD-request)))
  "Returns full string definition for message of type 'getAdvicePD-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAdvicePD-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAdvicePD-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getAdvicePD-request
))
;//! \htmlinclude getAdvicePD-response.msg.html

(cl:defclass <getAdvicePD-response> (roslisp-msg-protocol:ros-message)
  ((approachSpeed
    :reader approachSpeed
    :initarg :approachSpeed
    :type cl:integer
    :initform 0))
)

(cl:defclass getAdvicePD-response (<getAdvicePD-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAdvicePD-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAdvicePD-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_commander-srv:<getAdvicePD-response> is deprecated: use raptor_commander-srv:getAdvicePD-response instead.")))

(cl:ensure-generic-function 'approachSpeed-val :lambda-list '(m))
(cl:defmethod approachSpeed-val ((m <getAdvicePD-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor_commander-srv:approachSpeed-val is deprecated.  Use raptor_commander-srv:approachSpeed instead.")
  (approachSpeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAdvicePD-response>) ostream)
  "Serializes a message object of type '<getAdvicePD-response>"
  (cl:let* ((signed (cl:slot-value msg 'approachSpeed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAdvicePD-response>) istream)
  "Deserializes a message object of type '<getAdvicePD-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'approachSpeed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAdvicePD-response>)))
  "Returns string type for a service object of type '<getAdvicePD-response>"
  "raptor_commander/getAdvicePDResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdvicePD-response)))
  "Returns string type for a service object of type 'getAdvicePD-response"
  "raptor_commander/getAdvicePDResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAdvicePD-response>)))
  "Returns md5sum for a message object of type '<getAdvicePD-response>"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAdvicePD-response)))
  "Returns md5sum for a message object of type 'getAdvicePD-response"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAdvicePD-response>)))
  "Returns full string definition for message of type '<getAdvicePD-response>"
  (cl:format cl:nil "int32 approachSpeed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAdvicePD-response)))
  "Returns full string definition for message of type 'getAdvicePD-response"
  (cl:format cl:nil "int32 approachSpeed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAdvicePD-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAdvicePD-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getAdvicePD-response
    (cl:cons ':approachSpeed (approachSpeed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getAdvicePD)))
  'getAdvicePD-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getAdvicePD)))
  'getAdvicePD-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdvicePD)))
  "Returns string type for a service object of type '<getAdvicePD>"
  "raptor_commander/getAdvicePD")