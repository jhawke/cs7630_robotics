; Auto-generated. Do not edit!


(cl:in-package raptor_commander-srv)


;//! \htmlinclude getAdviceDET-request.msg.html

(cl:defclass <getAdviceDET-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass getAdviceDET-request (<getAdviceDET-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAdviceDET-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAdviceDET-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_commander-srv:<getAdviceDET-request> is deprecated: use raptor_commander-srv:getAdviceDET-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAdviceDET-request>) ostream)
  "Serializes a message object of type '<getAdviceDET-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAdviceDET-request>) istream)
  "Deserializes a message object of type '<getAdviceDET-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAdviceDET-request>)))
  "Returns string type for a service object of type '<getAdviceDET-request>"
  "raptor_commander/getAdviceDETRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceDET-request)))
  "Returns string type for a service object of type 'getAdviceDET-request"
  "raptor_commander/getAdviceDETRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAdviceDET-request>)))
  "Returns md5sum for a message object of type '<getAdviceDET-request>"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAdviceDET-request)))
  "Returns md5sum for a message object of type 'getAdviceDET-request"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAdviceDET-request>)))
  "Returns full string definition for message of type '<getAdviceDET-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAdviceDET-request)))
  "Returns full string definition for message of type 'getAdviceDET-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAdviceDET-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAdviceDET-request>))
  "Converts a ROS message object to a list"
  (cl:list 'getAdviceDET-request
))
;//! \htmlinclude getAdviceDET-response.msg.html

(cl:defclass <getAdviceDET-response> (roslisp-msg-protocol:ros-message)
  ((approachSpeed
    :reader approachSpeed
    :initarg :approachSpeed
    :type cl:integer
    :initform 0))
)

(cl:defclass getAdviceDET-response (<getAdviceDET-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <getAdviceDET-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'getAdviceDET-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor_commander-srv:<getAdviceDET-response> is deprecated: use raptor_commander-srv:getAdviceDET-response instead.")))

(cl:ensure-generic-function 'approachSpeed-val :lambda-list '(m))
(cl:defmethod approachSpeed-val ((m <getAdviceDET-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor_commander-srv:approachSpeed-val is deprecated.  Use raptor_commander-srv:approachSpeed instead.")
  (approachSpeed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <getAdviceDET-response>) ostream)
  "Serializes a message object of type '<getAdviceDET-response>"
  (cl:let* ((signed (cl:slot-value msg 'approachSpeed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <getAdviceDET-response>) istream)
  "Deserializes a message object of type '<getAdviceDET-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'approachSpeed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<getAdviceDET-response>)))
  "Returns string type for a service object of type '<getAdviceDET-response>"
  "raptor_commander/getAdviceDETResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceDET-response)))
  "Returns string type for a service object of type 'getAdviceDET-response"
  "raptor_commander/getAdviceDETResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<getAdviceDET-response>)))
  "Returns md5sum for a message object of type '<getAdviceDET-response>"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'getAdviceDET-response)))
  "Returns md5sum for a message object of type 'getAdviceDET-response"
  "3564d8cb07efc73a81845dd1f9b6a7af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<getAdviceDET-response>)))
  "Returns full string definition for message of type '<getAdviceDET-response>"
  (cl:format cl:nil "int32 approachSpeed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'getAdviceDET-response)))
  "Returns full string definition for message of type 'getAdviceDET-response"
  (cl:format cl:nil "int32 approachSpeed~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <getAdviceDET-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <getAdviceDET-response>))
  "Converts a ROS message object to a list"
  (cl:list 'getAdviceDET-response
    (cl:cons ':approachSpeed (approachSpeed msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'getAdviceDET)))
  'getAdviceDET-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'getAdviceDET)))
  'getAdviceDET-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'getAdviceDET)))
  "Returns string type for a service object of type '<getAdviceDET>"
  "raptor_commander/getAdviceDET")