; Auto-generated. Do not edit!


(cl:in-package raptor-srv)


;//! \htmlinclude distance_adv_srv-request.msg.html

(cl:defclass <distance_adv_srv-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass distance_adv_srv-request (<distance_adv_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <distance_adv_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'distance_adv_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<distance_adv_srv-request> is deprecated: use raptor-srv:distance_adv_srv-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <distance_adv_srv-request>) ostream)
  "Serializes a message object of type '<distance_adv_srv-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <distance_adv_srv-request>) istream)
  "Deserializes a message object of type '<distance_adv_srv-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<distance_adv_srv-request>)))
  "Returns string type for a service object of type '<distance_adv_srv-request>"
  "raptor/distance_adv_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'distance_adv_srv-request)))
  "Returns string type for a service object of type 'distance_adv_srv-request"
  "raptor/distance_adv_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<distance_adv_srv-request>)))
  "Returns md5sum for a message object of type '<distance_adv_srv-request>"
  "d8c624346e7957f0fb7d529b8a231ec2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'distance_adv_srv-request)))
  "Returns md5sum for a message object of type 'distance_adv_srv-request"
  "d8c624346e7957f0fb7d529b8a231ec2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<distance_adv_srv-request>)))
  "Returns full string definition for message of type '<distance_adv_srv-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'distance_adv_srv-request)))
  "Returns full string definition for message of type 'distance_adv_srv-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <distance_adv_srv-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <distance_adv_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'distance_adv_srv-request
))
;//! \htmlinclude distance_adv_srv-response.msg.html

(cl:defclass <distance_adv_srv-response> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:fixnum
    :initform 0))
)

(cl:defclass distance_adv_srv-response (<distance_adv_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <distance_adv_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'distance_adv_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<distance_adv_srv-response> is deprecated: use raptor-srv:distance_adv_srv-response instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <distance_adv_srv-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:distance-val is deprecated.  Use raptor-srv:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <distance_adv_srv-response>) ostream)
  "Serializes a message object of type '<distance_adv_srv-response>"
  (cl:let* ((signed (cl:slot-value msg 'distance)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <distance_adv_srv-response>) istream)
  "Deserializes a message object of type '<distance_adv_srv-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'distance) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<distance_adv_srv-response>)))
  "Returns string type for a service object of type '<distance_adv_srv-response>"
  "raptor/distance_adv_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'distance_adv_srv-response)))
  "Returns string type for a service object of type 'distance_adv_srv-response"
  "raptor/distance_adv_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<distance_adv_srv-response>)))
  "Returns md5sum for a message object of type '<distance_adv_srv-response>"
  "d8c624346e7957f0fb7d529b8a231ec2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'distance_adv_srv-response)))
  "Returns md5sum for a message object of type 'distance_adv_srv-response"
  "d8c624346e7957f0fb7d529b8a231ec2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<distance_adv_srv-response>)))
  "Returns full string definition for message of type '<distance_adv_srv-response>"
  (cl:format cl:nil "int16 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'distance_adv_srv-response)))
  "Returns full string definition for message of type 'distance_adv_srv-response"
  (cl:format cl:nil "int16 distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <distance_adv_srv-response>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <distance_adv_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'distance_adv_srv-response
    (cl:cons ':distance (distance msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'distance_adv_srv)))
  'distance_adv_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'distance_adv_srv)))
  'distance_adv_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'distance_adv_srv)))
  "Returns string type for a service object of type '<distance_adv_srv>"
  "raptor/distance_adv_srv")