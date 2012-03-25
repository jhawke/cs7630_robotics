; Auto-generated. Do not edit!


(cl:in-package raptor-srv)


;//! \htmlinclude twist_srv-request.msg.html

(cl:defclass <twist_srv-request> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type raptor-msg:Vector3
    :initform (cl:make-instance 'raptor-msg:Vector3))
   (angular
    :reader angular
    :initarg :angular
    :type raptor-msg:Vector3
    :initform (cl:make-instance 'raptor-msg:Vector3)))
)

(cl:defclass twist_srv-request (<twist_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <twist_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'twist_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<twist_srv-request> is deprecated: use raptor-srv:twist_srv-request instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <twist_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:linear-val is deprecated.  Use raptor-srv:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <twist_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:angular-val is deprecated.  Use raptor-srv:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <twist_srv-request>) ostream)
  "Serializes a message object of type '<twist_srv-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'linear) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'angular) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <twist_srv-request>) istream)
  "Deserializes a message object of type '<twist_srv-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'linear) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'angular) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<twist_srv-request>)))
  "Returns string type for a service object of type '<twist_srv-request>"
  "raptor/twist_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'twist_srv-request)))
  "Returns string type for a service object of type 'twist_srv-request"
  "raptor/twist_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<twist_srv-request>)))
  "Returns md5sum for a message object of type '<twist_srv-request>"
  "9f195f881246fdfa2798d1d3eebca84a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'twist_srv-request)))
  "Returns md5sum for a message object of type 'twist_srv-request"
  "9f195f881246fdfa2798d1d3eebca84a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<twist_srv-request>)))
  "Returns full string definition for message of type '<twist_srv-request>"
  (cl:format cl:nil "Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: raptor/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'twist_srv-request)))
  "Returns full string definition for message of type 'twist_srv-request"
  (cl:format cl:nil "Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: raptor/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <twist_srv-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'linear))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'angular))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <twist_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'twist_srv-request
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
))
;//! \htmlinclude twist_srv-response.msg.html

(cl:defclass <twist_srv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass twist_srv-response (<twist_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <twist_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'twist_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<twist_srv-response> is deprecated: use raptor-srv:twist_srv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <twist_srv-response>) ostream)
  "Serializes a message object of type '<twist_srv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <twist_srv-response>) istream)
  "Deserializes a message object of type '<twist_srv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<twist_srv-response>)))
  "Returns string type for a service object of type '<twist_srv-response>"
  "raptor/twist_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'twist_srv-response)))
  "Returns string type for a service object of type 'twist_srv-response"
  "raptor/twist_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<twist_srv-response>)))
  "Returns md5sum for a message object of type '<twist_srv-response>"
  "9f195f881246fdfa2798d1d3eebca84a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'twist_srv-response)))
  "Returns md5sum for a message object of type 'twist_srv-response"
  "9f195f881246fdfa2798d1d3eebca84a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<twist_srv-response>)))
  "Returns full string definition for message of type '<twist_srv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'twist_srv-response)))
  "Returns full string definition for message of type 'twist_srv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <twist_srv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <twist_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'twist_srv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'twist_srv)))
  'twist_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'twist_srv)))
  'twist_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'twist_srv)))
  "Returns string type for a service object of type '<twist_srv>"
  "raptor/twist_srv")