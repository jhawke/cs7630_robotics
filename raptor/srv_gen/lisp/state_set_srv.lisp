; Auto-generated. Do not edit!


(cl:in-package raptor-srv)


;//! \htmlinclude state_set_srv-request.msg.html

(cl:defclass <state_set_srv-request> (roslisp-msg-protocol:ros-message)
  ((model
    :reader model
    :initarg :model
    :type cl:fixnum
    :initform 0)
   (state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass state_set_srv-request (<state_set_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <state_set_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'state_set_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<state_set_srv-request> is deprecated: use raptor-srv:state_set_srv-request instead.")))

(cl:ensure-generic-function 'model-val :lambda-list '(m))
(cl:defmethod model-val ((m <state_set_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:model-val is deprecated.  Use raptor-srv:model instead.")
  (model m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <state_set_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:state-val is deprecated.  Use raptor-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <state_set_srv-request>) ostream)
  "Serializes a message object of type '<state_set_srv-request>"
  (cl:let* ((signed (cl:slot-value msg 'model)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <state_set_srv-request>) istream)
  "Deserializes a message object of type '<state_set_srv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'model) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<state_set_srv-request>)))
  "Returns string type for a service object of type '<state_set_srv-request>"
  "raptor/state_set_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state_set_srv-request)))
  "Returns string type for a service object of type 'state_set_srv-request"
  "raptor/state_set_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<state_set_srv-request>)))
  "Returns md5sum for a message object of type '<state_set_srv-request>"
  "75c5b792941ec6b4ff04cb01b9bf3ec3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'state_set_srv-request)))
  "Returns md5sum for a message object of type 'state_set_srv-request"
  "75c5b792941ec6b4ff04cb01b9bf3ec3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<state_set_srv-request>)))
  "Returns full string definition for message of type '<state_set_srv-request>"
  (cl:format cl:nil "int16 model~%int16 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'state_set_srv-request)))
  "Returns full string definition for message of type 'state_set_srv-request"
  (cl:format cl:nil "int16 model~%int16 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <state_set_srv-request>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <state_set_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'state_set_srv-request
    (cl:cons ':model (model msg))
    (cl:cons ':state (state msg))
))
;//! \htmlinclude state_set_srv-response.msg.html

(cl:defclass <state_set_srv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass state_set_srv-response (<state_set_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <state_set_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'state_set_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<state_set_srv-response> is deprecated: use raptor-srv:state_set_srv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <state_set_srv-response>) ostream)
  "Serializes a message object of type '<state_set_srv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <state_set_srv-response>) istream)
  "Deserializes a message object of type '<state_set_srv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<state_set_srv-response>)))
  "Returns string type for a service object of type '<state_set_srv-response>"
  "raptor/state_set_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state_set_srv-response)))
  "Returns string type for a service object of type 'state_set_srv-response"
  "raptor/state_set_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<state_set_srv-response>)))
  "Returns md5sum for a message object of type '<state_set_srv-response>"
  "75c5b792941ec6b4ff04cb01b9bf3ec3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'state_set_srv-response)))
  "Returns md5sum for a message object of type 'state_set_srv-response"
  "75c5b792941ec6b4ff04cb01b9bf3ec3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<state_set_srv-response>)))
  "Returns full string definition for message of type '<state_set_srv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'state_set_srv-response)))
  "Returns full string definition for message of type 'state_set_srv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <state_set_srv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <state_set_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'state_set_srv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'state_set_srv)))
  'state_set_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'state_set_srv)))
  'state_set_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'state_set_srv)))
  "Returns string type for a service object of type '<state_set_srv>"
  "raptor/state_set_srv")