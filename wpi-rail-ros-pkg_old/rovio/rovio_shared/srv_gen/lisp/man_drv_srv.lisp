; Auto-generated. Do not edit!


(cl:in-package rovio_shared-srv)


;//! \htmlinclude man_drv_srv-request.msg.html

(cl:defclass <man_drv_srv-request> (roslisp-msg-protocol:ros-message)
  ((drive
    :reader drive
    :initarg :drive
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass man_drv_srv-request (<man_drv_srv-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <man_drv_srv-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'man_drv_srv-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rovio_shared-srv:<man_drv_srv-request> is deprecated: use rovio_shared-srv:man_drv_srv-request instead.")))

(cl:ensure-generic-function 'drive-val :lambda-list '(m))
(cl:defmethod drive-val ((m <man_drv_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rovio_shared-srv:drive-val is deprecated.  Use rovio_shared-srv:drive instead.")
  (drive m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <man_drv_srv-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rovio_shared-srv:speed-val is deprecated.  Use rovio_shared-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<man_drv_srv-request>)))
    "Constants for message type '<man_drv_srv-request>"
  '((:STOP . 0)
    (:FORWARD . 1)
    (:BACKWARD . 2)
    (:STRAIGHT_LEFT . 3)
    (:STRAIGHT_RIGHT . 4)
    (:ROTATE_LEFT . 5)
    (:ROTATE_RIGHT . 6)
    (:DIAGONAL_FORWARD_LEFT . 7)
    (:DIAGONAL_FORWARD_RIGHT . 8)
    (:DIAGONAL_BACKWARD_LEFT . 9)
    (:DIAGONAL_BACKWARD_RIGHT . 10)
    (:HEAD_UP . 11)
    (:HEAD_DOWN . 12)
    (:HEAD_MIDDLE . 13)
    (:ROTATE_LEFT_20_DEG . 17)
    (:ROTATE_RIGHT_20_DEG . 18)
    (:MIN_DRIVE_VAL . 0)
    (:MAX_DRIVE_VAL . 18)
    (:FASTEST . 1)
    (:SLOWEST . 10)
    (:MIN_SPEED_VAL . 1)
    (:MAX_SPEED_VAL . 10))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'man_drv_srv-request)))
    "Constants for message type 'man_drv_srv-request"
  '((:STOP . 0)
    (:FORWARD . 1)
    (:BACKWARD . 2)
    (:STRAIGHT_LEFT . 3)
    (:STRAIGHT_RIGHT . 4)
    (:ROTATE_LEFT . 5)
    (:ROTATE_RIGHT . 6)
    (:DIAGONAL_FORWARD_LEFT . 7)
    (:DIAGONAL_FORWARD_RIGHT . 8)
    (:DIAGONAL_BACKWARD_LEFT . 9)
    (:DIAGONAL_BACKWARD_RIGHT . 10)
    (:HEAD_UP . 11)
    (:HEAD_DOWN . 12)
    (:HEAD_MIDDLE . 13)
    (:ROTATE_LEFT_20_DEG . 17)
    (:ROTATE_RIGHT_20_DEG . 18)
    (:MIN_DRIVE_VAL . 0)
    (:MAX_DRIVE_VAL . 18)
    (:FASTEST . 1)
    (:SLOWEST . 10)
    (:MIN_SPEED_VAL . 1)
    (:MAX_SPEED_VAL . 10))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <man_drv_srv-request>) ostream)
  "Serializes a message object of type '<man_drv_srv-request>"
  (cl:let* ((signed (cl:slot-value msg 'drive)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <man_drv_srv-request>) istream)
  "Deserializes a message object of type '<man_drv_srv-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'drive) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<man_drv_srv-request>)))
  "Returns string type for a service object of type '<man_drv_srv-request>"
  "rovio_shared/man_drv_srvRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'man_drv_srv-request)))
  "Returns string type for a service object of type 'man_drv_srv-request"
  "rovio_shared/man_drv_srvRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<man_drv_srv-request>)))
  "Returns md5sum for a message object of type '<man_drv_srv-request>"
  "14675853bd9417686a1390c3fb2eaae6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'man_drv_srv-request)))
  "Returns md5sum for a message object of type 'man_drv_srv-request"
  "14675853bd9417686a1390c3fb2eaae6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<man_drv_srv-request>)))
  "Returns full string definition for message of type '<man_drv_srv-request>"
  (cl:format cl:nil "~%int8 STOP=0~%int8 FORWARD=1~%int8 BACKWARD=2~%int8 STRAIGHT_LEFT=3~%int8 STRAIGHT_RIGHT=4~%int8 ROTATE_LEFT=5~%int8 ROTATE_RIGHT=6~%int8 DIAGONAL_FORWARD_LEFT=7~%int8 DIAGONAL_FORWARD_RIGHT=8~%int8 DIAGONAL_BACKWARD_LEFT=9~%int8 DIAGONAL_BACKWARD_RIGHT=10~%int8 HEAD_UP=11~%int8 HEAD_DOWN=12~%int8 HEAD_MIDDLE=13~%~%int8 ROTATE_LEFT_20_DEG=17~%int8 ROTATE_RIGHT_20_DEG=18~%int8 MIN_DRIVE_VAL=0~%int8 MAX_DRIVE_VAL=18~%~%~%int8 FASTEST=1~%int8 SLOWEST=10~%int8 MIN_SPEED_VAL=1~%int8 MAX_SPEED_VAL=10~%~%~%int8 drive~%int8 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'man_drv_srv-request)))
  "Returns full string definition for message of type 'man_drv_srv-request"
  (cl:format cl:nil "~%int8 STOP=0~%int8 FORWARD=1~%int8 BACKWARD=2~%int8 STRAIGHT_LEFT=3~%int8 STRAIGHT_RIGHT=4~%int8 ROTATE_LEFT=5~%int8 ROTATE_RIGHT=6~%int8 DIAGONAL_FORWARD_LEFT=7~%int8 DIAGONAL_FORWARD_RIGHT=8~%int8 DIAGONAL_BACKWARD_LEFT=9~%int8 DIAGONAL_BACKWARD_RIGHT=10~%int8 HEAD_UP=11~%int8 HEAD_DOWN=12~%int8 HEAD_MIDDLE=13~%~%int8 ROTATE_LEFT_20_DEG=17~%int8 ROTATE_RIGHT_20_DEG=18~%int8 MIN_DRIVE_VAL=0~%int8 MAX_DRIVE_VAL=18~%~%~%int8 FASTEST=1~%int8 SLOWEST=10~%int8 MIN_SPEED_VAL=1~%int8 MAX_SPEED_VAL=10~%~%~%int8 drive~%int8 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <man_drv_srv-request>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <man_drv_srv-request>))
  "Converts a ROS message object to a list"
  (cl:list 'man_drv_srv-request
    (cl:cons ':drive (drive msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude man_drv_srv-response.msg.html

(cl:defclass <man_drv_srv-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass man_drv_srv-response (<man_drv_srv-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <man_drv_srv-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'man_drv_srv-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rovio_shared-srv:<man_drv_srv-response> is deprecated: use rovio_shared-srv:man_drv_srv-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <man_drv_srv-response>) ostream)
  "Serializes a message object of type '<man_drv_srv-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <man_drv_srv-response>) istream)
  "Deserializes a message object of type '<man_drv_srv-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<man_drv_srv-response>)))
  "Returns string type for a service object of type '<man_drv_srv-response>"
  "rovio_shared/man_drv_srvResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'man_drv_srv-response)))
  "Returns string type for a service object of type 'man_drv_srv-response"
  "rovio_shared/man_drv_srvResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<man_drv_srv-response>)))
  "Returns md5sum for a message object of type '<man_drv_srv-response>"
  "14675853bd9417686a1390c3fb2eaae6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'man_drv_srv-response)))
  "Returns md5sum for a message object of type 'man_drv_srv-response"
  "14675853bd9417686a1390c3fb2eaae6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<man_drv_srv-response>)))
  "Returns full string definition for message of type '<man_drv_srv-response>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'man_drv_srv-response)))
  "Returns full string definition for message of type 'man_drv_srv-response"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <man_drv_srv-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <man_drv_srv-response>))
  "Converts a ROS message object to a list"
  (cl:list 'man_drv_srv-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'man_drv_srv)))
  'man_drv_srv-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'man_drv_srv)))
  'man_drv_srv-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'man_drv_srv)))
  "Returns string type for a service object of type '<man_drv_srv>"
  "rovio_shared/man_drv_srv")