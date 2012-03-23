; Auto-generated. Do not edit!


(cl:in-package raptor-srv)


;//! \htmlinclude obstacle_histogram-request.msg.html

(cl:defclass <obstacle_histogram-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass obstacle_histogram-request (<obstacle_histogram-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacle_histogram-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacle_histogram-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<obstacle_histogram-request> is deprecated: use raptor-srv:obstacle_histogram-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacle_histogram-request>) ostream)
  "Serializes a message object of type '<obstacle_histogram-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacle_histogram-request>) istream)
  "Deserializes a message object of type '<obstacle_histogram-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacle_histogram-request>)))
  "Returns string type for a service object of type '<obstacle_histogram-request>"
  "raptor/obstacle_histogramRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle_histogram-request)))
  "Returns string type for a service object of type 'obstacle_histogram-request"
  "raptor/obstacle_histogramRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacle_histogram-request>)))
  "Returns md5sum for a message object of type '<obstacle_histogram-request>"
  "72a9ca006e68c1dccee9d542f2c99d30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacle_histogram-request)))
  "Returns md5sum for a message object of type 'obstacle_histogram-request"
  "72a9ca006e68c1dccee9d542f2c99d30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacle_histogram-request>)))
  "Returns full string definition for message of type '<obstacle_histogram-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacle_histogram-request)))
  "Returns full string definition for message of type 'obstacle_histogram-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacle_histogram-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacle_histogram-request>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacle_histogram-request
))
;//! \htmlinclude obstacle_histogram-response.msg.html

(cl:defclass <obstacle_histogram-response> (roslisp-msg-protocol:ros-message)
  ((hist
    :reader hist
    :initarg :hist
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 40 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass obstacle_histogram-response (<obstacle_histogram-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacle_histogram-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacle_histogram-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<obstacle_histogram-response> is deprecated: use raptor-srv:obstacle_histogram-response instead.")))

(cl:ensure-generic-function 'hist-val :lambda-list '(m))
(cl:defmethod hist-val ((m <obstacle_histogram-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:hist-val is deprecated.  Use raptor-srv:hist instead.")
  (hist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacle_histogram-response>) ostream)
  "Serializes a message object of type '<obstacle_histogram-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'hist))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacle_histogram-response>) istream)
  "Deserializes a message object of type '<obstacle_histogram-response>"
  (cl:setf (cl:slot-value msg 'hist) (cl:make-array 40))
  (cl:let ((vals (cl:slot-value msg 'hist)))
    (cl:dotimes (i 40)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacle_histogram-response>)))
  "Returns string type for a service object of type '<obstacle_histogram-response>"
  "raptor/obstacle_histogramResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle_histogram-response)))
  "Returns string type for a service object of type 'obstacle_histogram-response"
  "raptor/obstacle_histogramResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacle_histogram-response>)))
  "Returns md5sum for a message object of type '<obstacle_histogram-response>"
  "72a9ca006e68c1dccee9d542f2c99d30")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacle_histogram-response)))
  "Returns md5sum for a message object of type 'obstacle_histogram-response"
  "72a9ca006e68c1dccee9d542f2c99d30")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacle_histogram-response>)))
  "Returns full string definition for message of type '<obstacle_histogram-response>"
  (cl:format cl:nil "int16[40] hist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacle_histogram-response)))
  "Returns full string definition for message of type 'obstacle_histogram-response"
  (cl:format cl:nil "int16[40] hist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacle_histogram-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'hist) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacle_histogram-response>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacle_histogram-response
    (cl:cons ':hist (hist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'obstacle_histogram)))
  'obstacle_histogram-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'obstacle_histogram)))
  'obstacle_histogram-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle_histogram)))
  "Returns string type for a service object of type '<obstacle_histogram>"
  "raptor/obstacle_histogram")