; Auto-generated. Do not edit!


(cl:in-package raptor-srv)


;//! \htmlinclude polar_histogram-request.msg.html

(cl:defclass <polar_histogram-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass polar_histogram-request (<polar_histogram-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <polar_histogram-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'polar_histogram-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<polar_histogram-request> is deprecated: use raptor-srv:polar_histogram-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <polar_histogram-request>) ostream)
  "Serializes a message object of type '<polar_histogram-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <polar_histogram-request>) istream)
  "Deserializes a message object of type '<polar_histogram-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<polar_histogram-request>)))
  "Returns string type for a service object of type '<polar_histogram-request>"
  "raptor/polar_histogramRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'polar_histogram-request)))
  "Returns string type for a service object of type 'polar_histogram-request"
  "raptor/polar_histogramRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<polar_histogram-request>)))
  "Returns md5sum for a message object of type '<polar_histogram-request>"
  "ce468a59e33030eec49abe26e1f14754")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'polar_histogram-request)))
  "Returns md5sum for a message object of type 'polar_histogram-request"
  "ce468a59e33030eec49abe26e1f14754")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<polar_histogram-request>)))
  "Returns full string definition for message of type '<polar_histogram-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'polar_histogram-request)))
  "Returns full string definition for message of type 'polar_histogram-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <polar_histogram-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <polar_histogram-request>))
  "Converts a ROS message object to a list"
  (cl:list 'polar_histogram-request
))
;//! \htmlinclude polar_histogram-response.msg.html

(cl:defclass <polar_histogram-response> (roslisp-msg-protocol:ros-message)
  ((hist
    :reader hist
    :initarg :hist
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 360 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass polar_histogram-response (<polar_histogram-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <polar_histogram-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'polar_histogram-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name raptor-srv:<polar_histogram-response> is deprecated: use raptor-srv:polar_histogram-response instead.")))

(cl:ensure-generic-function 'hist-val :lambda-list '(m))
(cl:defmethod hist-val ((m <polar_histogram-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader raptor-srv:hist-val is deprecated.  Use raptor-srv:hist instead.")
  (hist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <polar_histogram-response>) ostream)
  "Serializes a message object of type '<polar_histogram-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'hist))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <polar_histogram-response>) istream)
  "Deserializes a message object of type '<polar_histogram-response>"
  (cl:setf (cl:slot-value msg 'hist) (cl:make-array 360))
  (cl:let ((vals (cl:slot-value msg 'hist)))
    (cl:dotimes (i 360)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<polar_histogram-response>)))
  "Returns string type for a service object of type '<polar_histogram-response>"
  "raptor/polar_histogramResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'polar_histogram-response)))
  "Returns string type for a service object of type 'polar_histogram-response"
  "raptor/polar_histogramResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<polar_histogram-response>)))
  "Returns md5sum for a message object of type '<polar_histogram-response>"
  "ce468a59e33030eec49abe26e1f14754")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'polar_histogram-response)))
  "Returns md5sum for a message object of type 'polar_histogram-response"
  "ce468a59e33030eec49abe26e1f14754")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<polar_histogram-response>)))
  "Returns full string definition for message of type '<polar_histogram-response>"
  (cl:format cl:nil "int16[360] hist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'polar_histogram-response)))
  "Returns full string definition for message of type 'polar_histogram-response"
  (cl:format cl:nil "int16[360] hist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <polar_histogram-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'hist) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <polar_histogram-response>))
  "Converts a ROS message object to a list"
  (cl:list 'polar_histogram-response
    (cl:cons ':hist (hist msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'polar_histogram)))
  'polar_histogram-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'polar_histogram)))
  'polar_histogram-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'polar_histogram)))
  "Returns string type for a service object of type '<polar_histogram>"
  "raptor/polar_histogram")