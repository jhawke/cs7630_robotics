; Auto-generated. Do not edit!


(cl:in-package rovio_shared-srv)


;//! \htmlinclude wav_play-request.msg.html

(cl:defclass <wav_play-request> (roslisp-msg-protocol:ros-message)
  ((f
    :reader f
    :initarg :f
    :type cl:string
    :initform ""))
)

(cl:defclass wav_play-request (<wav_play-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wav_play-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wav_play-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rovio_shared-srv:<wav_play-request> is deprecated: use rovio_shared-srv:wav_play-request instead.")))

(cl:ensure-generic-function 'f-val :lambda-list '(m))
(cl:defmethod f-val ((m <wav_play-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rovio_shared-srv:f-val is deprecated.  Use rovio_shared-srv:f instead.")
  (f m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wav_play-request>) ostream)
  "Serializes a message object of type '<wav_play-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'f))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'f))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wav_play-request>) istream)
  "Deserializes a message object of type '<wav_play-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'f) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'f) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wav_play-request>)))
  "Returns string type for a service object of type '<wav_play-request>"
  "rovio_shared/wav_playRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wav_play-request)))
  "Returns string type for a service object of type 'wav_play-request"
  "rovio_shared/wav_playRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wav_play-request>)))
  "Returns md5sum for a message object of type '<wav_play-request>"
  "b7ec5ba08b681050147d22f3cf073480")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wav_play-request)))
  "Returns md5sum for a message object of type 'wav_play-request"
  "b7ec5ba08b681050147d22f3cf073480")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wav_play-request>)))
  "Returns full string definition for message of type '<wav_play-request>"
  (cl:format cl:nil "~%string f~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wav_play-request)))
  "Returns full string definition for message of type 'wav_play-request"
  (cl:format cl:nil "~%string f~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wav_play-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'f))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wav_play-request>))
  "Converts a ROS message object to a list"
  (cl:list 'wav_play-request
    (cl:cons ':f (f msg))
))
;//! \htmlinclude wav_play-response.msg.html

(cl:defclass <wav_play-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass wav_play-response (<wav_play-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <wav_play-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'wav_play-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rovio_shared-srv:<wav_play-response> is deprecated: use rovio_shared-srv:wav_play-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <wav_play-response>) ostream)
  "Serializes a message object of type '<wav_play-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <wav_play-response>) istream)
  "Deserializes a message object of type '<wav_play-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<wav_play-response>)))
  "Returns string type for a service object of type '<wav_play-response>"
  "rovio_shared/wav_playResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wav_play-response)))
  "Returns string type for a service object of type 'wav_play-response"
  "rovio_shared/wav_playResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<wav_play-response>)))
  "Returns md5sum for a message object of type '<wav_play-response>"
  "b7ec5ba08b681050147d22f3cf073480")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'wav_play-response)))
  "Returns md5sum for a message object of type 'wav_play-response"
  "b7ec5ba08b681050147d22f3cf073480")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<wav_play-response>)))
  "Returns full string definition for message of type '<wav_play-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'wav_play-response)))
  "Returns full string definition for message of type 'wav_play-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <wav_play-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <wav_play-response>))
  "Converts a ROS message object to a list"
  (cl:list 'wav_play-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'wav_play)))
  'wav_play-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'wav_play)))
  'wav_play-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'wav_play)))
  "Returns string type for a service object of type '<wav_play>"
  "rovio_shared/wav_play")