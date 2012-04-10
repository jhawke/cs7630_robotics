; Auto-generated. Do not edit!


(cl:in-package rovio_shared-srv)


;//! \htmlinclude head_ctrl-request.msg.html

(cl:defclass <head_ctrl-request> (roslisp-msg-protocol:ros-message)
  ((head_pos
    :reader head_pos
    :initarg :head_pos
    :type cl:fixnum
    :initform 0))
)

(cl:defclass head_ctrl-request (<head_ctrl-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head_ctrl-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head_ctrl-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rovio_shared-srv:<head_ctrl-request> is deprecated: use rovio_shared-srv:head_ctrl-request instead.")))

(cl:ensure-generic-function 'head_pos-val :lambda-list '(m))
(cl:defmethod head_pos-val ((m <head_ctrl-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rovio_shared-srv:head_pos-val is deprecated.  Use rovio_shared-srv:head_pos instead.")
  (head_pos m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<head_ctrl-request>)))
    "Constants for message type '<head_ctrl-request>"
  '((:HEAD_UP . 11)
    (:HEAD_DOWN . 12)
    (:HEAD_MIDDLE . 13))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'head_ctrl-request)))
    "Constants for message type 'head_ctrl-request"
  '((:HEAD_UP . 11)
    (:HEAD_DOWN . 12)
    (:HEAD_MIDDLE . 13))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head_ctrl-request>) ostream)
  "Serializes a message object of type '<head_ctrl-request>"
  (cl:let* ((signed (cl:slot-value msg 'head_pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head_ctrl-request>) istream)
  "Deserializes a message object of type '<head_ctrl-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'head_pos) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head_ctrl-request>)))
  "Returns string type for a service object of type '<head_ctrl-request>"
  "rovio_shared/head_ctrlRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_ctrl-request)))
  "Returns string type for a service object of type 'head_ctrl-request"
  "rovio_shared/head_ctrlRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head_ctrl-request>)))
  "Returns md5sum for a message object of type '<head_ctrl-request>"
  "8fc91ecf3dc7f4ab832a70ed14ec95b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head_ctrl-request)))
  "Returns md5sum for a message object of type 'head_ctrl-request"
  "8fc91ecf3dc7f4ab832a70ed14ec95b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head_ctrl-request>)))
  "Returns full string definition for message of type '<head_ctrl-request>"
  (cl:format cl:nil "~%int8 HEAD_UP=11~%int8 HEAD_DOWN=12~%int8 HEAD_MIDDLE=13~%~%int8 head_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head_ctrl-request)))
  "Returns full string definition for message of type 'head_ctrl-request"
  (cl:format cl:nil "~%int8 HEAD_UP=11~%int8 HEAD_DOWN=12~%int8 HEAD_MIDDLE=13~%~%int8 head_pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head_ctrl-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head_ctrl-request>))
  "Converts a ROS message object to a list"
  (cl:list 'head_ctrl-request
    (cl:cons ':head_pos (head_pos msg))
))
;//! \htmlinclude head_ctrl-response.msg.html

(cl:defclass <head_ctrl-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:fixnum
    :initform 0))
)

(cl:defclass head_ctrl-response (<head_ctrl-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <head_ctrl-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'head_ctrl-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name rovio_shared-srv:<head_ctrl-response> is deprecated: use rovio_shared-srv:head_ctrl-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <head_ctrl-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader rovio_shared-srv:response-val is deprecated.  Use rovio_shared-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<head_ctrl-response>)))
    "Constants for message type '<head_ctrl-response>"
  '((:SUCCESS . 0)
    (:FAILURE . 1)
    (:ROBOT_BUSY . 2)
    (:FEATURE_NOT_IMPLEMENTED . 3)
    (:UNKNOWN_CGI_ACTION . 4)
    (:NO_NS_SIGNAL . 5)
    (:NO_EMPTY_PATH_AVAILABLE . 6)
    (:FAILED_TO_READ_PATH . 7)
    (:PATH_BASEADDRESS_NOT_INITIALIZED . 8)
    (:PATH_NOT_FOUND . 9)
    (:PATH_NAME_NOT_SPECIFIED . 10)
    (:NOT_RECORDING_PATH . 11)
    (:FLASH_NOT_INITIALIZED . 12)
    (:FAILED_TO_DELETE_PATH . 13)
    (:FAILED_TO_READ_FROM_FLASH . 14)
    (:FAILED_TO_WRITE_TO_FLASH . 15)
    (:FLASH_NOT_READY . 16)
    (:NO_MEMORY_AVAILABLE . 17)
    (:NO_MCU_PORT_AVAILABLE . 18)
    (:NO_NS_PORT_AVAILABLE . 19)
    (:NS_PACKET_CHECKSUM_ERROR . 20)
    (:NS_UART_READ_ERROR . 21)
    (:PARAMETER_OUTOFRANGE . 22)
    (:NO_PARAMETER . 23))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'head_ctrl-response)))
    "Constants for message type 'head_ctrl-response"
  '((:SUCCESS . 0)
    (:FAILURE . 1)
    (:ROBOT_BUSY . 2)
    (:FEATURE_NOT_IMPLEMENTED . 3)
    (:UNKNOWN_CGI_ACTION . 4)
    (:NO_NS_SIGNAL . 5)
    (:NO_EMPTY_PATH_AVAILABLE . 6)
    (:FAILED_TO_READ_PATH . 7)
    (:PATH_BASEADDRESS_NOT_INITIALIZED . 8)
    (:PATH_NOT_FOUND . 9)
    (:PATH_NAME_NOT_SPECIFIED . 10)
    (:NOT_RECORDING_PATH . 11)
    (:FLASH_NOT_INITIALIZED . 12)
    (:FAILED_TO_DELETE_PATH . 13)
    (:FAILED_TO_READ_FROM_FLASH . 14)
    (:FAILED_TO_WRITE_TO_FLASH . 15)
    (:FLASH_NOT_READY . 16)
    (:NO_MEMORY_AVAILABLE . 17)
    (:NO_MCU_PORT_AVAILABLE . 18)
    (:NO_NS_PORT_AVAILABLE . 19)
    (:NS_PACKET_CHECKSUM_ERROR . 20)
    (:NS_UART_READ_ERROR . 21)
    (:PARAMETER_OUTOFRANGE . 22)
    (:NO_PARAMETER . 23))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <head_ctrl-response>) ostream)
  "Serializes a message object of type '<head_ctrl-response>"
  (cl:let* ((signed (cl:slot-value msg 'response)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <head_ctrl-response>) istream)
  "Deserializes a message object of type '<head_ctrl-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<head_ctrl-response>)))
  "Returns string type for a service object of type '<head_ctrl-response>"
  "rovio_shared/head_ctrlResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_ctrl-response)))
  "Returns string type for a service object of type 'head_ctrl-response"
  "rovio_shared/head_ctrlResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<head_ctrl-response>)))
  "Returns md5sum for a message object of type '<head_ctrl-response>"
  "8fc91ecf3dc7f4ab832a70ed14ec95b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'head_ctrl-response)))
  "Returns md5sum for a message object of type 'head_ctrl-response"
  "8fc91ecf3dc7f4ab832a70ed14ec95b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<head_ctrl-response>)))
  "Returns full string definition for message of type '<head_ctrl-response>"
  (cl:format cl:nil "~%int8 SUCCESS=0~%int8 FAILURE=1~%int8 ROBOT_BUSY=2~%int8 FEATURE_NOT_IMPLEMENTED=3~%int8 UNKNOWN_CGI_ACTION=4~%int8 NO_NS_SIGNAL=5~%int8 NO_EMPTY_PATH_AVAILABLE=6~%int8 FAILED_TO_READ_PATH=7~%int8 PATH_BASEADDRESS_NOT_INITIALIZED=8~%int8 PATH_NOT_FOUND=9~%int8 PATH_NAME_NOT_SPECIFIED=10~%int8 NOT_RECORDING_PATH=11~%int8 FLASH_NOT_INITIALIZED=12~%int8 FAILED_TO_DELETE_PATH=13~%int8 FAILED_TO_READ_FROM_FLASH=14~%int8 FAILED_TO_WRITE_TO_FLASH=15~%int8 FLASH_NOT_READY=16~%int8 NO_MEMORY_AVAILABLE=17~%int8 NO_MCU_PORT_AVAILABLE=18~%int8 NO_NS_PORT_AVAILABLE=19~%int8 NS_PACKET_CHECKSUM_ERROR=20~%int8 NS_UART_READ_ERROR=21~%int8 PARAMETER_OUTOFRANGE=22~%int8 NO_PARAMETER=23~%~%int8 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'head_ctrl-response)))
  "Returns full string definition for message of type 'head_ctrl-response"
  (cl:format cl:nil "~%int8 SUCCESS=0~%int8 FAILURE=1~%int8 ROBOT_BUSY=2~%int8 FEATURE_NOT_IMPLEMENTED=3~%int8 UNKNOWN_CGI_ACTION=4~%int8 NO_NS_SIGNAL=5~%int8 NO_EMPTY_PATH_AVAILABLE=6~%int8 FAILED_TO_READ_PATH=7~%int8 PATH_BASEADDRESS_NOT_INITIALIZED=8~%int8 PATH_NOT_FOUND=9~%int8 PATH_NAME_NOT_SPECIFIED=10~%int8 NOT_RECORDING_PATH=11~%int8 FLASH_NOT_INITIALIZED=12~%int8 FAILED_TO_DELETE_PATH=13~%int8 FAILED_TO_READ_FROM_FLASH=14~%int8 FAILED_TO_WRITE_TO_FLASH=15~%int8 FLASH_NOT_READY=16~%int8 NO_MEMORY_AVAILABLE=17~%int8 NO_MCU_PORT_AVAILABLE=18~%int8 NO_NS_PORT_AVAILABLE=19~%int8 NS_PACKET_CHECKSUM_ERROR=20~%int8 NS_UART_READ_ERROR=21~%int8 PARAMETER_OUTOFRANGE=22~%int8 NO_PARAMETER=23~%~%int8 response~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <head_ctrl-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <head_ctrl-response>))
  "Converts a ROS message object to a list"
  (cl:list 'head_ctrl-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'head_ctrl)))
  'head_ctrl-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'head_ctrl)))
  'head_ctrl-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'head_ctrl)))
  "Returns string type for a service object of type '<head_ctrl>"
  "rovio_shared/head_ctrl")