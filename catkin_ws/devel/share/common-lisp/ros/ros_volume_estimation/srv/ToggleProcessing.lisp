; Auto-generated. Do not edit!


(cl:in-package ros_volume_estimation-srv)


;//! \htmlinclude ToggleProcessing-request.msg.html

(cl:defclass <ToggleProcessing-request> (roslisp-msg-protocol:ros-message)
  ((enabled
    :reader enabled
    :initarg :enabled
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass ToggleProcessing-request (<ToggleProcessing-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToggleProcessing-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToggleProcessing-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_volume_estimation-srv:<ToggleProcessing-request> is deprecated: use ros_volume_estimation-srv:ToggleProcessing-request instead.")))

(cl:ensure-generic-function 'enabled-val :lambda-list '(m))
(cl:defmethod enabled-val ((m <ToggleProcessing-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_volume_estimation-srv:enabled-val is deprecated.  Use ros_volume_estimation-srv:enabled instead.")
  (enabled m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToggleProcessing-request>) ostream)
  "Serializes a message object of type '<ToggleProcessing-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'enabled) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToggleProcessing-request>) istream)
  "Deserializes a message object of type '<ToggleProcessing-request>"
    (cl:setf (cl:slot-value msg 'enabled) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToggleProcessing-request>)))
  "Returns string type for a service object of type '<ToggleProcessing-request>"
  "ros_volume_estimation/ToggleProcessingRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleProcessing-request)))
  "Returns string type for a service object of type 'ToggleProcessing-request"
  "ros_volume_estimation/ToggleProcessingRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToggleProcessing-request>)))
  "Returns md5sum for a message object of type '<ToggleProcessing-request>"
  "91b5d40c59e3059df49ed9dc7354ebfc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToggleProcessing-request)))
  "Returns md5sum for a message object of type 'ToggleProcessing-request"
  "91b5d40c59e3059df49ed9dc7354ebfc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToggleProcessing-request>)))
  "Returns full string definition for message of type '<ToggleProcessing-request>"
  (cl:format cl:nil "bool enabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToggleProcessing-request)))
  "Returns full string definition for message of type 'ToggleProcessing-request"
  (cl:format cl:nil "bool enabled~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToggleProcessing-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToggleProcessing-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ToggleProcessing-request
    (cl:cons ':enabled (enabled msg))
))
;//! \htmlinclude ToggleProcessing-response.msg.html

(cl:defclass <ToggleProcessing-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass ToggleProcessing-response (<ToggleProcessing-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ToggleProcessing-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ToggleProcessing-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ros_volume_estimation-srv:<ToggleProcessing-response> is deprecated: use ros_volume_estimation-srv:ToggleProcessing-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <ToggleProcessing-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_volume_estimation-srv:success-val is deprecated.  Use ros_volume_estimation-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <ToggleProcessing-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ros_volume_estimation-srv:message-val is deprecated.  Use ros_volume_estimation-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ToggleProcessing-response>) ostream)
  "Serializes a message object of type '<ToggleProcessing-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ToggleProcessing-response>) istream)
  "Deserializes a message object of type '<ToggleProcessing-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ToggleProcessing-response>)))
  "Returns string type for a service object of type '<ToggleProcessing-response>"
  "ros_volume_estimation/ToggleProcessingResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleProcessing-response)))
  "Returns string type for a service object of type 'ToggleProcessing-response"
  "ros_volume_estimation/ToggleProcessingResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ToggleProcessing-response>)))
  "Returns md5sum for a message object of type '<ToggleProcessing-response>"
  "91b5d40c59e3059df49ed9dc7354ebfc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ToggleProcessing-response)))
  "Returns md5sum for a message object of type 'ToggleProcessing-response"
  "91b5d40c59e3059df49ed9dc7354ebfc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ToggleProcessing-response>)))
  "Returns full string definition for message of type '<ToggleProcessing-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ToggleProcessing-response)))
  "Returns full string definition for message of type 'ToggleProcessing-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ToggleProcessing-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ToggleProcessing-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ToggleProcessing-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ToggleProcessing)))
  'ToggleProcessing-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ToggleProcessing)))
  'ToggleProcessing-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ToggleProcessing)))
  "Returns string type for a service object of type '<ToggleProcessing>"
  "ros_volume_estimation/ToggleProcessing")