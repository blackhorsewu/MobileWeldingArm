; Auto-generated. Do not edit!


(cl:in-package bunker_msgs-msg)


;//! \htmlinclude BunkerMotorState.msg.html

(cl:defclass <BunkerMotorState> (roslisp-msg-protocol:ros-message)
  ((current
    :reader current
    :initarg :current
    :type cl:float
    :initform 0.0)
   (rpm
    :reader rpm
    :initarg :rpm
    :type cl:float
    :initform 0.0)
   (temperature
    :reader temperature
    :initarg :temperature
    :type cl:float
    :initform 0.0))
)

(cl:defclass BunkerMotorState (<BunkerMotorState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BunkerMotorState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BunkerMotorState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bunker_msgs-msg:<BunkerMotorState> is deprecated: use bunker_msgs-msg:BunkerMotorState instead.")))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <BunkerMotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bunker_msgs-msg:current-val is deprecated.  Use bunker_msgs-msg:current instead.")
  (current m))

(cl:ensure-generic-function 'rpm-val :lambda-list '(m))
(cl:defmethod rpm-val ((m <BunkerMotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bunker_msgs-msg:rpm-val is deprecated.  Use bunker_msgs-msg:rpm instead.")
  (rpm m))

(cl:ensure-generic-function 'temperature-val :lambda-list '(m))
(cl:defmethod temperature-val ((m <BunkerMotorState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bunker_msgs-msg:temperature-val is deprecated.  Use bunker_msgs-msg:temperature instead.")
  (temperature m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BunkerMotorState>) ostream)
  "Serializes a message object of type '<BunkerMotorState>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rpm))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'temperature))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BunkerMotorState>) istream)
  "Deserializes a message object of type '<BunkerMotorState>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'current) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rpm) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'temperature) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BunkerMotorState>)))
  "Returns string type for a message object of type '<BunkerMotorState>"
  "bunker_msgs/BunkerMotorState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BunkerMotorState)))
  "Returns string type for a message object of type 'BunkerMotorState"
  "bunker_msgs/BunkerMotorState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BunkerMotorState>)))
  "Returns md5sum for a message object of type '<BunkerMotorState>"
  "9380628b50ebdc90ce46d4147360680d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BunkerMotorState)))
  "Returns md5sum for a message object of type 'BunkerMotorState"
  "9380628b50ebdc90ce46d4147360680d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BunkerMotorState>)))
  "Returns full string definition for message of type '<BunkerMotorState>"
  (cl:format cl:nil "float64 current~%float64 rpm~%float64 temperature~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BunkerMotorState)))
  "Returns full string definition for message of type 'BunkerMotorState"
  (cl:format cl:nil "float64 current~%float64 rpm~%float64 temperature~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BunkerMotorState>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BunkerMotorState>))
  "Converts a ROS message object to a list"
  (cl:list 'BunkerMotorState
    (cl:cons ':current (current msg))
    (cl:cons ':rpm (rpm msg))
    (cl:cons ':temperature (temperature msg))
))
