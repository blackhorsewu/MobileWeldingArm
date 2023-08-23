;; Auto-generated. Do not edit!


(when (boundp 'bunker_msgs::BunkerStatus)
  (if (not (find-package "BUNKER_MSGS"))
    (make-package "BUNKER_MSGS"))
  (shadow 'BunkerStatus (find-package "BUNKER_MSGS")))
(unless (find-package "BUNKER_MSGS::BUNKERSTATUS")
  (make-package "BUNKER_MSGS::BUNKERSTATUS"))

(in-package "ROS")
;;//! \htmlinclude BunkerStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(intern "*MOTOR_ID_FRONT_RIGHT*" (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(shadow '*MOTOR_ID_FRONT_RIGHT* (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(defconstant bunker_msgs::BunkerStatus::*MOTOR_ID_FRONT_RIGHT* 0)
(intern "*MOTOR_ID_FRONT_LEFT*" (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(shadow '*MOTOR_ID_FRONT_LEFT* (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(defconstant bunker_msgs::BunkerStatus::*MOTOR_ID_FRONT_LEFT* 1)
(intern "*MOTOR_ID_REAR_RIGHT*" (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(shadow '*MOTOR_ID_REAR_RIGHT* (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(defconstant bunker_msgs::BunkerStatus::*MOTOR_ID_REAR_RIGHT* 2)
(intern "*MOTOR_ID_REAR_LEFT*" (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(shadow '*MOTOR_ID_REAR_LEFT* (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(defconstant bunker_msgs::BunkerStatus::*MOTOR_ID_REAR_LEFT* 3)
(intern "*LIGHT_ID_FRONT*" (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(shadow '*LIGHT_ID_FRONT* (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(defconstant bunker_msgs::BunkerStatus::*LIGHT_ID_FRONT* 0)
(intern "*LIGHT_ID_REAR*" (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(shadow '*LIGHT_ID_REAR* (find-package "BUNKER_MSGS::BUNKERSTATUS"))
(defconstant bunker_msgs::BunkerStatus::*LIGHT_ID_REAR* 1)

(defun bunker_msgs::BunkerStatus-to-symbol (const)
  (cond
        ((= const 0) 'bunker_msgs::BunkerStatus::*MOTOR_ID_FRONT_RIGHT*)
        ((= const 1) 'bunker_msgs::BunkerStatus::*MOTOR_ID_FRONT_LEFT*)
        ((= const 2) 'bunker_msgs::BunkerStatus::*MOTOR_ID_REAR_RIGHT*)
        ((= const 3) 'bunker_msgs::BunkerStatus::*MOTOR_ID_REAR_LEFT*)
        ((= const 0) 'bunker_msgs::BunkerStatus::*LIGHT_ID_FRONT*)
        ((= const 1) 'bunker_msgs::BunkerStatus::*LIGHT_ID_REAR*)
        (t nil)))

(defclass bunker_msgs::BunkerStatus
  :super ros::object
  :slots (_header _linear_velocity _angular_velocity _base_state _control_mode _fault_code _battery_voltage _motor_states ))

(defmethod bunker_msgs::BunkerStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:linear_velocity __linear_velocity) 0.0)
    ((:angular_velocity __angular_velocity) 0.0)
    ((:base_state __base_state) 0)
    ((:control_mode __control_mode) 0)
    ((:fault_code __fault_code) 0)
    ((:battery_voltage __battery_voltage) 0.0)
    ((:motor_states __motor_states) (let (r) (dotimes (i 2) (push (instance bunker_msgs::BunkerMotorState :init) r)) r))
    )
   (send-super :init)
   (setq _header __header)
   (setq _linear_velocity (float __linear_velocity))
   (setq _angular_velocity (float __angular_velocity))
   (setq _base_state (round __base_state))
   (setq _control_mode (round __control_mode))
   (setq _fault_code (round __fault_code))
   (setq _battery_voltage (float __battery_voltage))
   (setq _motor_states __motor_states)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:linear_velocity
   (&optional __linear_velocity)
   (if __linear_velocity (setq _linear_velocity __linear_velocity)) _linear_velocity)
  (:angular_velocity
   (&optional __angular_velocity)
   (if __angular_velocity (setq _angular_velocity __angular_velocity)) _angular_velocity)
  (:base_state
   (&optional __base_state)
   (if __base_state (setq _base_state __base_state)) _base_state)
  (:control_mode
   (&optional __control_mode)
   (if __control_mode (setq _control_mode __control_mode)) _control_mode)
  (:fault_code
   (&optional __fault_code)
   (if __fault_code (setq _fault_code __fault_code)) _fault_code)
  (:battery_voltage
   (&optional __battery_voltage)
   (if __battery_voltage (setq _battery_voltage __battery_voltage)) _battery_voltage)
  (:motor_states
   (&rest __motor_states)
   (if (keywordp (car __motor_states))
       (send* _motor_states __motor_states)
     (progn
       (if __motor_states (setq _motor_states (car __motor_states)))
       _motor_states)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64 _linear_velocity
    8
    ;; float64 _angular_velocity
    8
    ;; uint8 _base_state
    1
    ;; uint8 _control_mode
    1
    ;; uint16 _fault_code
    2
    ;; float64 _battery_voltage
    8
    ;; bunker_msgs/BunkerMotorState[2] _motor_states
    (apply #'+ (send-all _motor_states :serialization-length))
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64 _linear_velocity
       (sys::poke _linear_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angular_velocity
       (sys::poke _angular_velocity (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; uint8 _base_state
       (write-byte _base_state s)
     ;; uint8 _control_mode
       (write-byte _control_mode s)
     ;; uint16 _fault_code
       (write-word _fault_code s)
     ;; float64 _battery_voltage
       (sys::poke _battery_voltage (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bunker_msgs/BunkerMotorState[2] _motor_states
     (dolist (elem _motor_states)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64 _linear_velocity
     (setq _linear_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angular_velocity
     (setq _angular_velocity (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; uint8 _base_state
     (setq _base_state (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _control_mode
     (setq _control_mode (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint16 _fault_code
     (setq _fault_code (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; float64 _battery_voltage
     (setq _battery_voltage (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bunker_msgs/BunkerMotorState[2] _motor_states
   (dotimes (i 2)
     (send (elt _motor_states i) :deserialize buf ptr-) (incf ptr- (send (elt _motor_states i) :serialization-length))
     )
   ;;
   self)
  )

(setf (get bunker_msgs::BunkerStatus :md5sum-) "a86f055b47966ef7079cc8ff92ba8758")
(setf (get bunker_msgs::BunkerStatus :datatype-) "bunker_msgs/BunkerStatus")
(setf (get bunker_msgs::BunkerStatus :definition-)
      "Header header

int8 MOTOR_ID_FRONT_RIGHT = 0
int8 MOTOR_ID_FRONT_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2
int8 MOTOR_ID_REAR_LEFT = 3

int8 LIGHT_ID_FRONT = 0
int8 LIGHT_ID_REAR = 1

# motion state
float64 linear_velocity
float64 angular_velocity

# base state
uint8 base_state
uint8 control_mode
uint16 fault_code
float64 battery_voltage

# motor state
BunkerMotorState[2] motor_states

# light state
#bool light_control_enabled
#BunkerLightState front_light_state
#BunkerLightState rear_light_state

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: bunker_msgs/BunkerMotorState
float64 current
float64 rpm
float64 temperature
")



(provide :bunker_msgs/BunkerStatus "a86f055b47966ef7079cc8ff92ba8758")


