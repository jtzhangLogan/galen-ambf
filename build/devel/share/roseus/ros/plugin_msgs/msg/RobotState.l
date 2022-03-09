;; Auto-generated. Do not edit!


(when (boundp 'plugin_msgs::RobotState)
  (if (not (find-package "PLUGIN_MSGS"))
    (make-package "PLUGIN_MSGS"))
  (shadow 'RobotState (find-package "PLUGIN_MSGS")))
(unless (find-package "PLUGIN_MSGS::ROBOTSTATE")
  (make-package "PLUGIN_MSGS::ROBOTSTATE"))

(in-package "ROS")
;;//! \htmlinclude RobotState.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass plugin_msgs::RobotState
  :super ros::object
  :slots (_header _joint_position _joint_velocity _joint_position_goal _joint_velocity_goal _joint_position_error _joint_velocity_error _end_effector_frame _cartesian_position _cartesian_velocity _cartesian_position_goal _cartesian_velocity_goal _cartesian_position_error _cartesian_velocity_error ))

(defmethod plugin_msgs::RobotState
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:joint_position __joint_position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_velocity __joint_velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_position_goal __joint_position_goal) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_velocity_goal __joint_velocity_goal) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_position_error __joint_position_error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:joint_velocity_error __joint_velocity_error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:end_effector_frame __end_effector_frame) (instance geometry_msgs::Pose :init))
    ((:cartesian_position __cartesian_position) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cartesian_velocity __cartesian_velocity) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cartesian_position_goal __cartesian_position_goal) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cartesian_velocity_goal __cartesian_velocity_goal) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cartesian_position_error __cartesian_position_error) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:cartesian_velocity_error __cartesian_velocity_error) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _joint_position __joint_position)
   (setq _joint_velocity __joint_velocity)
   (setq _joint_position_goal __joint_position_goal)
   (setq _joint_velocity_goal __joint_velocity_goal)
   (setq _joint_position_error __joint_position_error)
   (setq _joint_velocity_error __joint_velocity_error)
   (setq _end_effector_frame __end_effector_frame)
   (setq _cartesian_position __cartesian_position)
   (setq _cartesian_velocity __cartesian_velocity)
   (setq _cartesian_position_goal __cartesian_position_goal)
   (setq _cartesian_velocity_goal __cartesian_velocity_goal)
   (setq _cartesian_position_error __cartesian_position_error)
   (setq _cartesian_velocity_error __cartesian_velocity_error)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:joint_position
   (&optional __joint_position)
   (if __joint_position (setq _joint_position __joint_position)) _joint_position)
  (:joint_velocity
   (&optional __joint_velocity)
   (if __joint_velocity (setq _joint_velocity __joint_velocity)) _joint_velocity)
  (:joint_position_goal
   (&optional __joint_position_goal)
   (if __joint_position_goal (setq _joint_position_goal __joint_position_goal)) _joint_position_goal)
  (:joint_velocity_goal
   (&optional __joint_velocity_goal)
   (if __joint_velocity_goal (setq _joint_velocity_goal __joint_velocity_goal)) _joint_velocity_goal)
  (:joint_position_error
   (&optional __joint_position_error)
   (if __joint_position_error (setq _joint_position_error __joint_position_error)) _joint_position_error)
  (:joint_velocity_error
   (&optional __joint_velocity_error)
   (if __joint_velocity_error (setq _joint_velocity_error __joint_velocity_error)) _joint_velocity_error)
  (:end_effector_frame
   (&rest __end_effector_frame)
   (if (keywordp (car __end_effector_frame))
       (send* _end_effector_frame __end_effector_frame)
     (progn
       (if __end_effector_frame (setq _end_effector_frame (car __end_effector_frame)))
       _end_effector_frame)))
  (:cartesian_position
   (&optional __cartesian_position)
   (if __cartesian_position (setq _cartesian_position __cartesian_position)) _cartesian_position)
  (:cartesian_velocity
   (&optional __cartesian_velocity)
   (if __cartesian_velocity (setq _cartesian_velocity __cartesian_velocity)) _cartesian_velocity)
  (:cartesian_position_goal
   (&optional __cartesian_position_goal)
   (if __cartesian_position_goal (setq _cartesian_position_goal __cartesian_position_goal)) _cartesian_position_goal)
  (:cartesian_velocity_goal
   (&optional __cartesian_velocity_goal)
   (if __cartesian_velocity_goal (setq _cartesian_velocity_goal __cartesian_velocity_goal)) _cartesian_velocity_goal)
  (:cartesian_position_error
   (&optional __cartesian_position_error)
   (if __cartesian_position_error (setq _cartesian_position_error __cartesian_position_error)) _cartesian_position_error)
  (:cartesian_velocity_error
   (&optional __cartesian_velocity_error)
   (if __cartesian_velocity_error (setq _cartesian_velocity_error __cartesian_velocity_error)) _cartesian_velocity_error)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[] _joint_position
    (* 8    (length _joint_position)) 4
    ;; float64[] _joint_velocity
    (* 8    (length _joint_velocity)) 4
    ;; float64[] _joint_position_goal
    (* 8    (length _joint_position_goal)) 4
    ;; float64[] _joint_velocity_goal
    (* 8    (length _joint_velocity_goal)) 4
    ;; float64[] _joint_position_error
    (* 8    (length _joint_position_error)) 4
    ;; float64[] _joint_velocity_error
    (* 8    (length _joint_velocity_error)) 4
    ;; geometry_msgs/Pose _end_effector_frame
    (send _end_effector_frame :serialization-length)
    ;; float64[] _cartesian_position
    (* 8    (length _cartesian_position)) 4
    ;; float64[] _cartesian_velocity
    (* 8    (length _cartesian_velocity)) 4
    ;; float64[] _cartesian_position_goal
    (* 8    (length _cartesian_position_goal)) 4
    ;; float64[] _cartesian_velocity_goal
    (* 8    (length _cartesian_velocity_goal)) 4
    ;; float64[] _cartesian_position_error
    (* 8    (length _cartesian_position_error)) 4
    ;; float64[] _cartesian_velocity_error
    (* 8    (length _cartesian_velocity_error)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[] _joint_position
     (write-long (length _joint_position) s)
     (dotimes (i (length _joint_position))
       (sys::poke (elt _joint_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_velocity
     (write-long (length _joint_velocity) s)
     (dotimes (i (length _joint_velocity))
       (sys::poke (elt _joint_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_position_goal
     (write-long (length _joint_position_goal) s)
     (dotimes (i (length _joint_position_goal))
       (sys::poke (elt _joint_position_goal i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_velocity_goal
     (write-long (length _joint_velocity_goal) s)
     (dotimes (i (length _joint_velocity_goal))
       (sys::poke (elt _joint_velocity_goal i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_position_error
     (write-long (length _joint_position_error) s)
     (dotimes (i (length _joint_position_error))
       (sys::poke (elt _joint_position_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _joint_velocity_error
     (write-long (length _joint_velocity_error) s)
     (dotimes (i (length _joint_velocity_error))
       (sys::poke (elt _joint_velocity_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; geometry_msgs/Pose _end_effector_frame
       (send _end_effector_frame :serialize s)
     ;; float64[] _cartesian_position
     (write-long (length _cartesian_position) s)
     (dotimes (i (length _cartesian_position))
       (sys::poke (elt _cartesian_position i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cartesian_velocity
     (write-long (length _cartesian_velocity) s)
     (dotimes (i (length _cartesian_velocity))
       (sys::poke (elt _cartesian_velocity i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cartesian_position_goal
     (write-long (length _cartesian_position_goal) s)
     (dotimes (i (length _cartesian_position_goal))
       (sys::poke (elt _cartesian_position_goal i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cartesian_velocity_goal
     (write-long (length _cartesian_velocity_goal) s)
     (dotimes (i (length _cartesian_velocity_goal))
       (sys::poke (elt _cartesian_velocity_goal i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cartesian_position_error
     (write-long (length _cartesian_position_error) s)
     (dotimes (i (length _cartesian_position_error))
       (sys::poke (elt _cartesian_position_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _cartesian_velocity_error
     (write-long (length _cartesian_velocity_error) s)
     (dotimes (i (length _cartesian_velocity_error))
       (sys::poke (elt _cartesian_velocity_error i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[] _joint_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_position_goal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_position_goal (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_position_goal i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_velocity_goal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_velocity_goal (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_velocity_goal i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_position_error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_position_error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_position_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _joint_velocity_error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _joint_velocity_error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _joint_velocity_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; geometry_msgs/Pose _end_effector_frame
     (send _end_effector_frame :deserialize buf ptr-) (incf ptr- (send _end_effector_frame :serialization-length))
   ;; float64[] _cartesian_position
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cartesian_position (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cartesian_position i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cartesian_velocity
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cartesian_velocity (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cartesian_velocity i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cartesian_position_goal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cartesian_position_goal (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cartesian_position_goal i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cartesian_velocity_goal
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cartesian_velocity_goal (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cartesian_velocity_goal i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cartesian_position_error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cartesian_position_error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cartesian_position_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _cartesian_velocity_error
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cartesian_velocity_error (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _cartesian_velocity_error i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(setf (get plugin_msgs::RobotState :md5sum-) "a8eadd78fb47ede9ac5103ae9c18c045")
(setf (get plugin_msgs::RobotState :datatype-) "plugin_msgs/RobotState")
(setf (get plugin_msgs::RobotState :definition-)
      "Header     header
float64[]  joint_position
float64[]  joint_velocity
float64[]  joint_position_goal
float64[]  joint_velocity_goal
float64[]  joint_position_error
float64[]  joint_velocity_error

geometry_msgs/Pose             end_effector_frame

float64[] cartesian_position
float64[] cartesian_velocity
float64[] cartesian_position_goal
float64[] cartesian_velocity_goal
float64[] cartesian_position_error
float64[] cartesian_velocity_error

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
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :plugin_msgs/RobotState "a8eadd78fb47ede9ac5103ae9c18c045")


