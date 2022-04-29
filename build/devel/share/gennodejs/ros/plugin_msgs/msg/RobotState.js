// Auto-generated. Do not edit!

// (in-package plugin_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.joint_position = null;
      this.joint_velocity = null;
      this.joint_position_goal = null;
      this.joint_velocity_goal = null;
      this.joint_position_error = null;
      this.joint_velocity_error = null;
      this.end_effector_frame = null;
      this.cartesian_position = null;
      this.cartesian_velocity = null;
      this.cartesian_position_goal = null;
      this.cartesian_velocity_goal = null;
      this.cartesian_position_error = null;
      this.cartesian_velocity_error = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('joint_position')) {
        this.joint_position = initObj.joint_position
      }
      else {
        this.joint_position = [];
      }
      if (initObj.hasOwnProperty('joint_velocity')) {
        this.joint_velocity = initObj.joint_velocity
      }
      else {
        this.joint_velocity = [];
      }
      if (initObj.hasOwnProperty('joint_position_goal')) {
        this.joint_position_goal = initObj.joint_position_goal
      }
      else {
        this.joint_position_goal = [];
      }
      if (initObj.hasOwnProperty('joint_velocity_goal')) {
        this.joint_velocity_goal = initObj.joint_velocity_goal
      }
      else {
        this.joint_velocity_goal = [];
      }
      if (initObj.hasOwnProperty('joint_position_error')) {
        this.joint_position_error = initObj.joint_position_error
      }
      else {
        this.joint_position_error = [];
      }
      if (initObj.hasOwnProperty('joint_velocity_error')) {
        this.joint_velocity_error = initObj.joint_velocity_error
      }
      else {
        this.joint_velocity_error = [];
      }
      if (initObj.hasOwnProperty('end_effector_frame')) {
        this.end_effector_frame = initObj.end_effector_frame
      }
      else {
        this.end_effector_frame = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('cartesian_position')) {
        this.cartesian_position = initObj.cartesian_position
      }
      else {
        this.cartesian_position = [];
      }
      if (initObj.hasOwnProperty('cartesian_velocity')) {
        this.cartesian_velocity = initObj.cartesian_velocity
      }
      else {
        this.cartesian_velocity = [];
      }
      if (initObj.hasOwnProperty('cartesian_position_goal')) {
        this.cartesian_position_goal = initObj.cartesian_position_goal
      }
      else {
        this.cartesian_position_goal = [];
      }
      if (initObj.hasOwnProperty('cartesian_velocity_goal')) {
        this.cartesian_velocity_goal = initObj.cartesian_velocity_goal
      }
      else {
        this.cartesian_velocity_goal = [];
      }
      if (initObj.hasOwnProperty('cartesian_position_error')) {
        this.cartesian_position_error = initObj.cartesian_position_error
      }
      else {
        this.cartesian_position_error = [];
      }
      if (initObj.hasOwnProperty('cartesian_velocity_error')) {
        this.cartesian_velocity_error = initObj.cartesian_velocity_error
      }
      else {
        this.cartesian_velocity_error = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotState
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [joint_position]
    bufferOffset = _arraySerializer.float64(obj.joint_position, buffer, bufferOffset, null);
    // Serialize message field [joint_velocity]
    bufferOffset = _arraySerializer.float64(obj.joint_velocity, buffer, bufferOffset, null);
    // Serialize message field [joint_position_goal]
    bufferOffset = _arraySerializer.float64(obj.joint_position_goal, buffer, bufferOffset, null);
    // Serialize message field [joint_velocity_goal]
    bufferOffset = _arraySerializer.float64(obj.joint_velocity_goal, buffer, bufferOffset, null);
    // Serialize message field [joint_position_error]
    bufferOffset = _arraySerializer.float64(obj.joint_position_error, buffer, bufferOffset, null);
    // Serialize message field [joint_velocity_error]
    bufferOffset = _arraySerializer.float64(obj.joint_velocity_error, buffer, bufferOffset, null);
    // Serialize message field [end_effector_frame]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.end_effector_frame, buffer, bufferOffset);
    // Serialize message field [cartesian_position]
    bufferOffset = _arraySerializer.float64(obj.cartesian_position, buffer, bufferOffset, null);
    // Serialize message field [cartesian_velocity]
    bufferOffset = _arraySerializer.float64(obj.cartesian_velocity, buffer, bufferOffset, null);
    // Serialize message field [cartesian_position_goal]
    bufferOffset = _arraySerializer.float64(obj.cartesian_position_goal, buffer, bufferOffset, null);
    // Serialize message field [cartesian_velocity_goal]
    bufferOffset = _arraySerializer.float64(obj.cartesian_velocity_goal, buffer, bufferOffset, null);
    // Serialize message field [cartesian_position_error]
    bufferOffset = _arraySerializer.float64(obj.cartesian_position_error, buffer, bufferOffset, null);
    // Serialize message field [cartesian_velocity_error]
    bufferOffset = _arraySerializer.float64(obj.cartesian_velocity_error, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotState
    let len;
    let data = new RobotState(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [joint_position]
    data.joint_position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_velocity]
    data.joint_velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_position_goal]
    data.joint_position_goal = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_velocity_goal]
    data.joint_velocity_goal = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_position_error]
    data.joint_position_error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [joint_velocity_error]
    data.joint_velocity_error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [end_effector_frame]
    data.end_effector_frame = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [cartesian_position]
    data.cartesian_position = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cartesian_velocity]
    data.cartesian_velocity = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cartesian_position_goal]
    data.cartesian_position_goal = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cartesian_velocity_goal]
    data.cartesian_velocity_goal = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cartesian_position_error]
    data.cartesian_position_error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [cartesian_velocity_error]
    data.cartesian_velocity_error = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 8 * object.joint_position.length;
    length += 8 * object.joint_velocity.length;
    length += 8 * object.joint_position_goal.length;
    length += 8 * object.joint_velocity_goal.length;
    length += 8 * object.joint_position_error.length;
    length += 8 * object.joint_velocity_error.length;
    length += 8 * object.cartesian_position.length;
    length += 8 * object.cartesian_velocity.length;
    length += 8 * object.cartesian_position_goal.length;
    length += 8 * object.cartesian_velocity_goal.length;
    length += 8 * object.cartesian_position_error.length;
    length += 8 * object.cartesian_velocity_error.length;
    return length + 104;
  }

  static datatype() {
    // Returns string type for a message object
    return 'plugin_msgs/RobotState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a8eadd78fb47ede9ac5103ae9c18c045';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header     header
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotState(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.joint_position !== undefined) {
      resolved.joint_position = msg.joint_position;
    }
    else {
      resolved.joint_position = []
    }

    if (msg.joint_velocity !== undefined) {
      resolved.joint_velocity = msg.joint_velocity;
    }
    else {
      resolved.joint_velocity = []
    }

    if (msg.joint_position_goal !== undefined) {
      resolved.joint_position_goal = msg.joint_position_goal;
    }
    else {
      resolved.joint_position_goal = []
    }

    if (msg.joint_velocity_goal !== undefined) {
      resolved.joint_velocity_goal = msg.joint_velocity_goal;
    }
    else {
      resolved.joint_velocity_goal = []
    }

    if (msg.joint_position_error !== undefined) {
      resolved.joint_position_error = msg.joint_position_error;
    }
    else {
      resolved.joint_position_error = []
    }

    if (msg.joint_velocity_error !== undefined) {
      resolved.joint_velocity_error = msg.joint_velocity_error;
    }
    else {
      resolved.joint_velocity_error = []
    }

    if (msg.end_effector_frame !== undefined) {
      resolved.end_effector_frame = geometry_msgs.msg.Pose.Resolve(msg.end_effector_frame)
    }
    else {
      resolved.end_effector_frame = new geometry_msgs.msg.Pose()
    }

    if (msg.cartesian_position !== undefined) {
      resolved.cartesian_position = msg.cartesian_position;
    }
    else {
      resolved.cartesian_position = []
    }

    if (msg.cartesian_velocity !== undefined) {
      resolved.cartesian_velocity = msg.cartesian_velocity;
    }
    else {
      resolved.cartesian_velocity = []
    }

    if (msg.cartesian_position_goal !== undefined) {
      resolved.cartesian_position_goal = msg.cartesian_position_goal;
    }
    else {
      resolved.cartesian_position_goal = []
    }

    if (msg.cartesian_velocity_goal !== undefined) {
      resolved.cartesian_velocity_goal = msg.cartesian_velocity_goal;
    }
    else {
      resolved.cartesian_velocity_goal = []
    }

    if (msg.cartesian_position_error !== undefined) {
      resolved.cartesian_position_error = msg.cartesian_position_error;
    }
    else {
      resolved.cartesian_position_error = []
    }

    if (msg.cartesian_velocity_error !== undefined) {
      resolved.cartesian_velocity_error = msg.cartesian_velocity_error;
    }
    else {
      resolved.cartesian_velocity_error = []
    }

    return resolved;
    }
};

module.exports = RobotState;
