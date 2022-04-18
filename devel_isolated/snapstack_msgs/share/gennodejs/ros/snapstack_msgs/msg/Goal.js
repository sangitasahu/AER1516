// Auto-generated. Do not edit!

// (in-package snapstack_msgs.msg)


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

class Goal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.p = null;
      this.v = null;
      this.a = null;
      this.j = null;
      this.yaw = null;
      this.dyaw = null;
      this.power = null;
      this.mode_xy = null;
      this.mode_z = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('p')) {
        this.p = initObj.p
      }
      else {
        this.p = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('j')) {
        this.j = initObj.j
      }
      else {
        this.j = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('dyaw')) {
        this.dyaw = initObj.dyaw
      }
      else {
        this.dyaw = 0.0;
      }
      if (initObj.hasOwnProperty('power')) {
        this.power = initObj.power
      }
      else {
        this.power = false;
      }
      if (initObj.hasOwnProperty('mode_xy')) {
        this.mode_xy = initObj.mode_xy
      }
      else {
        this.mode_xy = 0;
      }
      if (initObj.hasOwnProperty('mode_z')) {
        this.mode_z = initObj.mode_z
      }
      else {
        this.mode_z = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Goal
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [p]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.p, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.v, buffer, bufferOffset);
    // Serialize message field [a]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.a, buffer, bufferOffset);
    // Serialize message field [j]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.j, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [dyaw]
    bufferOffset = _serializer.float64(obj.dyaw, buffer, bufferOffset);
    // Serialize message field [power]
    bufferOffset = _serializer.bool(obj.power, buffer, bufferOffset);
    // Serialize message field [mode_xy]
    bufferOffset = _serializer.uint8(obj.mode_xy, buffer, bufferOffset);
    // Serialize message field [mode_z]
    bufferOffset = _serializer.uint8(obj.mode_z, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Goal
    let len;
    let data = new Goal(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [p]
    data.p = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [a]
    data.a = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [j]
    data.j = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [dyaw]
    data.dyaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [power]
    data.power = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [mode_xy]
    data.mode_xy = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [mode_z]
    data.mode_z = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 115;
  }

  static datatype() {
    // Returns string type for a message object
    return 'snapstack_msgs/Goal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '29f7a5b62089bdabd9ea1780f356bc8b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Use this message to command the outer loop to track
    # a trajectory generated from a high-level trajectory planner.
    
    Header header
    
    # Current time-slice of desired trajectory
    geometry_msgs/Vector3 p # position
    geometry_msgs/Vector3 v # velocity
    geometry_msgs/Vector3 a # acceleration
    geometry_msgs/Vector3 j # jerk
    
    float64 yaw # heading / yaw angle
    float64 dyaw # d/dt{unrolled, unpitched body heading w.r.t world}
    # n.b., recall that dyaw = d/dt{psi} != r. Angular heading rate r is defined in
    # the body frame, but yaw is the heading of the local level frame w.r.t world.
    # For slow, nearly-level flight, dyaw ~= r. For more agile flight, it will
    # be useful to make sure you are commanding the correct quantity.
    # See, e.g., eq (7) and (8) in 
    # https://scholarsarchive.byu.edu/cgi/viewcontent.cgi?article=2324&context=facpub
    
    bool power # true if motors should be able to spin
    
    # Trajectory tracking mode constants
    uint8 MODE_POSITION_CONTROL     = 0
    uint8 MODE_VELOCITY_CONTROL     = 1
    uint8 MODE_ACCELERATION_CONTROL = 2
    
    # Trajectory tracking mode for x/y and z components.
    # The default is POSITION control, which uses position and velocity error
    # to calculate the control effort. VELOCITY control only uses vel error.
    # ACCELERATION mode does not use tracking error and could be used to provide
    # a control signal computed from something other than the default PID cntrl.
    uint8 mode_xy
    uint8 mode_z
    
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
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Goal(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.p !== undefined) {
      resolved.p = geometry_msgs.msg.Vector3.Resolve(msg.p)
    }
    else {
      resolved.p = new geometry_msgs.msg.Vector3()
    }

    if (msg.v !== undefined) {
      resolved.v = geometry_msgs.msg.Vector3.Resolve(msg.v)
    }
    else {
      resolved.v = new geometry_msgs.msg.Vector3()
    }

    if (msg.a !== undefined) {
      resolved.a = geometry_msgs.msg.Vector3.Resolve(msg.a)
    }
    else {
      resolved.a = new geometry_msgs.msg.Vector3()
    }

    if (msg.j !== undefined) {
      resolved.j = geometry_msgs.msg.Vector3.Resolve(msg.j)
    }
    else {
      resolved.j = new geometry_msgs.msg.Vector3()
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.dyaw !== undefined) {
      resolved.dyaw = msg.dyaw;
    }
    else {
      resolved.dyaw = 0.0
    }

    if (msg.power !== undefined) {
      resolved.power = msg.power;
    }
    else {
      resolved.power = false
    }

    if (msg.mode_xy !== undefined) {
      resolved.mode_xy = msg.mode_xy;
    }
    else {
      resolved.mode_xy = 0
    }

    if (msg.mode_z !== undefined) {
      resolved.mode_z = msg.mode_z;
    }
    else {
      resolved.mode_z = 0
    }

    return resolved;
    }
};

// Constants for message
Goal.Constants = {
  MODE_POSITION_CONTROL: 0,
  MODE_VELOCITY_CONTROL: 1,
  MODE_ACCELERATION_CONTROL: 2,
}

module.exports = Goal;
