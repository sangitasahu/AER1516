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

class ControlLog {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.p = null;
      this.p_ref = null;
      this.p_err = null;
      this.p_err_int = null;
      this.v = null;
      this.v_ref = null;
      this.v_err = null;
      this.a_ff = null;
      this.a_fb = null;
      this.j_ff = null;
      this.j_fb = null;
      this.q = null;
      this.q_ref = null;
      this.rpy = null;
      this.rpy_ref = null;
      this.w = null;
      this.w_ref = null;
      this.F_W = null;
      this.thrust = null;
      this.throttle = null;
      this.power = null;
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
      if (initObj.hasOwnProperty('p_ref')) {
        this.p_ref = initObj.p_ref
      }
      else {
        this.p_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('p_err')) {
        this.p_err = initObj.p_err
      }
      else {
        this.p_err = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('p_err_int')) {
        this.p_err_int = initObj.p_err_int
      }
      else {
        this.p_err_int = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('v')) {
        this.v = initObj.v
      }
      else {
        this.v = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('v_ref')) {
        this.v_ref = initObj.v_ref
      }
      else {
        this.v_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('v_err')) {
        this.v_err = initObj.v_err
      }
      else {
        this.v_err = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('a_ff')) {
        this.a_ff = initObj.a_ff
      }
      else {
        this.a_ff = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('a_fb')) {
        this.a_fb = initObj.a_fb
      }
      else {
        this.a_fb = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('j_ff')) {
        this.j_ff = initObj.j_ff
      }
      else {
        this.j_ff = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('j_fb')) {
        this.j_fb = initObj.j_fb
      }
      else {
        this.j_fb = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('q')) {
        this.q = initObj.q
      }
      else {
        this.q = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('q_ref')) {
        this.q_ref = initObj.q_ref
      }
      else {
        this.q_ref = new geometry_msgs.msg.Quaternion();
      }
      if (initObj.hasOwnProperty('rpy')) {
        this.rpy = initObj.rpy
      }
      else {
        this.rpy = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('rpy_ref')) {
        this.rpy_ref = initObj.rpy_ref
      }
      else {
        this.rpy_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('w')) {
        this.w = initObj.w
      }
      else {
        this.w = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('w_ref')) {
        this.w_ref = initObj.w_ref
      }
      else {
        this.w_ref = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('F_W')) {
        this.F_W = initObj.F_W
      }
      else {
        this.F_W = new geometry_msgs.msg.Vector3();
      }
      if (initObj.hasOwnProperty('thrust')) {
        this.thrust = initObj.thrust
      }
      else {
        this.thrust = 0.0;
      }
      if (initObj.hasOwnProperty('throttle')) {
        this.throttle = initObj.throttle
      }
      else {
        this.throttle = 0.0;
      }
      if (initObj.hasOwnProperty('power')) {
        this.power = initObj.power
      }
      else {
        this.power = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlLog
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [p]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.p, buffer, bufferOffset);
    // Serialize message field [p_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.p_ref, buffer, bufferOffset);
    // Serialize message field [p_err]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.p_err, buffer, bufferOffset);
    // Serialize message field [p_err_int]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.p_err_int, buffer, bufferOffset);
    // Serialize message field [v]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.v, buffer, bufferOffset);
    // Serialize message field [v_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.v_ref, buffer, bufferOffset);
    // Serialize message field [v_err]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.v_err, buffer, bufferOffset);
    // Serialize message field [a_ff]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.a_ff, buffer, bufferOffset);
    // Serialize message field [a_fb]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.a_fb, buffer, bufferOffset);
    // Serialize message field [j_ff]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.j_ff, buffer, bufferOffset);
    // Serialize message field [j_fb]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.j_fb, buffer, bufferOffset);
    // Serialize message field [q]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.q, buffer, bufferOffset);
    // Serialize message field [q_ref]
    bufferOffset = geometry_msgs.msg.Quaternion.serialize(obj.q_ref, buffer, bufferOffset);
    // Serialize message field [rpy]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rpy, buffer, bufferOffset);
    // Serialize message field [rpy_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.rpy_ref, buffer, bufferOffset);
    // Serialize message field [w]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.w, buffer, bufferOffset);
    // Serialize message field [w_ref]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.w_ref, buffer, bufferOffset);
    // Serialize message field [F_W]
    bufferOffset = geometry_msgs.msg.Vector3.serialize(obj.F_W, buffer, bufferOffset);
    // Serialize message field [thrust]
    bufferOffset = _serializer.float64(obj.thrust, buffer, bufferOffset);
    // Serialize message field [throttle]
    bufferOffset = _serializer.float64(obj.throttle, buffer, bufferOffset);
    // Serialize message field [power]
    bufferOffset = _serializer.bool(obj.power, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlLog
    let len;
    let data = new ControlLog(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [p]
    data.p = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [p_ref]
    data.p_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [p_err]
    data.p_err = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [p_err_int]
    data.p_err_int = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [v]
    data.v = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [v_ref]
    data.v_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [v_err]
    data.v_err = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [a_ff]
    data.a_ff = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [a_fb]
    data.a_fb = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [j_ff]
    data.j_ff = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [j_fb]
    data.j_fb = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [q]
    data.q = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [q_ref]
    data.q_ref = geometry_msgs.msg.Quaternion.deserialize(buffer, bufferOffset);
    // Deserialize message field [rpy]
    data.rpy = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [rpy_ref]
    data.rpy_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [w]
    data.w = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [w_ref]
    data.w_ref = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [F_W]
    data.F_W = geometry_msgs.msg.Vector3.deserialize(buffer, bufferOffset);
    // Deserialize message field [thrust]
    data.thrust = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [throttle]
    data.throttle = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [power]
    data.power = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 465;
  }

  static datatype() {
    // Returns string type for a message object
    return 'snapstack_msgs/ControlLog';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0127ad6ed84894e6d10c273726d40503';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # The outer loop trajectory tracker generates this msg for analysis / debugging
    
    Header header
    
    # position signals
    geometry_msgs/Vector3 p
    geometry_msgs/Vector3 p_ref
    geometry_msgs/Vector3 p_err
    geometry_msgs/Vector3 p_err_int
    
    # velocity signals
    geometry_msgs/Vector3 v
    geometry_msgs/Vector3 v_ref
    geometry_msgs/Vector3 v_err
    
    # acceleration signals
    geometry_msgs/Vector3 a_ff
    geometry_msgs/Vector3 a_fb
    
    # jerk signals
    geometry_msgs/Vector3 j_ff
    geometry_msgs/Vector3 j_fb
    
    # attitude signals
    geometry_msgs/Quaternion q
    geometry_msgs/Quaternion q_ref
    geometry_msgs/Vector3 rpy
    geometry_msgs/Vector3 rpy_ref
    
    # angular rate signals
    geometry_msgs/Vector3 w
    geometry_msgs/Vector3 w_ref
    
    geometry_msgs/Vector3 F_W # Desired total force [N], expressed in world
    float64 thrust   # total desired force [N]
    float64 throttle # percent throttle sent to each motor
    
    bool power # true if motors should be able to spin
    
    # TODO: add outer (and inner?) parameters
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
    const resolved = new ControlLog(null);
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

    if (msg.p_ref !== undefined) {
      resolved.p_ref = geometry_msgs.msg.Vector3.Resolve(msg.p_ref)
    }
    else {
      resolved.p_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.p_err !== undefined) {
      resolved.p_err = geometry_msgs.msg.Vector3.Resolve(msg.p_err)
    }
    else {
      resolved.p_err = new geometry_msgs.msg.Vector3()
    }

    if (msg.p_err_int !== undefined) {
      resolved.p_err_int = geometry_msgs.msg.Vector3.Resolve(msg.p_err_int)
    }
    else {
      resolved.p_err_int = new geometry_msgs.msg.Vector3()
    }

    if (msg.v !== undefined) {
      resolved.v = geometry_msgs.msg.Vector3.Resolve(msg.v)
    }
    else {
      resolved.v = new geometry_msgs.msg.Vector3()
    }

    if (msg.v_ref !== undefined) {
      resolved.v_ref = geometry_msgs.msg.Vector3.Resolve(msg.v_ref)
    }
    else {
      resolved.v_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.v_err !== undefined) {
      resolved.v_err = geometry_msgs.msg.Vector3.Resolve(msg.v_err)
    }
    else {
      resolved.v_err = new geometry_msgs.msg.Vector3()
    }

    if (msg.a_ff !== undefined) {
      resolved.a_ff = geometry_msgs.msg.Vector3.Resolve(msg.a_ff)
    }
    else {
      resolved.a_ff = new geometry_msgs.msg.Vector3()
    }

    if (msg.a_fb !== undefined) {
      resolved.a_fb = geometry_msgs.msg.Vector3.Resolve(msg.a_fb)
    }
    else {
      resolved.a_fb = new geometry_msgs.msg.Vector3()
    }

    if (msg.j_ff !== undefined) {
      resolved.j_ff = geometry_msgs.msg.Vector3.Resolve(msg.j_ff)
    }
    else {
      resolved.j_ff = new geometry_msgs.msg.Vector3()
    }

    if (msg.j_fb !== undefined) {
      resolved.j_fb = geometry_msgs.msg.Vector3.Resolve(msg.j_fb)
    }
    else {
      resolved.j_fb = new geometry_msgs.msg.Vector3()
    }

    if (msg.q !== undefined) {
      resolved.q = geometry_msgs.msg.Quaternion.Resolve(msg.q)
    }
    else {
      resolved.q = new geometry_msgs.msg.Quaternion()
    }

    if (msg.q_ref !== undefined) {
      resolved.q_ref = geometry_msgs.msg.Quaternion.Resolve(msg.q_ref)
    }
    else {
      resolved.q_ref = new geometry_msgs.msg.Quaternion()
    }

    if (msg.rpy !== undefined) {
      resolved.rpy = geometry_msgs.msg.Vector3.Resolve(msg.rpy)
    }
    else {
      resolved.rpy = new geometry_msgs.msg.Vector3()
    }

    if (msg.rpy_ref !== undefined) {
      resolved.rpy_ref = geometry_msgs.msg.Vector3.Resolve(msg.rpy_ref)
    }
    else {
      resolved.rpy_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.w !== undefined) {
      resolved.w = geometry_msgs.msg.Vector3.Resolve(msg.w)
    }
    else {
      resolved.w = new geometry_msgs.msg.Vector3()
    }

    if (msg.w_ref !== undefined) {
      resolved.w_ref = geometry_msgs.msg.Vector3.Resolve(msg.w_ref)
    }
    else {
      resolved.w_ref = new geometry_msgs.msg.Vector3()
    }

    if (msg.F_W !== undefined) {
      resolved.F_W = geometry_msgs.msg.Vector3.Resolve(msg.F_W)
    }
    else {
      resolved.F_W = new geometry_msgs.msg.Vector3()
    }

    if (msg.thrust !== undefined) {
      resolved.thrust = msg.thrust;
    }
    else {
      resolved.thrust = 0.0
    }

    if (msg.throttle !== undefined) {
      resolved.throttle = msg.throttle;
    }
    else {
      resolved.throttle = 0.0
    }

    if (msg.power !== undefined) {
      resolved.power = msg.power;
    }
    else {
      resolved.power = false
    }

    return resolved;
    }
};

module.exports = ControlLog;
