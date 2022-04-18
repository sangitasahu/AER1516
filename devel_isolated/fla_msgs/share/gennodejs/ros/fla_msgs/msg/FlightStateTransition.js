// Auto-generated. Do not edit!

// (in-package fla_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class FlightStateTransition {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.from_state = null;
      this.to_state = null;
      this.from_state_id = null;
      this.to_state_id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('from_state')) {
        this.from_state = initObj.from_state
      }
      else {
        this.from_state = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('to_state')) {
        this.to_state = initObj.to_state
      }
      else {
        this.to_state = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('from_state_id')) {
        this.from_state_id = initObj.from_state_id
      }
      else {
        this.from_state_id = 0;
      }
      if (initObj.hasOwnProperty('to_state_id')) {
        this.to_state_id = initObj.to_state_id
      }
      else {
        this.to_state_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlightStateTransition
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [from_state]
    bufferOffset = std_msgs.msg.String.serialize(obj.from_state, buffer, bufferOffset);
    // Serialize message field [to_state]
    bufferOffset = std_msgs.msg.String.serialize(obj.to_state, buffer, bufferOffset);
    // Serialize message field [from_state_id]
    bufferOffset = _serializer.uint8(obj.from_state_id, buffer, bufferOffset);
    // Serialize message field [to_state_id]
    bufferOffset = _serializer.uint8(obj.to_state_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlightStateTransition
    let len;
    let data = new FlightStateTransition(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [from_state]
    data.from_state = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [to_state]
    data.to_state = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [from_state_id]
    data.from_state_id = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [to_state_id]
    data.to_state_id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += std_msgs.msg.String.getMessageSize(object.from_state);
    length += std_msgs.msg.String.getMessageSize(object.to_state);
    return length + 2;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fla_msgs/FlightStateTransition';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '994f9c657664b2f00c832165b7503f8a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 IDLE=0
    uint8 INITIALIZING=1
    uint8 TAKING_OFF=2
    uint8 FOLLOWING_PATH=3
    uint8 HOVERING=4
    uint8 LANDING=5
    uint8 RETURN_TO_LAUNCH=6
    uint8 LANDED=7
    uint8 TARGET_SEARCH=8
    uint8 TARGET_APPROACH=9
    uint8 LANDING_SEARCH=10
    uint8 LANDING_APPROACH=11
    
    std_msgs/Header header
    std_msgs/String from_state
    std_msgs/String to_state
    uint8 from_state_id
    uint8 to_state_id
    
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
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlightStateTransition(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.from_state !== undefined) {
      resolved.from_state = std_msgs.msg.String.Resolve(msg.from_state)
    }
    else {
      resolved.from_state = new std_msgs.msg.String()
    }

    if (msg.to_state !== undefined) {
      resolved.to_state = std_msgs.msg.String.Resolve(msg.to_state)
    }
    else {
      resolved.to_state = new std_msgs.msg.String()
    }

    if (msg.from_state_id !== undefined) {
      resolved.from_state_id = msg.from_state_id;
    }
    else {
      resolved.from_state_id = 0
    }

    if (msg.to_state_id !== undefined) {
      resolved.to_state_id = msg.to_state_id;
    }
    else {
      resolved.to_state_id = 0
    }

    return resolved;
    }
};

// Constants for message
FlightStateTransition.Constants = {
  IDLE: 0,
  INITIALIZING: 1,
  TAKING_OFF: 2,
  FOLLOWING_PATH: 3,
  HOVERING: 4,
  LANDING: 5,
  RETURN_TO_LAUNCH: 6,
  LANDED: 7,
  TARGET_SEARCH: 8,
  TARGET_APPROACH: 9,
  LANDING_SEARCH: 10,
  LANDING_APPROACH: 11,
}

module.exports = FlightStateTransition;
