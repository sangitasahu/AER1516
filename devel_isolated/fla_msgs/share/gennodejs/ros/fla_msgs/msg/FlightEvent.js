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

class FlightEvent {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.event_id = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('event_id')) {
        this.event_id = initObj.event_id
      }
      else {
        this.event_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type FlightEvent
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [event_id]
    bufferOffset = _serializer.uint8(obj.event_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type FlightEvent
    let len;
    let data = new FlightEvent(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [event_id]
    data.event_id = _deserializer.uint8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 1;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fla_msgs/FlightEvent';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4110787c7e1d3f4e3706ce25b95d01e2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 BEGIN=0
    uint8 INITIALIZED=1
    uint8 ALTITUDE_STABLE=2
    uint8 GOAL_REACHED=3
    uint8 LAND_COMMANDED=4
    uint8 HOVER_COMMANDED=5
    uint8 RESET_COMMANDED=6
    uint8 LANDED=7
    uint8 RC_COMM_LOSS=8
    uint8 TARGET_DETECTED_OUT_OF_RANGE=9
    uint8 TARGET_DETECTED_IN_RANGE=10
    uint8 SEARCH_COMPLETE=11
    uint8 RTL_COMMANDED=12
    uint8 FLIGHTEVENT_MAX=13
    std_msgs/Header header
    uint8 event_id
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new FlightEvent(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.event_id !== undefined) {
      resolved.event_id = msg.event_id;
    }
    else {
      resolved.event_id = 0
    }

    return resolved;
    }
};

// Constants for message
FlightEvent.Constants = {
  BEGIN: 0,
  INITIALIZED: 1,
  ALTITUDE_STABLE: 2,
  GOAL_REACHED: 3,
  LAND_COMMANDED: 4,
  HOVER_COMMANDED: 5,
  RESET_COMMANDED: 6,
  LANDED: 7,
  RC_COMM_LOSS: 8,
  TARGET_DETECTED_OUT_OF_RANGE: 9,
  TARGET_DETECTED_IN_RANGE: 10,
  SEARCH_COMPLETE: 11,
  RTL_COMMANDED: 12,
  FLIGHTEVENT_MAX: 13,
}

module.exports = FlightEvent;
