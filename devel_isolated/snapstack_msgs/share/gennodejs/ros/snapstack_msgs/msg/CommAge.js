// Auto-generated. Do not edit!

// (in-package snapstack_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class CommAge {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.vicon_age_secs = null;
      this.goal_age_secs = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('vicon_age_secs')) {
        this.vicon_age_secs = initObj.vicon_age_secs
      }
      else {
        this.vicon_age_secs = 0.0;
      }
      if (initObj.hasOwnProperty('goal_age_secs')) {
        this.goal_age_secs = initObj.goal_age_secs
      }
      else {
        this.goal_age_secs = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CommAge
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [vicon_age_secs]
    bufferOffset = _serializer.float32(obj.vicon_age_secs, buffer, bufferOffset);
    // Serialize message field [goal_age_secs]
    bufferOffset = _serializer.float32(obj.goal_age_secs, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CommAge
    let len;
    let data = new CommAge(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [vicon_age_secs]
    data.vicon_age_secs = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [goal_age_secs]
    data.goal_age_secs = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'snapstack_msgs/CommAge';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '37ee5d091cfb61db7a1dcd668b6244ed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float32 vicon_age_secs
    float32 goal_age_secs
    
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
    const resolved = new CommAge(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.vicon_age_secs !== undefined) {
      resolved.vicon_age_secs = msg.vicon_age_secs;
    }
    else {
      resolved.vicon_age_secs = 0.0
    }

    if (msg.goal_age_secs !== undefined) {
      resolved.goal_age_secs = msg.goal_age_secs;
    }
    else {
      resolved.goal_age_secs = 0.0
    }

    return resolved;
    }
};

module.exports = CommAge;
