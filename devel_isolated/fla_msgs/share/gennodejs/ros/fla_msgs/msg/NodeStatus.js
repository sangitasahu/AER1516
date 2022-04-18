// Auto-generated. Do not edit!

// (in-package fla_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class NodeStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.pid = null;
      this.status = null;
      this.arg = null;
      this.load = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('pid')) {
        this.pid = initObj.pid
      }
      else {
        this.pid = 0;
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
      if (initObj.hasOwnProperty('arg')) {
        this.arg = initObj.arg
      }
      else {
        this.arg = 0;
      }
      if (initObj.hasOwnProperty('load')) {
        this.load = initObj.load
      }
      else {
        this.load = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NodeStatus
    // Serialize message field [id]
    bufferOffset = _serializer.uint32(obj.id, buffer, bufferOffset);
    // Serialize message field [pid]
    bufferOffset = _serializer.uint32(obj.pid, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint8(obj.status, buffer, bufferOffset);
    // Serialize message field [arg]
    bufferOffset = _serializer.int32(obj.arg, buffer, bufferOffset);
    // Serialize message field [load]
    bufferOffset = _serializer.float64(obj.load, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NodeStatus
    let len;
    let data = new NodeStatus(null);
    // Deserialize message field [id]
    data.id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pid]
    data.pid = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [arg]
    data.arg = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [load]
    data.load = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'fla_msgs/NodeStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bc0e9b8f7a292a18c6b50b43586a86d8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    uint8 INACTIVE=1
    uint8 STARTING=2
    uint8 INIT=3
    uint8 READY=4
    uint8 ALARM=5
    uint8 FAIL=6
    uint8 SILENT=7
    uint32 id
    uint32 pid
    uint8 status
    int32 arg
    float64 load
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NodeStatus(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.pid !== undefined) {
      resolved.pid = msg.pid;
    }
    else {
      resolved.pid = 0
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    if (msg.arg !== undefined) {
      resolved.arg = msg.arg;
    }
    else {
      resolved.arg = 0
    }

    if (msg.load !== undefined) {
      resolved.load = msg.load;
    }
    else {
      resolved.load = 0.0
    }

    return resolved;
    }
};

// Constants for message
NodeStatus.Constants = {
  INACTIVE: 1,
  STARTING: 2,
  INIT: 3,
  READY: 4,
  ALARM: 5,
  FAIL: 6,
  SILENT: 7,
}

module.exports = NodeStatus;
